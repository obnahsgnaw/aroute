package road

import (
	"errors"
	"fmt"
	"github.com/jonas-p/go-shp"
	"github.com/obnahsgnaw/aroute/astart"
	"math"
	"sync"
	"time"
)

// Network road network
type Network struct {
	nodes       map[int]*Node // 所有路的点
	edges       map[int]*Edge // 所有路
	fields      []shp.Field   // 属性字段
	maxDistance float64       // 寻点的最大距离
	temps       map[int64]*Temp
	tmpLock     sync.Mutex
}

func newNetwork(o ...Option) *Network {
	s := &Network{
		nodes:       make(map[int]*Node),
		edges:       make(map[int]*Edge),
		temps:       make(map[int64]*Temp),
		maxDistance: 100,
	}

	s.with(o...)

	return s
}

func (s *Network) with(o ...Option) {
	for _, fn := range o {
		fn(s)
	}
}

// Load 转换路网SHP文件为图结构
func (s *Network) Load(shpPath string) error {
	shape, err := shp.Open(shpPath)
	if err != nil {
		return errors.New("open shp file failed, err=" + err.Error())
	}
	defer func() { _ = shape.Close() }()
	s.fields = shape.Fields()

	// 用于快速查找节点的空间索引(简化实现)
	nodeIndex := make(map[string]*Node)
	currentNodeID := 1
	currentEdgeID := 1

	for shape.Next() {
		n, p := shape.Shape()
		switch geom := p.(type) {
		case *shp.PolyLine:
			attr := make(map[string]string)
			for k, f := range s.fields {
				val := shape.ReadAttribute(n, k)
				attr[f.String()] = val
			}
			s.processSegment(geom, &currentNodeID, &currentEdgeID, nodeIndex, &Attribute{values: attr})

		default:
			//log.Printf("忽略不支持的类型: %T", p)
		}
	}

	return nil
}

func (s *Network) processSegment(line *shp.PolyLine, currentNodeID *int, currentEdgeID *int, nodeIndex map[string]*Node, attr *Attribute) {
	// 处理每条线段(假设每条线段是道路的一段)
	for part := 0; part < int(line.NumParts); part++ {
		startIdx := line.Parts[part]
		endIdx := line.NumPoints
		if part < int(line.NumParts)-1 {
			endIdx = line.Parts[part+1]
		}

		// 获取线段的所有点
		points := line.Points[startIdx:endIdx]
		if len(points) < 2 {
			continue
		}

		// 处理线段上的节点
		var prevNode *Node
		for i, point := range points {
			// 创建或获取节点
			key := fmt.Sprintf("%.6f,%.6f", point.X, point.Y)
			node, exists := nodeIndex[key]
			if !exists {
				node = &Node{
					Point: Point{
						X: point.X,
						Y: point.Y,
					},
					ID: *currentNodeID,
					m:  s,
				}
				nodeIndex[key] = node
				s.nodes[node.ID] = node
				*currentNodeID++
			}

			if i > 0 && prevNode != nil {
				length := pointToPointDistance(Point{X: point.X, Y: point.Y}, Point{X: prevNode.X, Y: prevNode.Y})

				edge := &Edge{
					ID:     *currentEdgeID,
					From:   prevNode,
					To:     node,
					Length: length,
					Attr:   attr,
				}
				s.edges[edge.ID] = edge

				prevNode.OutEdges = append(prevNode.OutEdges, edge)
				*currentEdgeID++
				node.OutEdges = append(node.OutEdges, edge)
			}

			prevNode = node
		}
	}
}

func (s *Network) Find(points ...Point) (path []*Node, distance float64, found bool) {
	if len(points) < 2 {
		return
	}
	if len(points) == 2 {
		return s.find(genSessionId(), points[0], points[1])
	}

	temp := s.GetTemp(genSessionId())
	defer s.DelTemp(temp.sessionId)
	startNode, _, _ := s.connectToNetwork(points[0].X, points[0].Y, s.maxDistance, temp)
	if startNode == nil {
		return
	}
	var endNode *Node
	for _, point := range points[1:] {
		if endNode != nil {
			startNode = endNode
		}
		endNode, _, _ = s.connectToNetwork(point.X, point.Y, s.maxDistance, temp)
		if endNode == nil {
			return
		}
		path1, distance1, found1 := s.findNodePath(temp.SessionId(), startNode, endNode)
		if !found1 {
			return
		}
		if len(path) == 0 {
			path = append(path, path1...)
		} else {
			path = append(path, path1[1:]...)
		}
		distance += distance1
	}

	found = true
	return
}

func (s *Network) find(sessionId int64, start, end Point) (path []*Node, distance float64, found bool) {
	temp := s.GetTemp(sessionId)
	defer s.DelTemp(sessionId)
	startNode, _, _ := s.connectToNetwork(start.X, start.Y, s.maxDistance, temp)
	endNode, _, _ := s.connectToNetwork(end.X, end.Y, s.maxDistance, temp)
	if startNode == nil || endNode == nil {
		return
	}

	return s.findNodePath(sessionId, startNode, endNode)
}

func (s *Network) FindNodePath(startNode, endNode *Node) (path []*Node, distance float64, found bool) {
	return s.findNodePath(genSessionId(), startNode, endNode)
}

func (s *Network) findNodePath(sessionId int64, startNode, endNode *Node) (path []*Node, distance float64, found bool) {
	var path1 []astar.Pather
	path1, distance, found = astar.Path(sessionId, startNode, endNode)
	if found {
		path = reverseSlice(path1)
	}
	return
}

func (s *Network) connectToNetwork(x, y float64, maxDistance float64, temp *Temp) (*Node, *Edge, error) {
	// 1. 查找最近的道路边
	var nearestEdge *Edge
	var projectionPoint [2]float64
	minDist := math.MaxFloat64

	for _, edge := range s.edges {
		if temp.IsUselessEdge(edge.ID) {
			continue
		}
		dist, projPoint, ok := PointToLineDistanceAndFootPoint(Point{X: x, Y: y}, Point{X: edge.From.X, Y: edge.From.Y}, Point{X: edge.To.X, Y: edge.To.Y})

		if ok && dist < minDist {
			minDist = dist
			nearestEdge = edge
			projectionPoint = [2]float64{projPoint.X, projPoint.Y}
		}
	}

	for _, edge := range temp.edges {
		dist, projPoint, ok := PointToLineDistanceAndFootPoint(Point{X: x, Y: y}, Point{X: edge.From.X, Y: edge.From.Y}, Point{X: edge.To.X, Y: edge.To.Y})
		if ok && dist < minDist {
			minDist = dist
			nearestEdge = edge
			projectionPoint = [2]float64{projPoint.X, projPoint.Y}
		}
	}

	if nearestEdge == nil || minDist > maxDistance {
		return nil, nil, fmt.Errorf("没有找到附近的路网(最近距离%.2f米)", minDist)
	}

	// 2. 如果投影点接近现有节点，直接使用该节点
	threshold := 0.1 // 10厘米内视为相同点
	fromDist := pointToPointDistance(Point{X: projectionPoint[0], Y: projectionPoint[1]}, Point{X: nearestEdge.From.X, Y: nearestEdge.From.Y})
	toDist := pointToPointDistance(Point{X: projectionPoint[0], Y: projectionPoint[1]}, Point{X: nearestEdge.To.X, Y: nearestEdge.To.Y})

	if fromDist < threshold {
		return nearestEdge.From, nearestEdge, nil
	}
	if toDist < threshold {
		return nearestEdge.To, nearestEdge, nil
	}

	// 3. 创建新节点并将其连接到路网
	newNodeId := len(s.nodes) + 1
	if tempNodesLen := temp.NodeLen(); tempNodesLen > 0 {
		newNodeId += tempNodesLen / 3
	}
	newNode := &Node{
		Point: Point{
			X: projectionPoint[0],
			Y: projectionPoint[1],
		},
		ID: newNodeId,
		m:  s,
	}
	p1 := *nearestEdge.From
	p2 := *nearestEdge.To
	nearestEdgeFromNode := &p1
	if v, ok := temp.GetNode(nearestEdgeFromNode.ID); ok {
		nearestEdgeFromNode = v
	}
	nearestEdgeToNode := &p2
	if v, ok := temp.GetNode(nearestEdgeToNode.ID); ok {
		nearestEdgeToNode = v
	}
	temp.AddNode(newNode)
	temp.AddNode(nearestEdgeFromNode)
	temp.AddNode(nearestEdgeToNode)

	// 将新节点连接到原边的两个端点
	length1 := pointToPointDistance(Point{X: newNode.X, Y: newNode.Y}, Point{X: nearestEdgeFromNode.X, Y: nearestEdgeFromNode.Y})
	edge1 := &Edge{
		ID:     len(s.edges) + 1 + temp.EdgeLen(),
		From:   nearestEdgeFromNode,
		To:     newNode,
		Length: length1,
		Attr:   nearestEdge.Attr,
	}
	temp.AddEdge(edge1)
	nearestEdgeFromNode.OutEdges = append(nearestEdgeFromNode.OutEdges, edge1)
	newNode.OutEdges = append(newNode.OutEdges, edge1)

	length2 := pointToPointDistance(Point{X: newNode.X, Y: newNode.Y}, Point{X: nearestEdgeToNode.X, Y: nearestEdgeToNode.Y})
	edge2 := &Edge{
		ID:     len(s.edges) + 1 + temp.EdgeLen(),
		From:   newNode,
		To:     nearestEdgeToNode,
		Length: length2,
		Attr:   nearestEdge.Attr,
	}
	temp.AddEdge(edge2)
	newNode.OutEdges = append(newNode.OutEdges, edge2)
	nearestEdgeToNode.OutEdges = append(nearestEdgeToNode.OutEdges, edge2)

	// 4. 移除原来的边(因为它已被分割)
	for i, edge := range nearestEdgeFromNode.OutEdges {
		if edge.ID == nearestEdge.ID {
			nearestEdgeFromNode.OutEdges = append(nearestEdgeFromNode.OutEdges[:i], nearestEdgeFromNode.OutEdges[i+1:]...)
			break
		}
	}
	for i, edge := range nearestEdgeToNode.OutEdges {
		if edge.ID == nearestEdge.ID {
			nearestEdgeToNode.OutEdges = append(nearestEdgeToNode.OutEdges[:i], nearestEdgeToNode.OutEdges[i+1:]...)
			break
		}
	}
	temp.AddUselessEdge(nearestEdge)

	return newNode, nearestEdge, nil
}

func (s *Network) GetTemp(sessionId int64) *Temp {
	s.tmpLock.Lock()
	defer s.tmpLock.Unlock()
	if _, ok := s.temps[sessionId]; !ok {
		s.temps[sessionId] = NewTemp(sessionId)
	}
	return s.temps[sessionId]
}

func (s *Network) DelTemp(sessionId int64) {
	s.tmpLock.Lock()
	defer s.tmpLock.Unlock()
	if _, ok := s.temps[sessionId]; ok {
		delete(s.temps, sessionId)
	}
}

func reverseSlice(s []astar.Pather) (nodes []*Node) {
	l := len(s)
	for i := l - 1; i >= 0; i-- {
		nodes = append(nodes, s[i].(*Node))
	}
	return
}

func genSessionId() int64 {
	return time.Now().UnixMilli()
}
