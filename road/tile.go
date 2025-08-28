package road

import (
	"github.com/obnahsgnaw/aroute/astart"
	"math"
)

type Point struct {
	X, Y, Z float64
}

// Node road node
type Node struct {
	Point
	ID       int
	OutEdges []*Edge
	m        *Network
}

// Edge road edge
type Edge struct {
	ID       int
	From, To *Node
	Length   float64
	Attr     *Attribute
}

type Attribute struct {
	values map[string]string
}

func (s *Attribute) Get(key string) string {
	if val, ok := s.values[key]; ok {
		return val
	}
	return ""
}

func (s *Edge) Oneway() bool {
	return s.Attr.Get("oneway") == "T"
}

// PathNeighbors 返回相邻的可通行节点
func (n *Node) PathNeighbors(sessionId int64) []astar.Pather {
	var neighbors []astar.Pather
	for _, edge := range n.OutEdges {
		if edge.From.ID == n.ID {
			node := edge.To
			if v, ok := n.m.GetTemp(sessionId).GetNode(node.ID); ok {
				node = v
			}
			neighbors = append(neighbors, node)
		} else if !edge.Oneway() {
			node := edge.From
			if v, ok := n.m.GetTemp(sessionId).GetNode(node.ID); ok {
				node = v
			}
			neighbors = append(neighbors, node)
		}
	}
	return neighbors
}

// PathNeighborCost 计算移动到相邻节点的代价
func (n *Node) PathNeighborCost(_ int64, to astar.Pather) float64 {
	// 查找连接两个节点的边
	toNode := to.(*Node)
	for _, edge := range n.OutEdges {
		if edge.To.ID == toNode.ID {
			return edge.Length
		}
		if !edge.Oneway() && edge.From.ID == toNode.ID {
			return edge.Length
		}
	}
	return math.Inf(1)
}

// PathEstimatedCost 估算到目标节点的代价(启发式函数)
func (n *Node) PathEstimatedCost(_ int64, to astar.Pather) float64 {
	toNode := to.(*Node)
	dx := toNode.X - n.X
	dy := toNode.Y - n.Y
	return math.Sqrt(dx*dx + dy*dy) // 欧氏距离作为启发式估计
	// 使用曼哈顿距离作为启发式函数
	//absX := math.Abs(float64(toNode.X - t.X))
	//absY := math.Abs(float64(toNode.Y - t.Y))
	//return absX + absY
}
