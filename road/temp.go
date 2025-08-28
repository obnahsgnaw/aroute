package road

type Temp struct {
	sessionId    int64
	nodes        map[int]*Node
	edges        map[int]*Edge
	uselessEdges map[int]struct{}
}

func NewTemp(sessionId int64) *Temp {
	return &Temp{
		sessionId:    sessionId,
		nodes:        make(map[int]*Node),
		edges:        make(map[int]*Edge),
		uselessEdges: make(map[int]struct{}),
	}
}

func (s *Temp) AddNode(node *Node) {
	s.nodes[node.ID] = node
}

func (s *Temp) GetNode(nodeId int) (*Node, bool) {
	v, ok := s.nodes[nodeId]
	return v, ok
}

func (s *Temp) NodeLen() int {
	return len(s.nodes)
}

func (s *Temp) AddEdge(edge *Edge) {
	s.edges[edge.ID] = edge
}

func (s *Temp) GetEdge(edgeId int) (*Edge, bool) {
	v, ok := s.edges[edgeId]
	return v, ok
}

func (s *Temp) AddUselessEdge(edge *Edge) {
	s.uselessEdges[edge.ID] = struct{}{}
}

func (s *Temp) IsUselessEdge(edgeId int) bool {
	_, ok := s.uselessEdges[edgeId]
	return ok
}

func (s *Temp) EdgeLen() int {
	return len(s.edges)
}

func (s *Temp) SessionId() int64 {
	return s.sessionId
}
