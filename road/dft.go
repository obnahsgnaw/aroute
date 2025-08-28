package road

var _network *Network

func init() {
	_network = newNetwork()
}

func Default() *Network {
	return _network
}
