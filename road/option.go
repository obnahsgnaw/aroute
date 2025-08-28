package road

type Option func(*Network)

func PointFoundMaxDistance(maxDistance float64) Option {
	return func(s *Network) {
		s.maxDistance = maxDistance
	}
}
