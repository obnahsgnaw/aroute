package road

import (
	"math"
)

var rad = math.Pi / 180

// pointToPointDistance 经纬度距离 米
func pointToPointDistance(p1, p2 Point) float64 {
	lat1 := toRadian(p1.Y)
	lat2 := toRadian(p2.Y)
	theta := toRadian(p2.X) - toRadian(p1.X)
	dist := math.Acos(math.Sin(lat1)*math.Sin(lat2) + math.Cos(lat1)*math.Cos(lat2)*math.Cos(theta))
	dist *= 6371000 // 6378.137
	return math.Round(dist)
}

// 角度转幅度
func toRadian(angle float64) float64 {
	return angle * rad
}

// 幅度转角度
func toAngle(radian float64) float64 {
	return radian / rad
}
