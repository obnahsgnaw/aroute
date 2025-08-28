package road

import (
	"math"
)

// PointToLineDistanceAndFootPoint 计算点到线段的垂直距离及垂足点坐标
func PointToLineDistanceAndFootPoint(p, a, b Point) (distance float64, footPoint Point, onSegment bool) {
	// 将经纬度转换为三维笛卡尔坐标（单位向量）
	pCart := geographicToCartesian(p.Y, p.X)
	aCart := geographicToCartesian(a.Y, a.X)
	bCart := geographicToCartesian(b.Y, b.X)

	// 计算向量AB和AP
	ab := [3]float64{
		bCart[0] - aCart[0],
		bCart[1] - aCart[1],
		bCart[2] - aCart[2],
	}
	ap := [3]float64{
		pCart[0] - aCart[0],
		pCart[1] - aCart[1],
		pCart[2] - aCart[2],
	}

	// 计算AB的长度平方
	abLengthSq := ab[0]*ab[0] + ab[1]*ab[1] + ab[2]*ab[2]

	// 计算AP在AB上的投影比例
	t := (ap[0]*ab[0] + ap[1]*ab[1] + ap[2]*ab[2]) / abLengthSq

	// 计算垂足点在笛卡尔坐标中的位置
	var fCart [3]float64
	if t <= 0 {
		// 垂足在线段起点外侧
		fCart = aCart
		onSegment = false
	} else if t >= 1 {
		// 垂足在线段终点外侧
		fCart = bCart
		onSegment = false
	} else {
		// 垂足在线段上
		fCart = [3]float64{
			aCart[0] + t*ab[0],
			aCart[1] + t*ab[1],
			aCart[2] + t*ab[2],
		}
		onSegment = true
	}

	// 将垂足点转换为经纬度
	footPoint = cartesianToGeographic(fCart)

	// 计算点到垂足的距离（大圆距离）
	distance = haversineDistance(p, footPoint)

	return distance, footPoint, onSegment
}

// geographicToCartesian 将经纬度转换为三维笛卡尔坐标
func geographicToCartesian(lat, lon float64) [3]float64 {
	latRad := lat * math.Pi / 180
	lonRad := lon * math.Pi / 180

	// 假设地球是单位球体
	x := math.Cos(latRad) * math.Cos(lonRad)
	y := math.Cos(latRad) * math.Sin(lonRad)
	z := math.Sin(latRad)

	return [3]float64{x, y, z}
}

// cartesianToGeographic 将三维笛卡尔坐标转换为经纬度
func cartesianToGeographic(c [3]float64) Point {
	// 归一化
	norm := math.Sqrt(c[0]*c[0] + c[1]*c[1] + c[2]*c[2])
	x := c[0] / norm
	y := c[1] / norm
	z := c[2] / norm

	lat := math.Asin(z) * 180 / math.Pi
	lon := math.Atan2(y, x) * 180 / math.Pi

	return Point{Y: lat, X: lon}
}

// haversineDistance 使用Haversine公式计算两点间的大圆距离
func haversineDistance(a, b Point) float64 {
	const R = 6371000 // 地球平均半径，单位米

	lat1 := a.Y * math.Pi / 180
	lon1 := a.X * math.Pi / 180
	lat2 := b.Y * math.Pi / 180
	lon2 := b.X * math.Pi / 180

	dLat := lat2 - lat1
	dLon := lon2 - lon1

	d := math.Sin(dLat/2)*math.Sin(dLat/2) + math.Cos(lat1)*math.Cos(lat2)*math.Sin(dLon/2)*math.Sin(dLon/2)
	c := 2 * math.Atan2(math.Sqrt(d), math.Sqrt(1-d))

	return R * c
}
