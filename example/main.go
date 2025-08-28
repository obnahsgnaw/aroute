package main

import (
	"fmt"
	"github.com/obnahsgnaw/aroute/road"
	"log"
)

func main() {
	network := road.Default()
	err := network.Load("../out/demo.shp")
	if err != nil {
		log.Fatalf("转换路网失败: %v", err)
	}
	demo(network)
}

func demo(network *road.Network) {
	points := []road.Point{
		{X: 117.25844600, Y: 39.13584500},
		{X: 117.25843476, Y: 39.13563395},
	}
	path, distance, found := network.Find(points...)
	if !found {
		log.Println("未找到路径")
		return
	}

	fmt.Printf("路径总长度: %.2f 米\n", distance)
	fmt.Println("路径节点序列:")
	for _, node := range path {
		//fmt.Printf("%d. 节点ID:%d (%.6f, %.6f)\n", i+1, node.ID, node.X, node.Y)
		fmt.Printf("%.6f, %.6f\n", node.X, node.Y)
	}
}
