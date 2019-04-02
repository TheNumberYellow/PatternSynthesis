#pragma once
#include <Box2D\Box2D.h>
#include <vector>

struct Edge {
	b2Vec2 a;
	b2Vec2 b;
	Edge(b2Vec2 a, b2Vec2 b) : a(a), b(b) {}
	Edge(float32 aX, float32 aY, float32 bX, float32 bY) : a(aX, aY), b(bX, bY) {}
};



struct Voronoi {
	std::vector<Edge> edges; // Final vector of edges, to be used for simulation

	Voronoi(float32 width, float32 height, std::vector<b2Vec2> points);	// Create voronoi diagram with given points
	Voronoi(float32 width, float32 height, unsigned int numPoints); // Create voronoi diagram with random points

private:
	void createDiagram(float32 width, float32 height, std::vector<b2Vec2> points);
};