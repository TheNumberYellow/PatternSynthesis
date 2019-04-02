#define JC_VORONOI_IMPLEMENTATION

#include "Voronoi.h"
#include "jc_voronoi.h"
#include <time.h>

float32 RandomFloat(float32 a, float32 b) {
	float32 random = ((float32)rand()) / (float32)RAND_MAX;
	float32 diff = b - a;
	float32 r = random * diff;
	return a + r;
}

Voronoi::Voronoi(float32 width, float32 height, std::vector<b2Vec2> points) {
	createDiagram(width, height, points);


}

Voronoi::Voronoi(float32 width, float32 height, unsigned int numPoints)
{
	srand(time(NULL));
	std::vector<b2Vec2> randomPoints;
	float32 minX = -width / 2.0f;
	float32 minY = -height / 2.0f;
	float32 maxX = width / 2.0f;
	float32 maxY = height / 2.0f;


	for (int i = 0; i < numPoints; i++) {
		randomPoints.push_back(b2Vec2(RandomFloat(minX, maxX), RandomFloat(minY, maxY)));
	}

	createDiagram(width, height, randomPoints);
}

void Voronoi::createDiagram(float32 width, float32 height, std::vector<b2Vec2> points)
{
	jcv_diagram diagram;
	jcv_point* jcvPoints = new jcv_point[points.size()];

	for (int i = 0; i < points.size(); i++) {
		jcvPoints[i] = { points[i].x, points[i].y };
	}

	jcv_rect jcvRect;
	jcvRect.min = { -width / 2, -height / 2 };
	jcvRect.max = { width / 2, height / 2 };

	jcv_diagram_generate(points.size(), jcvPoints, &jcvRect, &diagram);


	const jcv_edge* edgeP = jcv_diagram_get_edges(&diagram);
	while (edgeP) {
		edges.push_back(Edge(b2Vec2((float32)edgeP->pos[0].x, (float32)edgeP->pos[0].y),
			b2Vec2((float32)edgeP->pos[1].x, (float32)edgeP->pos[1].y)));
		edgeP = jcv_diagram_get_next_edge(edgeP);
	}
}
