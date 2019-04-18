#define JC_VORONOI_IMPLEMENTATION

#include "Voronoi.h"
#include "jc_voronoi.h"
#include "util.h"
#include <time.h>
#include <iostream>


Voronoi::Voronoi(float32 width, float32 height, std::vector<b2Vec2> points) {
	createDiagram(width, height, points);


}

Voronoi::Voronoi(float32 width, float32 height, unsigned int numPoints, VoronoiDistributionType distribType)
{
	srand(time(NULL));
	std::vector<b2Vec2> randomPoints;
	float32 minX = -width / 2.0f;
	float32 minY = -height / 2.0f;
	float32 maxX = width / 2.0f;
	float32 maxY = height / 2.0f;

	if (distribType == UNIFORM_RANDOM) {
		unsigned int numSectionsX = std::sqrt(numPoints);
		unsigned int numSectionsY = std::sqrt(numPoints);
		for (int y = 0; y < numSectionsY; y++) {
			for (int x = 0; x < numSectionsX; x++) {
				float32 sectionSizeX = (maxX - minX) / numSectionsX;
				float32 sectionSizeY = (maxY - minY) / numSectionsY;

				std::vector<b2Vec2> newPoints;
				newPoints = genRandomPoints(minX + (sectionSizeX * x), minX + (sectionSizeX * (x + 1)), minY + (sectionSizeY * y), minY + sectionSizeY * (y + 1), (numPoints / (numSectionsX * numSectionsX)) <= 0 ? 1 : (numPoints / (numSectionsX * numSectionsX)));
				randomPoints.insert(std::end(randomPoints), std::begin(newPoints), std::end(newPoints));
			}
		}
	}
	else if (distribType == RANDOM) {
		randomPoints = genRandomPoints(minX, maxX, minY, maxY, numPoints);
	}
	else if (distribType == UNIFORM) {
		unsigned int numPerDimension = std::sqrt(numPoints);
		float32 sectionSizeX = (maxX - minX) / numPerDimension;
		float32 sectionSizeY = (maxY - minY) / numPerDimension;

		for (int y = 0; y < numPerDimension; y++) {
			for (int x = 0; x < numPerDimension; x++) {
				randomPoints.push_back(b2Vec2(minX + (sectionSizeX * x) + (sectionSizeX / 2), minY + (sectionSizeY * y) + (sectionSizeY / 2)));
			}
		}
	}

	createDiagram(width, height, randomPoints);
}

void Voronoi::createDiagram(float32 width, float32 height, std::vector<b2Vec2> points)
{
	std::cout << "Number of points: " << points.size() << std::endl;
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

std::vector<b2Vec2> Voronoi::genRandomPoints(float32 minX, float32 maxX, float32 minY, float32 maxY, unsigned int numPoints)
{
	std::vector<b2Vec2> points;
	for (int i = 0; i < numPoints; i++) {
		points.push_back(b2Vec2(RandomFloat(minX, maxX), RandomFloat(minY, maxY)));
	}
	return points;
}
