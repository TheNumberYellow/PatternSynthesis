#pragma once
#include <Box2D\Box2D.h>

#include <vector>
#include <functional>
#include <iostream>

#include "Voronoi.h"
#include "RASF.h"


float32 lerp(float32 a, float32 b, float32 t);

struct Border {
	float32 minX, maxX;
	float32 minY, maxY;
	Border(float32 minX, float32 minY, float32 maxX, float32 maxY) : minX(minX), maxX(maxX), minY(minY), maxY(maxY) {}
	bool isWithinBorder(b2Vec2 point) {
		return point.x > minX && point.x < maxX && point.y > minY && point.y < maxY;
	}
};

struct Spring {
	// The previous body in the line is required to apply the correct torque for the angular component of the spring
	// It is not required for the linear (distance) component
	b2Body* prevBody = nullptr;

	b2Body* body = nullptr;
	b2Body* nextBody = nullptr;

	float32 restAngle = 0.0f; // Target angle between prevBody and bodyB (from bodyA) (where flat = 0). In radians.
	float32 baseLineAngle = 0.0f;
	float32 restLength = 0.0f; // Target length between 

	float32 linearK = 20.0f;
	float32 rotK = 20.0f;

	Spring(b2Body* body, b2Body* nextBody, b2World* world);
	
	void setPrevBody(b2Body* prevBody);
	void setRestAngle(float32 angle);

	// Applies forces to bodies, pulling them together or pushing them apart based on distance
	// (Does not deal with rotational spring forces, those are only applicable when considering the entire spring line)
	void updateForces();
};

struct SpringLine {
	std::vector<Spring*> springs;

	b2Vec2 startPoint; // point the spring line starts at (Part of spring line)
	b2Vec2 endPoint; // point the spring line ends at (Part of spring line)

	b2Body* startBody = nullptr;
	b2Body* endBody = nullptr;

	// Vector for holding the angles of all the springlines the start point of the line 
	// is connected to (clockwise angle relative to this line's angle) (in radians)
	std::vector<float32> startAngles;  
	// Vector for holding the angles of all the springlines the end point of the line 
	// is connected to (clockwise angle relative to this line's angle) (in radians)
	std::vector<float32> endAngles; 

	std::function<float32(std::vector<float32>, std::vector<float32>, float32, unsigned int)> restAngleFunc;

	float32 initialAngle = 0.0f;

	SpringLine(b2Vec2 startPoint, b2Vec2 endPoint, std::vector<Spring*> springs, RASF restAngleFunc);
};

// To store all spring lines in vector for updating, creating new spring lines etc.
struct SpringWorld {

public:
	SpringWorld(b2World* world) : world(world) {}
	
	b2World* getWorld();

	// Apply forces on all springs, and takes physics timestep
	void update(float32 timeStep);

	void createSpringLine(b2Vec2 from, b2Vec2 to, unsigned int numSegments, RASF restAngleFunc, bool dynamic = true);

	void createSpringLine(Edge edge, unsigned int numSegments, RASF restAngleFunc, bool dynamic = true);

	// numSegments: number of segments per box side
	// sideAngle: angle of springs of box sides (corners are 90 degrees)
	void createSpringBox(unsigned int numSegments,  RASF_TYPE type, float32 sideAngle);

	// Border: Edges outside of border will be static (only if both points in edge are outside border)
	// Edges: Edges of system
	// Angle Severity: rest angle function multiplier
	void createSystem(Border border, std::vector<Edge> edges, RASF_TYPE type, float32 angleSeverity = 1.0f);
	
	// numSegments: number of segments per squiggle side
	void createSquiggle(unsigned int numSegments, RASF_TYPE type, float32 angleSeverity = 1.0f);

	void createFractalTree(unsigned int fractalDepth, RASF_TYPE type, float32 angleSeverity = 1.0f);
	
	void createRandomizedFractalTree(unsigned int fractalDepth, RASF_TYPE type, float32 rasfValue = 1.0f);

	std::vector<Edge> getSpringEdges();
	
	void initSpringWorld();
private:

	std::vector<SpringLine*> springLines;
	std::vector<Spring*> connectiveSprings; // Springs connecting different spring lines

	b2World* world;

	// Goes through spring lines and attaches them together
	void connectSpringLines();
	// Goes through spring lines and sets inner rest angles based on that spring line's RASF
	void initRestAngles();
	
	// Connects spring lines together then initializes rest angles

	void drawTree(float32 x1, float32 y1, float32 angle, int depth, RASF func);
	void drawRandomizedTree(float32 x1, float32 y1, float32 angle, int depth, RASF func);
};


