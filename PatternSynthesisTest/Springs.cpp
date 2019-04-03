#include "Springs.h"
#include "RASF.h"

static const float INVSCALE = 1.0f / 30.0f;

float32 clampAngle(float32 angle)
{
	//return fmod((angle + b2_pi), (2.0f * b2_pi)) - b2_pi;
	if (angle > b2_pi) {
		return angle - (2.0f * b2_pi);
	}
	else if (angle < -b2_pi){
		return angle + (2.0f * b2_pi);
	}
	return angle;
}

Spring::Spring(b2Body* body, b2Body* nextBody, b2World* world) :
	body(body), nextBody(nextBody)
{
	restLength = b2Distance(body->GetPosition(), nextBody->GetPosition());
	
}

void Spring::setPrevBody(b2Body* prevBody)
{
	this->prevBody = prevBody;
	b2Vec2 diffVecPrev = body->GetPosition() - prevBody->GetPosition();
	b2Vec2 diffVecNext = body->GetPosition() - nextBody->GetPosition();

	float32 anglePrev = atan2(diffVecPrev.y, diffVecPrev.x);
	float32 angleNext = atan2(diffVecNext.y, diffVecNext.x);

	baseLineAngle = b2_pi - abs(anglePrev - angleNext) - b2_pi;
	//std::cout << "Base line angle: " << baseLineAngle * RADTODEG << std::endl;
}

void Spring::setRestAngle(float32 angle)
{
	restAngle = angle;
}

void Spring::updateForces()
{
	float32 distance = b2Distance(body->GetPosition(), nextBody->GetPosition());
	float32 force = linearK * (distance - restLength);
	
	b2Vec2 diffVec = nextBody->GetPosition() - body->GetPosition();
	diffVec.Normalize();

	body->ApplyForceToCenter((force / 2.0f) * diffVec, true);
	nextBody->ApplyForceToCenter((force / 2.0f) * -diffVec, true);
	
	if (!prevBody) return;
	
	b2Vec2 diffVecPrev = body->GetPosition() - prevBody->GetPosition();
	b2Vec2 diffVecNext = body->GetPosition() - nextBody->GetPosition();

	float64 anglePrev = atan2(diffVecPrev.y, diffVecPrev.x);
	float64 angleNext = atan2(diffVecNext.y, diffVecNext.x);
	
	float64 dot = diffVecPrev.x * diffVecNext.x + diffVecPrev.y * diffVecNext.y;
	float64 det = diffVecPrev.x * diffVecNext.y - diffVecPrev.y * diffVecNext.x;

	float64 angle = atan2(det, dot);
	if (angle < 0) angle += 2 * b2_pi;

	float64 deltaAngle = angle - restAngle + baseLineAngle;

	float64 rotForce = rotK * deltaAngle;

	//std::cout << "Inner Angle: " << angle * RADTODEG << " Angle Prev: " << anglePrev * RADTODEG << " Angle Next: " << angleNext * RADTODEG << std::endl;


	b2Vec2 prevBodyForceVec = b2Vec2(diffVecPrev.y, -diffVecPrev.x);
	b2Vec2 nextBodyForceVec = b2Vec2(-diffVecNext.y, diffVecNext.x);
	prevBodyForceVec.Normalize();
	nextBodyForceVec.Normalize();
	
	prevBody->ApplyForceToCenter((rotForce / 2.0f) * prevBodyForceVec, true);
	nextBody->ApplyForceToCenter((rotForce / 2.0f) * nextBodyForceVec, true);
	
	body->ApplyForceToCenter((rotForce / 2.0f) * -prevBodyForceVec, true);
	body->ApplyForceToCenter((rotForce / 2.0f) * -nextBodyForceVec, true);

	

}

SpringLine::SpringLine(b2Vec2 startPoint, b2Vec2 endPoint, std::vector<Spring*> springs, RASF restAngleFunc) :
	startPoint(startPoint), endPoint(endPoint),
	springs(springs),
	restAngleFunc(restAngleFunc)
{
	b2Vec2 diffVector = endPoint - startPoint;
	initialAngle = atan2(diffVector.y, diffVector.x);
}

// Goes through spring lines and attaches them together
void SpringWorld::connectSpringLines() {
	float32	minDistance = 0.00001; // TODO: maybe test this
	for (auto s1 = springLines.begin(); s1 != springLines.end(); s1++) {
		for (auto s2 = s1 + 1; s2 != springLines.end(); s2++) {

			if (s1 == s2) continue;
			
			// TODO: This is ugly and error prone

			b2RevoluteJointDef jointDef;
			jointDef.collideConnected = false;

			if (b2Distance((*s1)->startPoint, (*s2)->startPoint) <= minDistance) {
				jointDef.bodyA = (*s1)->startBody;
				jointDef.bodyB = (*s2)->startBody;
				world->CreateJoint(&jointDef);
				
				float32 s1Angle = clampAngle((*s2)->initialAngle - (*s1)->initialAngle);
				float32 s2Angle = clampAngle((*s1)->initialAngle - (*s2)->initialAngle);
				(*s1)->startAngles.push_back(s1Angle);
				(*s2)->startAngles.push_back(s2Angle);
			}
			if (b2Distance((*s1)->startPoint, (*s2)->endPoint) <= minDistance) {
				jointDef.bodyA = (*s1)->startBody;
				jointDef.bodyB = (*s2)->endBody;
				world->CreateJoint(&jointDef);
				
				float32 s1Angle = clampAngle((*s2)->initialAngle - (*s1)->initialAngle);
				float32 s2Angle = clampAngle((*s1)->initialAngle - (*s2)->initialAngle);
				(*s1)->startAngles.push_back(s1Angle);
				(*s2)->endAngles.push_back(s2Angle);
			}
			if (b2Distance((*s1)->endPoint, (*s2)->startPoint) <= minDistance) {
				jointDef.bodyA = (*s1)->endBody;
				jointDef.bodyB = (*s2)->startBody;
				world->CreateJoint(&jointDef);
				
				float32 s1Angle = clampAngle((*s2)->initialAngle - (*s1)->initialAngle);
				float32 s2Angle = clampAngle((*s1)->initialAngle - (*s2)->initialAngle);
				(*s1)->endAngles.push_back(s1Angle);
				(*s2)->startAngles.push_back(s2Angle);
			}
			if (b2Distance((*s1)->endPoint, (*s2)->endPoint) <= minDistance) {
				jointDef.bodyA = (*s1)->endBody;
				jointDef.bodyB = (*s2)->endBody;
				world->CreateJoint(&jointDef);
				
				float32 s1Angle = clampAngle((*s2)->initialAngle - (*s1)->initialAngle);
				float32 s2Angle = clampAngle((*s1)->initialAngle - (*s2)->initialAngle);
				(*s1)->endAngles.push_back(s1Angle);
				(*s2)->endAngles.push_back(s2Angle);
			}

		}
	}
}

 void SpringWorld::initRestAngles() {
	for (SpringLine* s : springLines) {
			
		for (int i = 0; i < s->springs.size(); i++) {

			float32 t = (float32)i / (float32)s->springs.size(); // How far along the spring line, from 0 to 1 

			s->springs[i]->restAngle = s->restAngleFunc(s->startAngles, s->endAngles, t, s->springs.size());
		}
	}
}

void SpringWorld::initSpringWorld() {
	connectSpringLines();
	initRestAngles();
}

b2World* SpringWorld::getWorld()
{
	return world;
}

// Takes physics timestep
void SpringWorld::update(float32 timeStep) {
	for (SpringLine* sl : springLines) {
		for (Spring* s : sl->springs) {
			s->updateForces();
		}
	}
	world->Step(timeStep, 80, 30); // Hard-coded position/velocity iterations
}

void SpringWorld::createSpringLine(b2Vec2 from, b2Vec2 to, unsigned int numSegments, RASF restAngleFunc, bool dynamic) {
	// TODO: this function is heavily coupled to box2d

	std::vector<Spring*> springs;

	b2Vec2 diffVector = to - from;
	float32 lineLength = diffVector.Length();
	float32 segmentLength = lineLength / (float32)numSegments;

	float32 lineAngle = atan2(diffVector.y, diffVector.x); // In radians

	b2BodyDef sectionBodyDef;

	if (dynamic) sectionBodyDef.type = b2_dynamicBody;
	
	sectionBodyDef.angularDamping = 1.0f; // TODO: May change this
	sectionBodyDef.linearDamping = 1.0f; // TODO: and this

	b2Body* firstBody = nullptr;
	b2Body* lastBody = nullptr;

	b2Body* prevPrevSpringBody = nullptr;
	b2Body* prevSpringBody = nullptr;

	for (unsigned int i = 0; i < numSegments + 1; i++) {

		b2Vec2 bodyPos = from + (((float32)i / (float32)numSegments) * diffVector);

		sectionBodyDef.position.Set(bodyPos.x, bodyPos.y);
		sectionBodyDef.angle = lineAngle;

		b2Body* springBody = world->CreateBody(&sectionBodyDef);

		b2PolygonShape sectionShape;
		sectionShape.SetAsBox(0.15f, 0.15f, b2Vec2(0.0f, 0.0f), 0.0f); // SetAsBox uses half-widths, so / 2.0f

		b2FixtureDef sectionFixtureDef;
		sectionFixtureDef.shape = &sectionShape;
		sectionFixtureDef.density = 1.0f;
		sectionFixtureDef.filter.categoryBits = 0x0002; // Sections can never interact with other sections
		sectionFixtureDef.filter.maskBits = 0x0004;

		springBody->CreateFixture(&sectionFixtureDef);

		if (i == 0) firstBody = springBody;

		if (prevSpringBody) {
			Spring* s = new Spring(prevSpringBody, springBody, world);  // For now, the rest angle doesn't need to be set (we need to know what the spring line is attached to first)
			if (prevPrevSpringBody) s->setPrevBody(prevPrevSpringBody);
			s->setRestAngle(10.0f * DEGTORAD);

			springs.push_back(s);
			prevPrevSpringBody = prevSpringBody;
		}
		prevSpringBody = springBody;
	}
	lastBody = prevSpringBody;
	
	SpringLine* line = new SpringLine(from, to, springs, restAngleFunc);
	line->startBody = firstBody;
	line->endBody = lastBody;

	springLines.push_back(line);
}

void SpringWorld::createSpringLine(Edge edge, unsigned int numSegments, RASF restAngleFunc, bool dynamic) {
	createSpringLine(edge.a, edge.b, numSegments, restAngleFunc, dynamic);
}

// numSegments: number of segments per box side
// sideAngle: angle of springs of box sides (corners are 90 degrees)
void SpringWorld::createSpringBox(unsigned int numSegments, RASF_TYPE type, float32 sideAngle) {

	auto func = getRASF(type, sideAngle);

	createSpringLine(b2Vec2(-5.0f, -5.0f), b2Vec2(5.0f, -5.0f), numSegments, func);
	createSpringLine(b2Vec2(5.0f, -5.0f), b2Vec2(5.0f, 5.0f), numSegments, func);
	createSpringLine(b2Vec2(5.0f, 5.0f), b2Vec2(-5.0f, 5.0f), numSegments, func);
	createSpringLine(b2Vec2(-5.0f, 5.0f), b2Vec2(-5.0f, -5.0f), numSegments, func);

	initSpringWorld();
}


void SpringWorld::createSystem(Border border, std::vector<Edge> edges, RASF_TYPE type,  float32 angleSeverity) {
	auto func = getRASF(type, angleSeverity);

	border.minX *= INVSCALE;
	border.minY *= INVSCALE;
	border.maxX *= INVSCALE;
	border.maxY *= INVSCALE;

	
	for (Edge e : edges) {
		

		float32 distance = b2Distance(e.a, e.b);
		
		unsigned int numSegments = distance * 2.0f; // TODO: Too many segments may lead to glitchyness? Not sure exactly, will look into box2d 
		
		if (numSegments == 0) numSegments = 1;
		
		
		// If either edge end is within border, line is dynamic
		if (border.isWithinBorder(e.a) || border.isWithinBorder(e.b)) {
			createSpringLine(e, numSegments, func, true);
		}
		// If both ends are outside border, it is a non-dynamic border line
		else {
			createSpringLine(e, numSegments, func, false);
		}
	}

	initSpringWorld();

}

void SpringWorld::createSquiggle(unsigned int numSegments, RASF_TYPE type, float32 angleSeverity)
{
	auto func = getRASF(type, angleSeverity);

	createSpringLine(b2Vec2(-10.0f, -5.0f), b2Vec2(0.0f, -5.0f), numSegments, func);
	createSpringLine(b2Vec2(0.0f, -5.0f), b2Vec2(0.0f, 5.0f), numSegments, func);
	createSpringLine(b2Vec2(0.0f, 5.0f), b2Vec2(10.0f, 5.0f), numSegments, func);

	initSpringWorld();
}


void SpringWorld::drawTree(float32 x1, float32 y1, float32 angle, int depth, RASF func)
{
	if (depth) {
		float32 x2 = (x1 + std::cos(angle) * depth * 1.15f);
		float32 y2 = (y1 + std::sin(angle) * depth * 1.15f);
		
		float32 distance = b2Distance(b2Vec2(x1, y1), b2Vec2(x2, y2));
		unsigned int numSegments = distance * 2.0f; // TODO: Too many segments may lead to glitchyness? Not sure exactly, will look into box2d 

		if (numSegments == 0) numSegments = 1;

		createSpringLine(b2Vec2(x1, y1), b2Vec2(x2, y2), numSegments, func);
		drawTree(x2, y2, angle - (20.0f * DEGTORAD), depth - 1, func);
		drawTree(x2, y2, angle + (20.0f * DEGTORAD), depth - 1, func);
	}
}

void SpringWorld::createFractalTree(unsigned int fractalDepth, RASF_TYPE type, float32 angleSeverity)
{
	auto func = getRASF(type, angleSeverity);

	drawTree(0.0f, 10.0f, -90.0f * DEGTORAD, fractalDepth, func);
	
	initSpringWorld();
}

std::vector<Edge> SpringWorld::getSpringEdges()
{
	std::vector<Edge> edges;
	for (SpringLine* sl : springLines) {
		for (Spring* s : sl->springs) {
			edges.push_back(Edge(s->body->GetPosition(), s->nextBody->GetPosition()));
		}
	}
	return edges;
}
