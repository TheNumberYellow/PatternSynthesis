#define JC_VORONOI_IMPLEMENTATION

#include <SFML\Graphics.hpp>
#include <Box2D\Box2D.h>

#include <iostream>
#include <vector>
#include <time.h>


#include "Voronoi.h"
#include "Springs.h"

// IDEAS: 
//createSpringLine takes two angles to lerp between, angles could be determined by angle of intersecting bodies
//createSpringLine takes optional body to attach to at from vector
//Maybe use bezier curves for angles?
//+ Look up "composite bezier curves"

// Box2d works in MKS (meters-kilogram-seconds) units, this scale is used to convert to pixel coordinates (ONLY WHEN DRAWING)
// 1 Meter = 30 pixels
static const float SCALE = 30.f;
static const float INVSCALE = 1.0f / 30.0f;

void drawPolygonShape(b2Body* body, b2PolygonShape* shape, sf::RenderWindow* window) {
	sf::ConvexShape cShape;
	cShape.setFillColor(sf::Color::Black);

	cShape.setPointCount(shape->m_count);
	for (int i = 0; i < shape->m_count; i++) {
		b2Vec2 point = shape->m_vertices[i];
		point = body->GetWorldPoint(point);

		sf::Vector2f worldPoint = sf::Vector2f((point.x * SCALE) + (window->getSize().x / 2), (point.y * SCALE) + (window->getSize().y / 2));
		
		cShape.setPoint(i, worldPoint);
	}
	 
	window->draw(cShape);
}

void drawEdges(std::vector<Edge> edges, sf::RenderWindow* window) {
	
	for (Edge e : edges) {
		unsigned int windowWidth = window->getSize().x;
		unsigned int windowHeight = window->getSize().y;
		e.a.x = (e.a.x * SCALE) + windowWidth / 2;
		e.a.y = (e.a.y * SCALE) + windowHeight / 2;
		e.b.x = (e.b.x * SCALE) + windowWidth / 2;
		e.b.y = (e.b.y * SCALE) + windowHeight / 2;
		

		sf::Vertex line[] = {
			sf::Vertex(sf::Vector2f(e.a.x, e.a.y), sf::Color::Black),
			sf::Vertex(sf::Vector2f(e.b.x, e.b.y), sf::Color::Black)
		};
		window->draw(line, 2, sf::Lines);
	}
}

void drawBodies(SpringWorld sWorld, sf::RenderWindow* window) {
	for (b2Body* bodyIt = sWorld.getWorld()->GetBodyList(); bodyIt != nullptr; bodyIt = bodyIt->GetNext()) {
		b2Fixture* f = bodyIt->GetFixtureList();

		while (f != NULL) {
			b2Shape::Type shapeType = f->GetType();

			if (shapeType == b2Shape::e_polygon) {
				b2PolygonShape* shape = (b2PolygonShape*)f->GetShape();

				drawPolygonShape(bodyIt, shape, window);
			}
			f = f->GetNext();
		}

	}
}

void saveScreenshot(sf::RenderWindow& window, std::string filename) {
	sf::Vector2u windowSize = window.getSize();
	sf::Texture texture;
	texture.create(windowSize.x, windowSize.y);
	texture.update(window);

	sf::Image screenShot = texture.copyToImage();
	screenShot.saveToFile(filename);

}

RASF_TYPE decideRASFType() {
	std::cout << "Rest angle set function (RASF) type?" << std::endl;
	std::cout << "Press [1] for constant RASF." << std::endl;
	std::cout << "Press [2] for basic LERP RASF. (Largest angles)" << std::endl;
	std::cout << "Press [3] for average angle RASF." << std::endl;
	std::cout << "Press [4] for randomized RASF." << std::endl;
	std::cout << "Press [5] for SIN wave RASF." << std::endl;
	std::cout << "Press [6] for pseudorandomized RASF." << std::endl;
	std::cout << "Press [7] for sequential SIN wave RASF." << std::endl;
	std::cout << "Press [8] for sequential SIN wave + LERP RASF." << std::endl;

	int input = 0;
	std::cin >> input;
	switch (input) {
	case 1:
		return RASF_CONSTANT;
	case 2:
		return RASF_BASIC_LERP;
	case 3:
		return RASF_AVERAGE;
	case 4:
		return RASF_RANDOMIZED;
	case 5:
		return RASF_SINWAVE;
	case 6:
		return RASF_PSEUDORANDOM;
	case 7:
		return RASF_SEQUENTIAL_SIN;
	case 8:
		return RASF_SEQUENTIAL_SIN_PLUS_LERP;
	default:
		return RASF_CONSTANT;
	}
}

void decidePatternToCreate(SpringWorld* sWorld, unsigned int screenWidth, unsigned int screenHeight) {

	while (true) {
		std::cout << "Press [1] for single line diagram." << std::endl;
		std::cout << "Press [2] for box diagram." << std::endl;
		std::cout << "Press [3] for test squiggle diagram." << std::endl;
		std::cout << "Press [4] for Voronoi diagram." << std::endl;
		std::cout << "Press [5] for fractal tree diagram." << std::endl;
		std::cout << "Press [6] for randomized fractal tree diagram." << std::endl;
		std::cout << "Press [7] for uniformly random Voronoi diagram." << std::endl;
		std::cout << "Press [8] for uniform grid." << std::endl;
		
		std::cout << std::endl;

		std::cout << "Press [a] for LERP Voronoi diagram." << std::endl;
		std::cout << "Press [b] for tree diagram." << std::endl;
		std::cout << "Press [c] for... really basic \"scales\" diagram." << std::endl;
		std::cout << "Press [d] for dried mud cracks-esque diagram." << std::endl;
		std::cout << "Press [e] for consecutive sine waves diagram." << std::endl;

		char input;
		std::cin >> input;
		
		switch (input) {
		case '1':
		{
			unsigned int springs = 0;
			RASF_TYPE type;
			float32 restAngle = 0.0f;

			std::cout << "Springs in line?" << std::endl;
			std::cin >> springs;

			std::cout << "Rest angle? (in degrees)" << std::endl;
			std::cin >> restAngle;

			std::cout << "Creating line." << std::endl;
			sWorld->createSpringLine(b2Vec2(-10.0f, 0.0f), b2Vec2(10.0f, 0.0f), springs, getSequentialSinRASF(restAngle));
			sWorld->initSpringWorld();
		}
			return;
		case '2':
		{
			unsigned int sideSprings = 0;
			RASF_TYPE type;
			float32 rasfValue = 0.0f;

			std::cout << "Springs per side?" << std::endl;
			std::cin >> sideSprings;
			
			type = decideRASFType();

			std::cout << "RASF value? (either a multiplier value, or an angle in degrees)" << std::endl;
			std::cin >> rasfValue;

			std::cout << "Creating box." << std::endl;
			sWorld->createSpringBox(sideSprings, type, rasfValue);
		}
			return;
		case '3':
		{
			unsigned int sideSprings = 0;
			RASF_TYPE type;
			float32 rasfValue = 0.0f;

			std::cout << "Springs per side?" << std::endl;
			std::cin >> sideSprings;
			
			type = decideRASFType();
			
			std::cout << "RASF value? (either a multiplier value, or an angle in degrees)" << std::endl;
			std::cin >> rasfValue;

			std::cout << "Creating test squiggle." << std::endl;
			sWorld->createSquiggle(sideSprings, type, rasfValue);

			return;
		}
		case '4':
		{
			unsigned int numPoints = 0;
			RASF_TYPE type;
			float32 rasfValue = 0.0f;

			std::cout << "Number of points in Voronoi diagram?" << std::endl;
			std::cin >> numPoints;
			
			type = decideRASFType();
			
			std::cout << "RASF value? (either a multiplier value, or an angle in degrees)" << std::endl;
			std::cin >> rasfValue;

			std::cout << "Creating Voronoi diagram." << std::endl;
			
			Voronoi v(screenWidth * INVSCALE, screenHeight * INVSCALE, numPoints, RANDOM);
			Border b((-(int)screenWidth / 2.0f), (-(int)screenHeight / 2.0f), ((int)screenWidth / 2.0f), ((int)screenHeight / 2.0f));
			sWorld->createSystem(b, v.edges, type, rasfValue);
		}
			return;
		case '5':
		{
			unsigned int fractalDepth = 0;
			RASF_TYPE type;
			float32 rasfValue = 0.0f;

			std::cout << "Fractal depth?" << std::endl;
			std::cin >> fractalDepth;

			type = decideRASFType();

			std::cout << "RASF value? (either a multiplier value, or an angle in degrees)" << std::endl;
			std::cin >> rasfValue;

			sWorld->createFractalTree(fractalDepth, type, rasfValue);
		}
			return;	
		case '6':
		{
			unsigned int fractalDepth = 0;
			RASF_TYPE type;
			float32 rasfValue = 0.0f;

			std::cout << "Fractal depth?" << std::endl;
			std::cin >> fractalDepth;

			type = decideRASFType();

			std::cout << "RASF value? (either a multiplier value, or an angle in degrees)" << std::endl;
			std::cin >> rasfValue;

			sWorld->createRandomizedFractalTree(fractalDepth, type, rasfValue);
		}
			return;
		case '7':
		{
			unsigned int numPoints = 0;
			RASF_TYPE type;
			float32 rasfValue = 0.0f;

			std::cout << "Number of points in Voronoi diagram?" << std::endl;
			std::cin >> numPoints;

			type = decideRASFType();

			std::cout << "RASF value? (either a multiplier value, or an angle in degrees)" << std::endl;
			std::cin >> rasfValue;

			std::cout << "Creating Voronoi diagram." << std::endl;

			Voronoi v(screenWidth* INVSCALE, screenHeight* INVSCALE, numPoints, UNIFORM_RANDOM);
			Border b((-(int)screenWidth / 2.0f), (-(int)screenHeight / 2.0f), ((int)screenWidth / 2.0f), ((int)screenHeight / 2.0f));
			sWorld->createSystem(b, v.edges, type, rasfValue);
		}
			return;
		case '8':
		{
			unsigned int numPoints = 0;
			RASF_TYPE type;
			float32 rasfValue = 0.0f;

			std::cout << "Size of one side of grid?" << std::endl;
			std::cin >> numPoints;

			type = decideRASFType();

			std::cout << "RASF value? (either a multiplier value, or an angle in degrees)" << std::endl;
			std::cin >> rasfValue;

			std::cout << "Creating Voronoi diagram." << std::endl;

			Voronoi v(screenWidth* INVSCALE, screenHeight * INVSCALE, numPoints * numPoints, UNIFORM);
			Border b((-(int)screenWidth / 2.0f), (-(int)screenHeight / 2.0f), ((int)screenWidth / 2.0f), ((int)screenHeight / 2.0f));
			sWorld->createSystem(b, v.edges, type, rasfValue);
		}
			return;
		case 'a':
		{
			Voronoi v(screenWidth* INVSCALE, screenHeight * INVSCALE, 100, RANDOM);
			Border b((-(int)screenWidth / 2.0f), (-(int)screenHeight / 2.0f), ((int)screenWidth / 2.0f), ((int)screenHeight / 2.0f));
			sWorld->createSystem(b, v.edges, RASF_BASIC_LERP, 1.0f);
		}
			return;
		case 'b':
		{
			sWorld->createRandomizedFractalTree(7, RASF_RANDOMIZED, 2.0f);
		}
			return;
		case 'c':
		{
			Voronoi v(screenWidth* INVSCALE, screenHeight * INVSCALE, 10 * 10, UNIFORM);
			Border b((-(int)screenWidth / 2.0f), (-(int)screenHeight / 2.0f), ((int)screenWidth / 2.0f), ((int)screenHeight / 2.0f));
			sWorld->createSystem(b, v.edges, RASF_CONSTANT, 10.0f);
		}
			return;
		case 'd':
		{
			Voronoi v(screenWidth* INVSCALE, screenHeight * INVSCALE, 100, UNIFORM_RANDOM);
			Border b((-(int)screenWidth / 2.0f), (-(int)screenHeight / 2.0f), ((int)screenWidth / 2.0f), ((int)screenHeight / 2.0f));
			sWorld->createSystem(b, v.edges, RASF_PSEUDORANDOM, 1.0f);
		}
			return;
		case 'e':
		{
			Voronoi v(screenWidth* INVSCALE, screenHeight* INVSCALE, 40, UNIFORM_RANDOM);
			Border b((-(int)screenWidth / 2.0f), (-(int)screenHeight / 2.0f), ((int)screenWidth / 2.0f), ((int)screenHeight / 2.0f));
			sWorld->createSystem(b, v.edges, RASF_SEQUENTIAL_SIN, 1.0f);

		}
			return;
		default:
			std::cout << "Incorrect input." << std::endl;
			break;
		}
	}

}

void printHelpText() {
	std::cout << "Press [p] to play/pause." << std::endl;
	std::cout << "Press [SPACE] to show bodies." << std::endl;
	std::cout << "Press [ENTER] to save image." << std::endl;
}

int main() {
	srand(time(0));

	// Create world, without gravity
	b2Vec2 gravity(0.0f, 0.0f);
	SpringWorld sWorld(new b2World(gravity));
	
	unsigned int screenWidth = 1000;
	unsigned int screenHeight = 1000;
	
	decidePatternToCreate(&sWorld, screenWidth, screenHeight);

	// Create window
	sf::ContextSettings settings;
	settings.antialiasingLevel = 4;
	sf::RenderWindow window(sf::VideoMode(screenWidth, screenHeight), "PS", sf::Style::Default, settings);
	window.setFramerateLimit(0); // TODO: maybe enable vsync and just make springs faster (not that it matters for pattern synthesis)

	std::cout << "There are " << sWorld.getWorld()->GetBodyCount() << " bodies in the scene." << std::endl;
	
	printHelpText();
		
	bool playing = false;
	bool drawB = false;
	while (window.isOpen()) {

		if (playing) sWorld.update(1.0f / 60.0f);
		
		window.clear(sf::Color::White);
		// Render things

		drawEdges(sWorld.getSpringEdges(), &window);
		if (drawB) drawBodies(sWorld, &window);
		
		sf::Event event;
		while (window.pollEvent(event)) {
			switch(event.type) {
			case sf::Event::Closed:
				window.close();
				break;
			case sf::Event::MouseButtonPressed:
				sWorld.update(1.0f / 60.0f);
				break;
			case sf::Event::KeyReleased:
				switch (event.key.code) {
				case sf::Keyboard::P:
					playing = !playing;
					break;
				case sf::Keyboard::Space:
					drawB = !drawB;
					break;
				case sf::Keyboard::Enter:
					std::string filename;
					std::cout << "Save image as:" << std::endl;
					std::cin >> filename;
					filename = "OutputImages\\" + filename + ".png";
					saveScreenshot(window, filename);
					break;
				}
			}
		}
	

		// Display
		window.display();
	}


}

