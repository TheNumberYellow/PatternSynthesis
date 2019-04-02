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
	std::cout << "Press [2] for basic LERP RASF." << std::endl;
	std::cout << "Press [3] for average angle RASF." << std::endl;
	
	int input = 0;
	std::cin >> input;
	switch (input) {
	case 1:
		return RASF_CONSTANT;
	case 2:
		return RASF_BASIC_LERP;
	case 3:
		return RASF_AVERAGE;
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
		int input;
		std::cin >> input;
		
		switch (input) {
		case 1:
		{
			unsigned int springs = 0;
			std::cout << "Springs in line?" << std::endl;
			std::cin >> springs;
			std::cout << "Creating line." << std::endl;
			
		}
			return;
		case 2:
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
		case 3:
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
		case 4:
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
			
			Voronoi v(screenWidth * INVSCALE, screenHeight * INVSCALE, numPoints);
			Border b((-(int)screenWidth / 2.0f), (-(int)screenHeight / 2.0f), ((int)screenWidth / 2.0f), ((int)screenHeight / 2.0f));
			sWorld->createSystem(b, v.edges, type, rasfValue);
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
	// Create world, without gravity
	b2Vec2 gravity(0.0f, 0.0f);
	SpringWorld sWorld(new b2World(gravity));
	
	unsigned int screenWidth = 800;
	unsigned int screenHeight = 800;
	
	auto func = [](float32 start, float32 end, float32 T) {
		return 0.0f;
	};
	
	//sWorld.createSpringLine(b2Vec2(-400 * INVSCALE, 0.0f), b2Vec2(400 * INVSCALE, 0.0f), 2, func, true);
	//sWorld.createSpringLine(b2Vec2(400 * INVSCALE, 0.0f), b2Vec2(-400 * INVSCALE, 0.0f), 2, func, true);
	

	decidePatternToCreate(&sWorld, screenWidth, screenHeight);
	//sWorld.createSpringBox(6, 20);

	//for (int i = 0; i < sWorld.springLines.size(); i++) {
	//	std::cout << "SpringLine " << i << " has angle " << sWorld.springLines[i]->startAngles[0] * RADTODEG << " at start and angle " << sWorld.springLines[i]->endAngles[0] * RADTODEG << " at end.\n";
	//}

	// Create window
	sf::ContextSettings settings;
	settings.antialiasingLevel = 4;
	sf::RenderWindow window(sf::VideoMode(screenWidth, screenHeight), "PSTest", sf::Style::Default, settings);
	window.setFramerateLimit(0); // TODO: maybe enable vsync and just make springs faster (not that it matters for pattern synthesis)

	time_t timeStart = time(NULL);

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

