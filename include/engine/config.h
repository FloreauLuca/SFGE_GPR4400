/*
MIT License

Copyright (c) 2017 SAE Institute Switzerland AG

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef SFGE_CONFIG_H
#define SFGE_CONFIG_H

//STL includes
#include <list>
#include <memory>

//Externals include
#include <SFML/System/Vector2.hpp>
//tool_engine include
#include <utility/json_utility.h>
#include <engine/globals.h>
#include "SFML/Graphics/Color.hpp"
#include "p2vector.h"

namespace sfge
{



/**
* \brief Used by the Engine to get a Configuration.
*/
struct Configuration 
{
	~Configuration();
	bool devMode = true;
	bool editor = true;
	bool windowLess = false;
	/**
	 * \brief The screen resolution used for the editor
	 */
	sf::Vector2i screenResolution = sf::Vector2i(1280, 720);

	p2Vec2 gravity = p2Vec2(0.0f, 9.81f);
	/**
	 * \brief The limited framerate
	 */
	unsigned int maxFramerate = 60;
	float fixedDeltaTime = 0.02f;
	int velocityIterations = 8;
	int positionIterations = 2;
	size_t currentEntitiesNmb = INIT_ENTITY_NMB;

	std::string windowName = "SFGE 1.1";
	std::string scriptsDirname = "scripts/";
	std::string dataDirname = "data/";

	sf::Color bgColor = sf::Color::Black;
	/**
	* \brief Used to load the overall Configuration of the GameEngine at start
	*/
	static std::unique_ptr<Configuration> LoadConfig(std::string configFilename);
	static std::unique_ptr<Configuration> LoadConfig(json& configJson);

};
}
#endif // !CONFIG_H
