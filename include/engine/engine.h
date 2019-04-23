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

#ifndef SFGE_ENGINE_H
#define SFGE_ENGINE_H



#include <memory>
#include <string>
#include <ctpl_stl.h>

#include <engine/config.h>
#include <utility/json_utility.h>

#include <editor/profiler.h>
#include <Remotery.h>

#include <SFML/System/Clock.hpp>
namespace sf
{
class RenderWindow;
}
namespace sfge
{

/**
* Prototypes declarations
*/

struct Configuration;
class Graphics2dManager;
class AudioManager;
class SceneManager;
class InputManager;
class PythonEngine;
class Physics2dManager;
class EntityManager;
class Transform2dManager;
class Editor;
struct SystemsContainer;

/**
* \brief The main Engine class to centralize the frame process and the references
*/
class Engine
{
public:
	Engine();
	/**
	* \brief Initialize all the modules of the Game Engine, reading the config file too
	*/
	void Init(std::string configFilename="data/config.json");
	void Init(json& configJson);
	void Init(std::unique_ptr<Configuration> config);
	/**
	* \brief Starting the Game Engine after the Init()
	*/
	void Start();

	/**
	 * \brief Destroy all the modules
	 */
	void Destroy();
	void Clear();
	/**
	* \brief Reload is used after loading a new scene
	*/
	void Collect();

	~Engine();
	/**
	* \brief A getter of the Configuration
	* \return The Configuration struct got by the Engine
	*/
	Configuration * GetConfig() const;

	Graphics2dManager* GetGraphics2dManager();
	AudioManager* GetAudioManager();
	SceneManager* GetSceneManager();
	InputManager* GetInputManager();
	PythonEngine* GetPythonEngine();
	Physics2dManager* GetPhysicsManager();
	EntityManager* GetEntityManager();
	Transform2dManager* GetTransform2dManager();
	Editor* GetEditor();

	ctpl::thread_pool& GetThreadPool();
	ProfilerFrameData& GetProfilerFrameData();
	float GetTimeSinceInit();
	float GetDeltaTime();
	bool running = false;
protected:
	void InitModules();
	ctpl::thread_pool m_ThreadPool;
	sf::RenderWindow* m_Window = nullptr;
	std::unique_ptr<Configuration> m_Config;
	float m_DeltaTime = 0.0f;
	sf::Clock m_EngineClock;
	Remotery* rmt;
	//
	std::unique_ptr<SystemsContainer> m_SystemsContainer;

  	ProfilerFrameData m_FrameData;

};


}

#endif // !SFGE_ENGINE_H
