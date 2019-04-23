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

#include <memory>

#include <SFML/System/Time.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>


#include <engine/engine.h>
#include <engine/globals.h>

#include <utility/log.h>

#include <graphics/graphics2d.h>
#include <audio/audio.h>
#include <engine/scene.h>
#include <input/input.h>
#include <python/python_engine.h>
#include <physics/physics2d.h>
#include <editor/editor.h>
#include <engine/entity.h>
#include <engine/transform2d.h>


namespace sfge
{

struct SystemsContainer
{
public:
	SystemsContainer(Engine &engine) :
		graphics2dManager(engine),
		audioManager(engine),
		sceneManager(engine),
		inputManager(engine),
		pythonEngine(engine),
		physicsManager(engine),
		editor(engine),
		entityManager(engine),
		transformManager(engine)
	{

	}
	SystemsContainer(const SystemsContainer&) = delete;

	Graphics2dManager graphics2dManager;
	AudioManager audioManager;
	SceneManager sceneManager;
	InputManager inputManager;
	PythonEngine pythonEngine;
	Physics2dManager physicsManager;
	Editor editor;
	EntityManager entityManager;
	Transform2dManager transformManager;

};


Engine::Engine()
{
	m_SystemsContainer = std::make_unique<SystemsContainer>(*this);

	rmt_CreateGlobalInstance(&rmt);
}
Engine::~Engine()
{
	m_SystemsContainer = nullptr;
}

void Engine::Init(std::string configFilename)
{
	const auto configJsonPtr = LoadJson(configFilename);
	if (configJsonPtr)
		Init(*configJsonPtr);

}

void Engine::Init(json& configJson)
{
	m_Config = Configuration::LoadConfig(configJson);
	InitModules();
}

void Engine::Init(std::unique_ptr<Configuration> config)
{
	m_Config = std::move(config);
	InitModules();
}

void Engine::InitModules()
{
	m_EngineClock.restart();
	if (m_Config == nullptr)
	{
		Log::GetInstance()->Error("[Error] Game tool_engine Configuration");
	}
	else
	{
		Log::GetInstance()->Msg("Game tool_engine Configuration Successfull");
	}
    {
        std::ostringstream oss;
        oss << "Number of cores on machine: "<<std::thread::hardware_concurrency ();
        Log::GetInstance ()->Msg (oss.str ());
    }
    m_ThreadPool.resize(std::thread::hardware_concurrency ()-1);

	m_SystemsContainer->entityManager.OnEngineInit();
	m_SystemsContainer->transformManager.OnEngineInit();
	m_SystemsContainer->graphics2dManager.OnEngineInit();
	m_SystemsContainer->audioManager.OnEngineInit();
	m_SystemsContainer->sceneManager.OnEngineInit();
	m_SystemsContainer->inputManager.OnEngineInit();
	m_SystemsContainer->pythonEngine.OnEngineInit();
	m_SystemsContainer->physicsManager.OnEngineInit();
	m_SystemsContainer->editor.OnEngineInit();

	m_Window = m_SystemsContainer->graphics2dManager.GetWindow();
	running = true;
}

void Engine::Start()
{
	sf::Clock globalClock;
	sf::Clock updateClock;
	sf::Clock fixedUpdateClock;
	sf::Clock graphicsUpdateClock;
	sf::Time previousFixedUpdateTime = globalClock.getElapsedTime();
	sf::Time deltaFixedUpdateTime = sf::Time();
	sf::Time dt = sf::Time();

	rmt_BindOpenGL();
	while (running && m_Window != nullptr)
	{

		rmt_ScopedOpenGLSample(SFGE_Frame_GL);
		rmt_ScopedCPUSample(SFGE_Frame,0)

		bool isFixedUpdateFrame = false;
		sf::Event event{};
		while (m_Window != nullptr && 
			m_Window->pollEvent(event))
		{
            m_SystemsContainer->editor.ProcessEvent(event);
			if (event.type == sf::Event::Closed)
			{
				running = false;
				m_Window->close();
			}

			if(event.type == sf::Event::Resized)
			{
				m_Config->screenResolution = sf::Vector2i(event.size.width, event.size.height);

			}
			
		}
		if (!running)
		{
			continue;
		}


		m_SystemsContainer->inputManager.OnUpdate(dt.asSeconds());
		auto fixedUpdateTime = globalClock.getElapsedTime() - previousFixedUpdateTime;
		if (fixedUpdateTime.asSeconds() > m_Config->fixedDeltaTime)
		{
			fixedUpdateClock.restart ();
			m_SystemsContainer->physicsManager.OnFixedUpdate();
			previousFixedUpdateTime = globalClock.getElapsedTime();
			m_SystemsContainer->pythonEngine.OnFixedUpdate();
			m_SystemsContainer->sceneManager.OnFixedUpdate();
			deltaFixedUpdateTime = fixedUpdateClock.getElapsedTime ();
			m_FrameData.frameFixedUpdate = deltaFixedUpdateTime;
			isFixedUpdateFrame = true;
		}
		m_SystemsContainer->pythonEngine.OnUpdate(dt.asSeconds());

		m_SystemsContainer->sceneManager.OnUpdate(dt.asSeconds());

		m_SystemsContainer->editor.OnUpdate(dt.asSeconds());
        m_SystemsContainer->transformManager.OnUpdate(dt.asSeconds());
		m_SystemsContainer->graphics2dManager.OnUpdate(dt.asSeconds());



		graphicsUpdateClock.restart();

		m_SystemsContainer->graphics2dManager.OnDraw();

		m_SystemsContainer->pythonEngine.OnDraw();
		m_SystemsContainer->sceneManager.OnDraw();
		m_SystemsContainer->editor.OnDraw();

        m_SystemsContainer->graphics2dManager.Display();
		const sf::Time graphicsDt = graphicsUpdateClock.getElapsedTime ();
		dt = updateClock.restart();
		if(isFixedUpdateFrame)
		{
			m_FrameData.graphicsTime = graphicsDt;
			m_FrameData.frameTotalTime = dt;
		}
		m_DeltaTime = dt.asSeconds();
	}

	rmt_UnbindOpenGL();
	Destroy();
}

void Engine::Destroy() 
{

	m_SystemsContainer->pythonEngine.Destroy();
	m_SystemsContainer->entityManager.Destroy();
	m_SystemsContainer->graphics2dManager.Destroy();
	m_SystemsContainer->audioManager.Destroy();
	m_SystemsContainer->sceneManager.Destroy();
	m_SystemsContainer->inputManager.Destroy();
	m_SystemsContainer->editor.Destroy();
	m_SystemsContainer->physicsManager.Destroy();
	rmt_DestroyGlobalInstance(rmt);

}

void Engine::Clear() 
{
	m_SystemsContainer->entityManager.OnBeforeSceneLoad();
	m_SystemsContainer->graphics2dManager.OnBeforeSceneLoad();
	m_SystemsContainer->audioManager.OnBeforeSceneLoad();
	m_SystemsContainer->sceneManager.OnBeforeSceneLoad();
	m_SystemsContainer->inputManager.OnBeforeSceneLoad();
	m_SystemsContainer->pythonEngine.OnBeforeSceneLoad();
	m_SystemsContainer->editor.OnBeforeSceneLoad();
	m_SystemsContainer->physicsManager.OnBeforeSceneLoad();
}

void Engine::Collect() 
{

	m_SystemsContainer->entityManager.OnAfterSceneLoad();
	m_SystemsContainer->graphics2dManager.OnAfterSceneLoad();
	m_SystemsContainer->audioManager.OnAfterSceneLoad();
	m_SystemsContainer->sceneManager.OnAfterSceneLoad();
	m_SystemsContainer->inputManager.OnAfterSceneLoad();
	m_SystemsContainer->pythonEngine.OnAfterSceneLoad();
	m_SystemsContainer->editor.OnAfterSceneLoad();
	m_SystemsContainer->physicsManager.OnAfterSceneLoad();
}


Configuration * Engine::GetConfig() const
{
	return m_Config.get();
}

Graphics2dManager* Engine::GetGraphics2dManager()
{
	return m_SystemsContainer?&m_SystemsContainer->graphics2dManager:nullptr;
}


AudioManager* Engine::GetAudioManager()
{
	return m_SystemsContainer ? &m_SystemsContainer->audioManager : nullptr;
}

SceneManager* Engine::GetSceneManager() 
{
	return m_SystemsContainer ? &m_SystemsContainer->sceneManager : nullptr;
}

InputManager* Engine::GetInputManager() 
{
	return m_SystemsContainer ? &m_SystemsContainer->inputManager : nullptr;
}

PythonEngine* Engine::GetPythonEngine() 
{
	return m_SystemsContainer ? &m_SystemsContainer->pythonEngine : nullptr;
}

Physics2dManager* Engine::GetPhysicsManager() 
{
	return m_SystemsContainer ? &m_SystemsContainer->physicsManager : nullptr;
}

EntityManager* Engine::GetEntityManager() 
{
	return m_SystemsContainer ? &m_SystemsContainer->entityManager : nullptr;
}

Transform2dManager* Engine::GetTransform2dManager() 
{
	return m_SystemsContainer ? &m_SystemsContainer->transformManager : nullptr;
}

Editor* Engine::GetEditor() 
{
	return m_SystemsContainer ? &m_SystemsContainer->editor : nullptr;
}

ctpl::thread_pool & Engine::GetThreadPool()
{
	return m_ThreadPool;
}

ProfilerFrameData& Engine::GetProfilerFrameData()
{
    return m_FrameData;
}

float Engine::GetTimeSinceInit()
{
	return m_EngineClock.getElapsedTime().asSeconds();
}

float Engine::GetDeltaTime()
{
	return m_DeltaTime;
}
}
