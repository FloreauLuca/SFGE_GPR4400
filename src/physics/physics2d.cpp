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

#include <physics/physics2d.h>
#include <python/python_engine.h>
#include <engine/config.h>
#include <engine/engine.h>
namespace sfge
{

const float Physics2dManager::pixelPerMeter = 100.0f;


void Physics2dManager::OnEngineInit()
{
	p2Vec2 gravity;
	if(const auto configPtr = m_Engine.GetConfig())
		gravity = configPtr->gravity;
	m_World = std::make_shared<p2World>(gravity);
	m_ContactListener = std::make_unique<ContactListener>(m_Engine);
	m_World->SetContactListener(m_ContactListener.get());

	m_BodyManager.OnEngineInit();
	m_ColliderManager.OnEngineInit();
}

void Physics2dManager::OnUpdate(float dt)
{
	(void)dt;
}

void Physics2dManager::OnFixedUpdate()
{
	rmt_ScopedCPUSample(Physics2dManager,0);
	const auto config = m_Engine.GetConfig();
	if (config != nullptr and m_World != nullptr)
	{
		m_World->Step(config->fixedDeltaTime);
		m_BodyManager.OnFixedUpdate();
	}
}

std::weak_ptr<p2World> Physics2dManager::GetWorld() const
{
	return m_World;
}


void Physics2dManager::Destroy()
{
	if (m_World != nullptr)
	{
		m_World = nullptr;
	}
	if (m_ContactListener != nullptr)
	{
		m_ContactListener = nullptr;
	}

}

void Physics2dManager::OnBeforeSceneLoad()
{
	Destroy();
	OnEngineInit();
}

void Physics2dManager::OnAfterSceneLoad()
{
}


Body2dManager* Physics2dManager::GetBodyManager()
{
	return &m_BodyManager;
}

ColliderManager* Physics2dManager::GetColliderManager()
{
	return &m_ColliderManager;
}
p2World* Physics2dManager::Getp2World()
{
	return m_World.get();
}
/*
float Physics2dManager::Raycast(Vec2f startPoint, Vec2f direction, float rayLength)
{
	RaycastCallback rayCastCallback;
	rayCastCallback.fraction = 1.0f;
	m_World->RayCast(&rayCastCallback, pixel2meter(startPoint), pixel2meter(startPoint+direction*rayLength));
	return rayCastCallback.fraction;
}
*/
void ContactListener::BeginContact(p2Contact* contact)
{
	auto* pythonEngine = m_Engine.GetPythonEngine();
	const auto colliderA = static_cast<ColliderData*>(contact->GetColliderA()->GetUserData());
	const auto colliderB = static_cast<ColliderData*>(contact->GetColliderB()->GetUserData());
	/*
	{
		std::ostringstream oss;
		oss << "Begin Contact between: " << colliderA->entity << " and: " << colliderB->entity;
		Log::GetInstance()->Msg(oss.str());
	}
	*/
	auto& pySystems = pythonEngine->GetPySystemManager().GetPySystems();
	for(size_t i = 0;i < pySystems.size();i++)
	{
		if(pySystems[i] != nullptr)
		{
			pySystems[i]->OnContact(colliderA, colliderB, true);
		}
	}
		
}

void ContactListener::EndContact(p2Contact* contact)
{
	auto pythonEngine = m_Engine.GetPythonEngine();
	auto* colliderA = static_cast<ColliderData*>(contact->GetColliderA()->GetUserData());
	auto* colliderB = static_cast<ColliderData*>(contact->GetColliderB()->GetUserData());

	/*{
		std::ostringstream oss;
		oss << "End Contact between: " << colliderA->entity << " and: " << colliderB->entity;
		Log::GetInstance()->Msg(oss.str());
	}*/

	auto& pySystems = pythonEngine->GetPySystemManager().GetPySystems();
	for (size_t i = 0; i < pySystems.size(); i++)
	{
		if (pySystems[i] != nullptr)
		{
			pySystems[i]->OnContact(colliderA, colliderB, false);
		}
	}
}


float pixel2meter(float pixel)
{
	return pixel / Physics2dManager::pixelPerMeter;
}

float pixel2meter(int pixel)
{
	return pixel / Physics2dManager::pixelPerMeter;
}

p2Vec2 pixel2meter(sf::Vector2f pixel)
{
	return p2Vec2(pixel2meter(pixel.x), pixel2meter(pixel.y));
}

p2Vec2 pixel2meter(sf::Vector2i pixel)
{
	return p2Vec2(pixel2meter(pixel.x), pixel2meter(pixel.y));
}

float meter2pixel(float meter)
{
	return meter * Physics2dManager::pixelPerMeter;
}

p2Vec2 pixel2meter(Vec2f pixel)
{
	return p2Vec2(pixel2meter(pixel.x), pixel2meter(pixel.y));
}
Vec2f meter2pixel(p2Vec2 meter)
{
	return Vec2f(meter2pixel(meter.x), meter2pixel(meter.y));
}

ContactListener::ContactListener(Engine& engine):
	m_Engine(engine)
{
}
/*
float32 RaycastCallback::ReportFixture(b2Fixture *fixture, const p2Vec2 &point, const p2Vec2 &normal, float32 fraction)
{
	if(fraction < this->fraction)
	{
		this->fraction = fraction;
		touchedFixture = fixture;
	}
	return 1.0f;
}
*/
}
