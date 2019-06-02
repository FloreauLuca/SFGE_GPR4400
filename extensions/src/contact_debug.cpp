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

#include <engine/engine.h>
#include <engine/config.h>
#include <graphics/graphics2d.h>
#include <physics/body2d.h>
#include <physics/physics2d.h>
#include "extensions/contact_debug.h"


namespace sfge::ext
{

	ContactDebug::ContactDebug(Engine& engine) :
		System(engine)
	{

	}

	void ContactDebug::OnEngineInit()
	{
		m_Transform2DManager = m_Engine.GetTransform2dManager();
		m_Body2DManager = m_Engine.GetPhysicsManager()->GetBodyManager();
		m_TextureManager = m_Engine.GetGraphics2dManager()->GetTextureManager();
		m_SpriteManager = m_Engine.GetGraphics2dManager()->GetSpriteManager();
		m_ShapeManager = m_Engine.GetGraphics2dManager()->GetShapeManager();
		m_Graphics2DManager = m_Engine.GetGraphics2dManager();


		auto config = m_Engine.GetConfig();
		fixedDeltaTime = config->fixedDeltaTime;
		screenSize = sf::Vector2f(config->screenResolution.x, config->screenResolution.y);
		auto* entityManager = m_Engine.GetEntityManager();

		entities = entityManager->GetEntitiesWithType(ComponentType::BODY2D);
		for (auto i = 0u; i < entities.size(); i++)
		{
			auto body = m_Body2DManager->GetComponentPtr(entities[i]);
			bodies.push_back(body->GetBody());
			auto shape = m_ShapeManager->GetComponentPtr(entities[i]);
			shape->SetFillColor(sf::Color::Red);
		}
		count.resize(entities.size());
	}

	void ContactDebug::OnUpdate(float dt)
	{
		(void)dt;
		/*
		for (auto i = 0u; i < entities.size(); i++)
		{
			auto transform = m_Transform2DManager->GetComponentPtr(entities[i]);
			transform->EulerAngle += 5*dt;
		}
		*/
	}


	void ContactDebug::OnFixedUpdate()
	{
		rmt_ScopedCPUSample(ContactDebugFixedUpdate, 0);

	}

	void ContactDebug::OnDraw()
	{
		rmt_ScopedCPUSample(ContactDebugDraw, 0);
		for (int i = 0; i < entities.size(); i++)
		{
			auto shape = m_ShapeManager->GetComponentPtr(entities[i]);
			if (count[i] > 0)
			{
				shape->SetFillColor(sf::Color::Green);
			}
			else
			{
				shape->SetFillColor(sf::Color::Magenta);

			}
		}
		
	}

	void ContactDebug::OnContact(ColliderData* c1, ColliderData* c2, bool enter)
	{
		if (enter)
		{
			for (int i = 0; i < entities.size(); i++)
			{
				if (entities[i] == c1->entity)
				{
					count[i] += 1;
				}
				if (entities[i] == c2->entity)
				{
					count[i] += 1;
				}
			}
		}
		else
		{
			for (int i = 0; i < entities.size(); i++)
			{
				if (entities[i] == c1->entity)
				{
					count[i] -= 1;
				}
				if (entities[i] == c2->entity)
				{
					count[i] -= 1;
				}
			}
		}
	}
}
