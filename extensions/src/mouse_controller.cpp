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
#include "extensions/mouse_controller.h"


namespace sfge::ext
{

	MouseController::MouseController(Engine& engine) :
		System(engine)
	{

	}

	void MouseController::OnEngineInit()
	{
		m_Transform2DManager = m_Engine.GetTransform2dManager();
		m_Body2DManager = m_Engine.GetPhysicsManager()->GetBodyManager();
		m_ColliderManager = m_Engine.GetPhysicsManager()->GetColliderManager();
		m_TextureManager = m_Engine.GetGraphics2dManager()->GetTextureManager();
		m_SpriteManager = m_Engine.GetGraphics2dManager()->GetSpriteManager();
		m_ShapeManager = m_Engine.GetGraphics2dManager()->GetShapeManager();
		m_Graphics2DManager = m_Engine.GetGraphics2dManager();
		m_InputManager = m_Engine.GetInputManager();


		auto config = m_Engine.GetConfig();
		fixedDeltaTime = config->fixedDeltaTime;
		screenSize = sf::Vector2f(config->screenResolution.x, config->screenResolution.y);
		auto* entityManager = m_Engine.GetEntityManager();

		entities = entityManager->GetEntitiesWithType(ComponentType::BODY2D);
		for (auto i = 0u; i < entities.size(); i++)
		{
			auto body = m_Body2DManager->GetComponentPtr(entities[i]);
			bodies.push_back(body->GetBody());
		}
	}

	void MouseController::OnUpdate(float dt)
	{
		(void)dt;
		MouseManager& mouseManager = m_InputManager->GetMouseManager();
		p2Vec2 mousePosition = pixel2meter(mouseManager.GetPosition());
		if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
		{
			if (currentBody == nullptr)
			{
				for (p2Body* body : bodies)
				{
					if ((body->GetPosition() - mousePosition).GetMagnitude() < 0.5f)
					{
						currentBody = body;
						break;
					}
				}
			}
			if (currentBody != nullptr)
			{
				currentBody->SetPosition(mousePosition);
				currentBody->SetLinearVelocity(p2Vec2());
				currentBody->SetAngularVelocity(0);
			}
		} else if (sf::Mouse::isButtonPressed(sf::Mouse::Right) && currentBody == nullptr)
		{
			if (MAX_NEW_ENTITY > bodies.size())
			{
				auto* entityManager = m_Engine.GetEntityManager();
				const auto newEntity = entityManager->CreateEntity(0);

				auto transformPtr = m_Transform2DManager->AddComponent(newEntity);
				transformPtr->Position = sf::Vector2f(meter2pixel(mousePosition));
				auto body = m_Body2DManager->AddComponent(newEntity);
				body->GetBody()->ChangeType(p2BodyType::DYNAMIC);
				currentBody = body->GetBody();
				bodies.push_back(currentBody);
				auto collider = m_ColliderManager->AddComponent(newEntity);
				auto shape = m_ShapeManager->AddComponent(newEntity);
			}

		} else if (!sf::Mouse::isButtonPressed(sf::Mouse::Right))
		{
		currentBody = nullptr;
		}
		if (m_InputManager->GetKeyboardManager().IsKeyHeld(sf::Keyboard::V))
		{
			std::cout << std::to_string(mouseManager.GetPosition().x) + " , " + std::to_string(mouseManager.GetPosition().y) << std::endl;
		}
		if (m_InputManager->GetKeyboardManager().IsKeyHeld(sf::Keyboard::B))
		{
			std::cout << std::to_string(pixel2meter(mouseManager.GetPosition().x)) + " , " + std::to_string(pixel2meter(mouseManager.GetPosition().y)) << std::endl;
		}
	}

	void MouseController::OnFixedUpdate()
	{
		rmt_ScopedCPUSample(StayOnScreenFixedUpdate, 0);
		auto config = m_Engine.GetConfig();
		fixedDeltaTime = config->fixedDeltaTime;
		screenSize = sf::Vector2f(config->screenResolution.x, config->screenResolution.y);
		for (auto entity : entities)
		{
			auto transform = m_Transform2DManager->GetComponentPtr(entity);
			auto body = m_Body2DManager->GetComponentPtr(entity);
			auto position = transform->Position;
			if ((position.x < 0 && body->GetLinearVelocity().x < 0) || (position.x > screenSize.x && body->GetLinearVelocity().x > 0))
			{
				body->SetLinearVelocity(p2Vec2(-body->GetLinearVelocity().x, body->GetLinearVelocity().y));
			}
			if ((position.y < 0 && body->GetLinearVelocity().y < 0) || (position.y > screenSize.y && body->GetLinearVelocity().y > 0))
			{
				body->SetLinearVelocity(p2Vec2(body->GetLinearVelocity().x, -body->GetLinearVelocity().y));
			}
		}
	}

}
