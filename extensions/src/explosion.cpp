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

#include <extensions/explosion.h>
#include <engine/engine.h>
#include <engine/config.h>
#include <graphics/graphics2d.h>
#include <physics/body2d.h>
#include <physics/physics2d.h>
#include <corecrt_math_defines.h>
#include "input/input.h"

namespace sfge::ext
{
	Explosion::Explosion(Engine& engine) :
		System(engine)
	{
	}

	void Explosion::OnEngineInit()
	{
		m_Transform2DManager = m_Engine.GetTransform2dManager();
		m_Body2DManager = m_Engine.GetPhysicsManager()->GetBodyManager();
		m_TextureManager = m_Engine.GetGraphics2dManager()->GetTextureManager();
		m_SpriteManager = m_Engine.GetGraphics2dManager()->GetSpriteManager();
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
		Bombe = entityManager->GetEntityByName("Bombe");
	}

	void Explosion::OnUpdate(float dt)
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

	void Explosion::OnFixedUpdate()
	{
		rmt_ScopedCPUSample(FlipperScriptFixedUpdate, 0);
	}


	void Explosion::OnDraw()
	{
		auto bombe = m_Body2DManager->GetComponentPtr(Bombe)->GetBody();
		auto bombeSprite = m_SpriteManager->GetComponentPtr(Bombe);
		
		if (m_InputManager->GetKeyboardManager().IsKeyHeld(sf::Keyboard::Space))
		{
			std::string texturePath = "data/sprites/round.png";
			const auto textureId = m_TextureManager->LoadTexture(texturePath);
			const auto texture = m_TextureManager->GetTexture(textureId);
			bombeSprite->SetTexture(texture);
			float power = 5;
			float radius = 5;
			for (p2Body* body : bodies)
			{
				float dist = (bombe->GetPosition() - body->GetPosition()).GetMagnitude();
				if (dist == 0 || dist >= radius || bombe == body) continue;
				p2Vec2 force = (body->GetPosition() - bombe->GetPosition()).Normalized() * power;
				m_Graphics2DManager->DrawVector(meter2pixel(force), meter2pixel(body->GetPosition()), sf::Color::Red);
				body->ApplyForceToCenter(force);
			}
		}
		else
		{
			std::string texturePath = "data/sprites/bomb.png";
			const auto textureId = m_TextureManager->LoadTexture(texturePath);
			const auto texture = m_TextureManager->GetTexture(textureId);
			bombeSprite->SetTexture(texture);
		}
	}
}
