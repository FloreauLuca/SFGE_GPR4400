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

#include <extensions/planet_system.h>
#include <engine/engine.h>
#include <engine/config.h>
#include <graphics/graphics2d.h>
#include <physics/body2d.h>
#include <physics/physics2d.h>


namespace sfge::ext
{
	PlanetSystem::PlanetSystem(Engine& engine):
		System(engine)

	{
	}

	void PlanetSystem::OnEngineInit()
	{
		m_Transform2DManager = m_Engine.GetTransform2dManager();
		m_Body2DManager = m_Engine.GetPhysicsManager()->GetBodyManager();
		m_TextureManager = m_Engine.GetGraphics2dManager()->GetTextureManager();
#ifdef WITH_VERTEXARRAY
		m_Graphics2DManager = m_Engine.GetGraphics2dManager();
#endif
		m_SpriteManager = m_Engine.GetGraphics2dManager()->GetSpriteManager();


		auto config = m_Engine.GetConfig();
		fixedDeltaTime = config->fixedDeltaTime;
		screenSize = sf::Vector2f(config->screenResolution.x, config->screenResolution.y);
		auto* entityManager = m_Engine.GetEntityManager();
		entityManager->ResizeEntityNmb(entitiesNmb);

#ifdef WITH_VERTEXARRAY
		const auto textureId = m_TextureManager->LoadTexture("data/sprites/round.png");
		texture = m_TextureManager->GetTexture(textureId);
		textureSize = sf::Vector2f(texture->getSize().x, texture->getSize().y);
#endif

		for (auto i = 0u; i < entitiesNmb; i++)
		{
			const auto newEntity = entityManager->CreateEntity(i + 1);

#ifdef MULTI_THREAD
		m_Positions[i] = sf::Vector2f(std::rand() % static_cast<int>(screenSize.x), std::rand() % static_cast<int>(screenSize.y));
#else
			auto transformPtr = m_Transform2DManager->AddComponent(newEntity);
			transformPtr->Position = sf::Vector2f(std::rand() % static_cast<int>(screenSize.x), std::rand() % static_cast<int>(screenSize.y));
#endif
#ifdef WITH_PHYSICS
			auto body = m_Body2DManager->AddComponent(newEntity);
			body->SetLinearVelocity(CalculateInitSpeed(transformPtr->Position));
#else
#ifndef MULTI_THREAD
		m_Velocities[i] = meter2pixel(CalculateInitSpeed(transformPtr->Position));
#else

		m_Velocities[i] = meter2pixel(CalculateInitSpeed(m_Positions[i]));
#endif
#endif

#ifndef WITH_VERTEXARRAY
		std::string texturePath = "data/sprites/round.png";
		const auto textureId = m_TextureManager->LoadTexture(texturePath);
		const auto texture = m_TextureManager->GetTexture(textureId);

		auto sprite = m_SpriteManager->AddComponent(newEntity);
		sprite->SetTexture(texture);
		
		auto& spriteInfo = m_SpriteManager->GetComponentInfo(newEntity);
		spriteInfo.name = "Sprite";
		//spriteInfo.sprite = sprite;
		spriteInfo.textureId = textureId;
		spriteInfo.texturePath = texturePath;

#else
			m_VertexArray[4 * i].texCoords = sf::Vector2f(0, 0);
			m_VertexArray[4 * i + 1].texCoords = sf::Vector2f(textureSize.x, 0);
			m_VertexArray[4 * i + 2].texCoords = textureSize;
			m_VertexArray[4 * i + 3].texCoords = sf::Vector2f(0, textureSize.y);
#endif
		}
	}

	void PlanetSystem::OnUpdate(float dt)
	{
		(void)dt;
	}

	void PlanetSystem::UpdateRange(int startIndex, int endIndex)
	{
#ifdef MULTI_THREAD
	for(int i = startIndex; i < endIndex; i++)
	{

		const auto force = meter2pixel(CalculateNewForce(m_Positions[i]));
		m_Velocities[i] += force / planetMass * fixedDeltaTime;
		m_Positions[i] += m_Velocities[i] * fixedDeltaTime;
		const auto pos = m_Positions[i];

		m_VertexArray[4 * i].position = pos - textureSize / 2.0f;
		m_VertexArray[4 * i + 1].position = pos + sf::Vector2f(textureSize.x / 2.0f, -textureSize.y / 2.0f);
		m_VertexArray[4 * i + 2].position = pos + textureSize / 2.0f;
		m_VertexArray[4 * i + 3].position = pos + sf::Vector2f(-textureSize.x / 2.0f, textureSize.y / 2.0f);
	}
#else
		(void)startIndex;
		(void)endIndex;
#endif
	}

	void PlanetSystem::OnFixedUpdate()
	{
		rmt_ScopedCPUSample(PlanetSystemFixedUpdate,0);
#ifdef MULTI_THREAD
	auto& threadPool = m_Engine.GetThreadPool();
	const auto coreNmb = threadPool.size();

	std::vector<std::future<void>> joinFutures(coreNmb);
	for(int threadIndex = 0; threadIndex < coreNmb;threadIndex++)
	{
		int start = (threadIndex + 1)*entitiesNmb/ (coreNmb + 1);
		int end = (threadIndex + 2)*entitiesNmb / (coreNmb + 1)-1;
		auto worldUpdateFunction = std::bind(&PlanetSystem::UpdateRange, this, start, end);
		joinFutures[threadIndex] = threadPool.push(worldUpdateFunction);
	}
	UpdateRange(0, entitiesNmb / (coreNmb + 1) - 1);
	for(int i = 0; i < coreNmb;i++)
	{
		joinFutures[i].get();
	}
#else
		for (auto i = 0u; i < entitiesNmb; i++)
		{
#ifdef WITH_PHYSICS
			const auto transformPtr = m_Engine.GetTransform2dManager()->GetComponentPtr(i + 1);
			auto bodyPtr = m_Engine.GetPhysicsManager()->GetBodyManager()->GetComponentPtr(i + 1);
			bodyPtr->ApplyForce(CalculateNewForce(transformPtr->Position) / planetMass * fixedDeltaTime);
#else
		auto transformPtr = m_Engine.GetTransform2dManager()->GetComponentPtr(i + 1);
		const auto force = meter2pixel(CalculateNewForce(transformPtr->Position));

		m_Velocities[i] += force / planetMass * fixedDeltaTime;
		transformPtr->Position += m_Velocities[i] * fixedDeltaTime;
#endif
#ifdef WITH_VERTEXARRAY
			const auto pos = transformPtr->Position;

			m_VertexArray[4 * i].position = pos - textureSize / 2.0f;
			m_VertexArray[4 * i + 1].position = pos + sf::Vector2f(textureSize.x / 2.0f, -textureSize.y / 2.0f);
			m_VertexArray[4 * i + 2].position = pos + textureSize / 2.0f;
			m_VertexArray[4 * i + 3].position = pos + sf::Vector2f(-textureSize.x / 2.0f, textureSize.y / 2.0f);


#endif
		}
#endif
	}

	void PlanetSystem::OnDraw()
	{
		rmt_ScopedCPUSample(PlanetSystemDraw,0);
#ifdef WITH_VERTEXARRAY
		sf::RenderStates renderStates;
		renderStates.texture = texture;
		auto window = m_Graphics2DManager->GetWindow();
		window->draw(m_VertexArray, renderStates);
#endif
	}

	p2Vec2 PlanetSystem::CalculateInitSpeed(sf::Vector2f position) const
	{
		const auto deltaToCenter = screenSize / 2.0f - position;
		auto velDir = sf::Vector2f(-deltaToCenter.y, deltaToCenter.x);
		velDir /= Magnitude(velDir);

		const auto speed = sqrtf(Magnitude(CalculateNewForce(position)) / planetMass * pixel2meter(Magnitude(deltaToCenter)));
		return p2Vec2(velDir.x * speed, velDir.y * speed);
	}

	p2Vec2 PlanetSystem::CalculateNewForce(sf::Vector2f position) const
	{
		const auto deltaToCenter = screenSize / 2.0f - position;
		const auto r = Magnitude(deltaToCenter);
		const auto force = gravityConst * centerMass * planetMass / (r * r);
		return pixel2meter(deltaToCenter / r * force);
	}

	float PlanetSystem::Magnitude(sf::Vector2f v)
	{
		return sqrtf(v.x * v.x + v.y * v.y);
	}

	float PlanetSystem::Magnitude(p2Vec2 v)
	{
		return sqrtf(v.x * v.x + v.y * v.y);
	}
}
