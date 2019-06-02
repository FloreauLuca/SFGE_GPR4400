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

#include <extensions/quad_tree_test.h>
#include <engine/engine.h>
#include <engine/config.h>
#include <graphics/graphics2d.h>
#include <physics/body2d.h>
#include <physics/physics2d.h>



namespace sfge::ext
{

	QuadTreeTest::QuadTreeTest(Engine& engine) :
		System(engine)
	{

	}
	
	void QuadTreeTest::OnEngineInit()
	{
		m_Transform2DManager = m_Engine.GetTransform2dManager();
		m_Physics2DManager = m_Engine.GetPhysicsManager();
		m_TextureManager = m_Engine.GetGraphics2dManager()->GetTextureManager();
		m_SpriteManager = m_Engine.GetGraphics2dManager()->GetSpriteManager();
		m_Graphics2DManager = m_Engine.GetGraphics2dManager();
		m_ShapeManager = m_Engine.GetGraphics2dManager()->GetShapeManager();
		m_Body2DManager = m_Engine.GetPhysicsManager()->GetBodyManager();

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

	void QuadTreeTest::OnUpdate(float dt)
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


	void QuadTreeTest::OnFixedUpdate()
	{
		rmt_ScopedCPUSample(QuadTreeTestFixedUpdate, 0);

	}

	void QuadTreeTest::OnDraw()
	{
		rmt_ScopedCPUSample(QuadTreeTestDraw, 0);
		m_QuadTree = m_Physics2DManager->Getp2World()->GetQuadtree();
		colorIterator = 0;
		DrawQuadTree(m_QuadTree);
	}

	void QuadTreeTest::DrawQuadTree(p2QuadTree* quadTree)
	{
		p2AABB aabb = quadTree->GetBounds();
		m_Graphics2DManager->DrawLine(meter2pixel(p2Vec2(aabb.topRight.x, aabb.topRight.y)),
		                              meter2pixel(p2Vec2(aabb.bottomLeft.x, aabb.topRight.y)), sf::Color::Cyan);
		m_Graphics2DManager->DrawLine(meter2pixel(p2Vec2(aabb.topRight.x, aabb.topRight.y)),
		                              meter2pixel(p2Vec2(aabb.topRight.x, aabb.bottomLeft.y)), sf::Color::Cyan);
		m_Graphics2DManager->DrawLine(meter2pixel(p2Vec2(aabb.bottomLeft.x, aabb.bottomLeft.y)),
		                              meter2pixel(p2Vec2(aabb.bottomLeft.x, aabb.topRight.y)), sf::Color::Cyan);
		m_Graphics2DManager->DrawLine(meter2pixel(p2Vec2(aabb.bottomLeft.x, aabb.bottomLeft.y)),
		                              meter2pixel(p2Vec2(aabb.topRight.x, aabb.bottomLeft.y)), sf::Color::Cyan);
		
		std::vector<p2Body*> objects = quadTree->GetObjects();
		for (p2Body* object : objects)
		{
			for (int i = 0; i < bodies.size(); i++)
			{
				if (bodies[i] == object)
				{
					Shape* shape = m_ShapeManager->GetComponentPtr(entities[i]);
					if (quadTree->GetChild()[0] == nullptr)
					{
						//shape->SetFillColor(colors[colorIterator % colors.size()]);
					}
					else
					{
						//shape->SetFillColor(sf::Color::White);
					}
				}
			}
		}
		
		if (quadTree->GetChild()[0] == nullptr)
		{
			colorIterator++;
		}
		else
		{
			for (int i = 0; i < 4; i++)
			{
				DrawQuadTree(quadTree->GetChild()[i]);
			}
		}
	}
}
