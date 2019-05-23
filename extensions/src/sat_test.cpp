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

#include <extensions/sat_test.h>
#include <engine/engine.h>
#include <engine/config.h>
#include <graphics/graphics2d.h>
#include <physics/body2d.h>
#include <physics/physics2d.h>
#include <corecrt_math_defines.h>

namespace sfge::ext
{
	SatTest::SatTest(Engine& engine) :
		System(engine)
	{
	}

	void SatTest::OnEngineInit()
	{
		m_Transform2DManager = m_Engine.GetTransform2dManager();
		m_Body2DManager = m_Engine.GetPhysicsManager()->GetBodyManager();
		m_TextureManager = m_Engine.GetGraphics2dManager()->GetTextureManager();
		m_SpriteManager = m_Engine.GetGraphics2dManager()->GetSpriteManager();
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
		}
	}

	void SatTest::OnUpdate(float dt)
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


	void SatTest::OnFixedUpdate()
	{
		rmt_ScopedCPUSample(AabbTestFixedUpdate, 0);
	}

	void SatTest::OnDraw()
	{
		rmt_ScopedCPUSample(AabbTestDraw, 0);
		for (auto i = 0u; i < bodies.size(); i++)
		{
			for (auto j = i; j < bodies.size(); j++)
			{
				if (bodies[i] == bodies[j]) continue;
				DrawSAT(bodies[i], bodies[j]);
			}
		}
	}

	void SatTest::DrawSAT(p2Body* bodyA, p2Body* bodyB)
	{
		float resize = 1;
		for (p2Collider colliderA : *bodyA->GetCollider())
		{
			for (p2Collider colliderB : *bodyB->GetCollider())
			{
				if (colliderA.GetColliderType() == p2ColliderType::CIRCLE)
				{
					if (colliderB.GetColliderType() == p2ColliderType::CIRCLE)
					{
						if (p2CircleShape* circleshapeA = dynamic_cast<p2CircleShape*>(colliderA.GetShape()))
						{
							if (p2CircleShape* circleshapeB = dynamic_cast<p2CircleShape*>(colliderB.GetShape()))
							{
								if (p2Vec2::Distance(bodyA->GetPosition(), bodyB->GetPosition()) < circleshapeA->GetRadius() + circleshapeB->GetRadius())
								{
									//return true;
								}
							}
						}
					}
					else if (colliderB.GetColliderType() == p2ColliderType::BOX)
					{
						if (p2CircleShape* circleshapeA = dynamic_cast<p2CircleShape*>(colliderA.GetShape()))
						{
							if (p2RectShape* rectShapeB = dynamic_cast<p2RectShape*>(colliderB.GetShape()))
							{
								p2Mat22 mtv;
								float newAngle = bodyB->GetAngle() / 180 * M_PI;
								p2Vec2 u = bodyA->GetPosition() - (bodyB->GetPosition());
								u = u.Normalized() * (u.GetMagnitude() - circleshapeA->GetRadius());
								p2Vec2 x = p2Vec2(rectShapeB->GetSize().x + circleshapeA->GetRadius(), 0).Rotate(newAngle);
								p2Vec2 y = p2Vec2(0, rectShapeB->GetSize().y + circleshapeA->GetRadius()).Rotate(newAngle);
								p2Vec2 resultX = x * (p2Vec2::Dot(u, x) / p2Vec2::Dot(x, x));
								p2Vec2 resultY = y * (p2Vec2::Dot(u, y) / p2Vec2::Dot(y, y));
								mtv = p2Mat22(p2Vec2(0, 0), rectShapeB->GetSize());
								float resultXMagn = resultX.GetMagnitude();
								float resultYMagn = resultY.GetMagnitude();
								float uMagn = u.GetMagnitude();

								float projX = resultX.GetMagnitude() / x.GetMagnitude();
								float projY = resultY.GetMagnitude() / y.GetMagnitude();


								float kx = (p2Vec2::Dot(u, x) / p2Vec2::Dot(x, x));
								float ky = (p2Vec2::Dot(u, y) / p2Vec2::Dot(y, y));
								if ((kx < 1 && kx > -1) && (ky < 1 && ky > -1))
								{
									if (kx < 1)
									{
										if (mtv.rows[1].GetMagnitude() > (x * (1 - kx)).GetMagnitude())
										{
											mtv.rows[1] = (x * (1 - kx));
											mtv.rows[0] = u + bodyB->GetPosition();
										}
									}
									if (kx > -1)
									{
										if (mtv.rows[1].GetMagnitude() > (x * (-1 - kx)).GetMagnitude())
										{
											mtv.rows[1] = (x * (-1 - kx));
											mtv.rows[0] = u + bodyB->GetPosition();
										}
									}
									if (ky < 1)
									{
										if (mtv.rows[1].GetMagnitude() > (y * (1 - ky)).GetMagnitude())
										{
											mtv.rows[1] = (y * (1 - ky));
											mtv.rows[0] = u + bodyB->GetPosition();
										}
									}
									if (ky > -1)
									{
										if (mtv.rows[1].GetMagnitude() > (y * (-1 - ky)).GetMagnitude())
										{
											mtv.rows[1] = (y * (-1 - ky));
											mtv.rows[0] = u + bodyB->GetPosition();
										}
									}
									m_Graphics2DManager->DrawVector(meter2pixel(mtv.rows[1]) * resize, meter2pixel(mtv.rows[0]) * resize, sf::Color::Red);
								}
								

								m_Graphics2DManager->DrawVector(meter2pixel(u) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Green);
								m_Graphics2DManager->DrawVector(meter2pixel(x) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Red);
								m_Graphics2DManager->DrawVector(meter2pixel(y) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Red);
								if (kx > 1 || kx < -1)
								{
									m_Graphics2DManager->DrawVector(meter2pixel(resultX) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Blue);
								}
								else
								{
									m_Graphics2DManager->DrawVector(meter2pixel(resultX) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Yellow);
								}
								if (ky > 1 || ky < -1)
								{
									m_Graphics2DManager->DrawVector(meter2pixel(resultY) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Blue);
								}
								else
								{
									m_Graphics2DManager->DrawVector(meter2pixel(resultY) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Yellow);
								}

								/*
								m_Graphics2DManager->DrawVector(meter2pixel(u) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Red);
								m_Graphics2DManager->DrawVector(meter2pixel(x) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Red);
								m_Graphics2DManager->DrawVector(meter2pixel(y) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Red);
								m_Graphics2DManager->DrawVector(meter2pixel(resultX) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Blue);
								m_Graphics2DManager->DrawVector(meter2pixel(resultY) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Blue);
								*/
								//return !((projX > 1 && projX > 1) || (projX < -1 && projX < -1) || (projY > 1 && projY > 1) || (projY < -1 && projY < -1));
							}
						}
					}
				}
				else if (colliderA.GetColliderType() == p2ColliderType::BOX)
				{
					if (colliderB.GetColliderType() == p2ColliderType::CIRCLE)
					{
						if (p2CircleShape* circleshapeB = dynamic_cast<p2CircleShape*>(colliderB.GetShape()))
						{
							if (p2RectShape* rectShapeA = dynamic_cast<p2RectShape*>(colliderA.GetShape()))
							{
								p2Mat22 mtv;
								float newAngle = bodyA->GetAngle() / 180 * M_PI;
								p2Vec2 u = bodyB->GetPosition() - (bodyA->GetPosition());
								u = u.Normalized() * (u.GetMagnitude() - circleshapeB->GetRadius());
								p2Vec2 x = p2Vec2(rectShapeA->GetSize().x, 0).Rotate(newAngle);
								p2Vec2 y = p2Vec2(0, rectShapeA->GetSize().y).Rotate(newAngle);
								p2Vec2 resultX = x * (p2Vec2::Dot(u, x) / p2Vec2::Dot(x, x));
								p2Vec2 resultY = y * (p2Vec2::Dot(u, y) / p2Vec2::Dot(y, y));
								mtv = p2Mat22(p2Vec2(0, 0), rectShapeA->GetSize());
								float resultXMagn = resultX.GetMagnitude();
								float resultYMagn = resultY.GetMagnitude();
								float uMagn = u.GetMagnitude();

								float projX = resultX.GetMagnitude() / x.GetMagnitude();
								float projY = resultY.GetMagnitude() / y.GetMagnitude();

								float kx = (p2Vec2::Dot(u, x) / p2Vec2::Dot(x, x));
								float ky = (p2Vec2::Dot(u, y) / p2Vec2::Dot(y, y));
								if ((kx < 1 && kx > -1) && (ky < 1 && ky > -1))
								{
									if (kx < 1)
									{
										if (mtv.rows[1].GetMagnitude() > (x * (1 - kx)).GetMagnitude())
										{
											mtv.rows[1] = (x * (1 - kx));
											mtv.rows[0] = u;
										}
									}
									if (kx > -1)
									{
										if (mtv.rows[1].GetMagnitude() > (x * (-1 - kx)).GetMagnitude())
										{
											mtv.rows[1] = (x * (-1 - kx));
											mtv.rows[0] = u + bodyA->GetPosition();
										}
									}
									if (ky < 1)
									{
										if (mtv.rows[1].GetMagnitude() > (y * (1 - ky)).GetMagnitude())
										{
											mtv.rows[1] = (y * (1 - ky));
											mtv.rows[0] = u + bodyA->GetPosition();
										}
									}
									if (ky > -1)
									{
										if (mtv.rows[1].GetMagnitude() > (y * (-1 - ky)).GetMagnitude())
										{
											mtv.rows[1] = (y * (-1 - ky));
											mtv.rows[0] = u + bodyA->GetPosition();
										}
									}
									m_Graphics2DManager->DrawVector(meter2pixel(mtv.rows[1]) * resize, meter2pixel(mtv.rows[0]) * resize, sf::Color::Red);
								}
								
								m_Graphics2DManager->DrawVector(meter2pixel(u) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Green);
								m_Graphics2DManager->DrawVector(meter2pixel(x) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Red);
								m_Graphics2DManager->DrawVector(meter2pixel(y) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Red);
								if (kx > 1 || kx < -1)
								{
									m_Graphics2DManager->DrawVector(meter2pixel(resultX) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Blue);
								}
								else
								{
									m_Graphics2DManager->DrawVector(meter2pixel(resultX) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Yellow);
								}
								if (ky > 1 || ky < -1)
								{
									m_Graphics2DManager->DrawVector(meter2pixel(resultY) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Blue);
								}
								else
								{
									m_Graphics2DManager->DrawVector(meter2pixel(resultY) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Yellow);
								}
								

								/*
								m_Graphics2DManager->DrawVector(meter2pixel(u) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Red);
								m_Graphics2DManager->DrawVector(meter2pixel(x) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Red);
								m_Graphics2DManager->DrawVector(meter2pixel(y) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Red);
								m_Graphics2DManager->DrawVector(meter2pixel(resultX) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Blue);
								m_Graphics2DManager->DrawVector(meter2pixel(resultY) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Blue);
								*/

								//return ((projXmax > 1 && projXmin > 1) || (projXmax < -1 && projXmin < -1) || (projYmax > 1 && projYmin > 1) || (projYmax < -1 && projYmin < -1));
							}
						}
					}
					else if (colliderB.GetColliderType() == p2ColliderType::BOX)
					{
						if (p2RectShape* rectShapeA = dynamic_cast<p2RectShape*>(colliderA.GetShape()))
						{
							if (p2RectShape* rectShapeB = dynamic_cast<p2RectShape*>(colliderB.GetShape()))
							{
								p2Mat22 mtv;
								float projXSup = 0;
								float projXInf = 0;
								float projYInf = 0;
								float projYSup = 0;
								float newAngle = bodyB->GetAngle() / 180 * M_PI;
								p2Vec2 x = p2Vec2(rectShapeB->GetSize().x, 0).Rotate(newAngle);
								p2Vec2 y = p2Vec2(0, rectShapeB->GetSize().y).Rotate(newAngle);
								mtv = p2Mat22(p2Vec2(0, 0), rectShapeB->GetSize());
								for (p2Vec2 cornerA : rectShapeA->GetCorner())
								{
									p2Vec2 u = bodyA->GetPosition() + cornerA - (bodyB->GetPosition());

									p2Vec2 resultX = x * (p2Vec2::Dot(u, x) / p2Vec2::Dot(x, x));
									p2Vec2 resultY = y * (p2Vec2::Dot(u, y) / p2Vec2::Dot(y, y));
									
									float kx = (p2Vec2::Dot(u, x) / p2Vec2::Dot(x, x));
									float ky = (p2Vec2::Dot(u, y) / p2Vec2::Dot(y, y));
									/*
									m_Graphics2DManager->DrawVector(meter2pixel(u) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Green);
									m_Graphics2DManager->DrawVector(meter2pixel(x) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Red);
									m_Graphics2DManager->DrawVector(meter2pixel(y) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Red);
									if (kx > 1 || kx < -1)
									{
										m_Graphics2DManager->DrawVector(meter2pixel(resultX) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Blue);
									}
									else
									{
										m_Graphics2DManager->DrawVector(meter2pixel(resultX) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Yellow);
									}
									if (ky > 1 || ky < -1)
									{
										m_Graphics2DManager->DrawVector(meter2pixel(resultY) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Blue);
									}
									else
									{
										m_Graphics2DManager->DrawVector(meter2pixel(resultY) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Yellow);
									}
									*/

									if (kx > 1)
									{
										projXSup++;
									}
									if (kx < -1)
									{
										projXInf++;
									}
									if (ky > 1)
									{
										projYSup++;
									}
									if (ky < -1)
									{
										projYInf++;
									}
									if ((kx < 1 && kx > -1) && (ky < 1 && ky > -1))
									{
										if (kx < 1)
										{
											if (mtv.rows[1].GetMagnitude() > (x*(1- kx)).GetMagnitude())
											{
												mtv.rows[1] = (x*(1 - kx));
												mtv.rows[0] = cornerA + bodyA->GetPosition();
											}
										}
										if (kx > -1)
										{
											if (mtv.rows[1].GetMagnitude() > (x*(-1-kx)).GetMagnitude())
											{
												mtv.rows[1] = (x*(-1 - kx));
												mtv.rows[0] = cornerA + bodyA->GetPosition();
											}
										}
										if (ky < 1)
										{
											if (mtv.rows[1].GetMagnitude() > (y*(1-ky)).GetMagnitude())
											{
												mtv.rows[1] = (y*(1 - ky));
												mtv.rows[0] = cornerA + bodyA->GetPosition();
											}
										}
										if (ky > -1)
										{
											if (mtv.rows[1].GetMagnitude() > (y*(-1-ky)).GetMagnitude())
											{
												mtv.rows[1] = (y*(-1 - ky));
												mtv.rows[0] = cornerA + bodyA->GetPosition();
											}
										}
									}
									
								}
								if ((projXSup == 4 || projXInf == 4 || projYSup == 4 || projYInf == 4))
								{
									continue;
								}

								projXSup = 0;
								projXInf = 0;
								projYInf = 0;
								projYSup = true;
								newAngle = bodyA->GetAngle() / 180 * M_PI;
								x = p2Vec2(rectShapeA->GetSize().x, 0).Rotate(newAngle);
								y = p2Vec2(0, rectShapeA->GetSize().y).Rotate(newAngle);
								for (p2Vec2 cornerB : rectShapeB->GetCorner())
								{
									p2Vec2 u = bodyB->GetPosition() + cornerB - (bodyA->GetPosition());

									p2Vec2 resultX = x * (p2Vec2::Dot(u, x) / p2Vec2::Dot(x, x));
									p2Vec2 resultY = y * (p2Vec2::Dot(u, y) / p2Vec2::Dot(y, y));

									float kx = (p2Vec2::Dot(u, x) / p2Vec2::Dot(x, x));
									float ky = (p2Vec2::Dot(u, y) / p2Vec2::Dot(y, y));
									/*
									m_Graphics2DManager->DrawVector(meter2pixel(u) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Green);
									m_Graphics2DManager->DrawVector(meter2pixel(x) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Red);
									m_Graphics2DManager->DrawVector(meter2pixel(y) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Red);
									
									if (kx > 1 || kx < -1)
									{
										m_Graphics2DManager->DrawVector(meter2pixel(resultX) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Blue);
									} else
									{
										m_Graphics2DManager->DrawVector(meter2pixel(resultX) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Yellow);
									}
									if (ky > 1 || ky < -1)
									{
										m_Graphics2DManager->DrawVector(meter2pixel(resultY) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Blue);
									}
									else
									{
										m_Graphics2DManager->DrawVector(meter2pixel(resultY) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Yellow);
									}
									*/

									if (kx > 1)
									{
										projXSup++;
									}
									if (kx < -1)
									{
										projXInf++;
									}
									if (ky > 1)
									{
										projYSup++;
									}
									if (ky < -1)
									{
										projYInf++;
									}
									if ((kx < 1 && kx > -1) && (ky < 1 && ky > -1))
									{
										if (kx < 1)
										{
											if (mtv.rows[1].GetMagnitude() > (x*(1-kx)).GetMagnitude())
											{
												mtv.rows[1] = (x*(1 - kx));
												mtv.rows[0] = cornerB + bodyB->GetPosition();
											}
										}
										if (kx > -1)
										{
											if (mtv.rows[1].GetMagnitude() > (x*(-1-kx)).GetMagnitude())
											{
												mtv.rows[1] = (x*(-1 - kx));
												mtv.rows[0] = cornerB + bodyB->GetPosition();
											}
										}
										if (ky < 1)
										{
											if (mtv.rows[1].GetMagnitude() > (y*(1-kx)).GetMagnitude())
											{
												mtv.rows[1] = (y*(1 - kx));
												mtv.rows[0] = cornerB + bodyB->GetPosition();
											}
										}
										if (ky > -1)
										{
											if (mtv.rows[1].GetMagnitude() > (y*(-1-kx)).GetMagnitude())
											{
												mtv.rows[1] = (y*(-1 - kx));
												mtv.rows[0] = cornerB + bodyB->GetPosition();
											}
										}
									}
								}

								if ((projXSup == 4 || projXInf == 4 || projYSup == 4 || projYInf == 4))
								{
									continue;
								}
								m_Graphics2DManager->DrawVector(meter2pixel(mtv.rows[1]) * resize, meter2pixel(mtv.rows[0]) * resize, sf::Color::Red);
							}
						}
					}
				}
			}
		}
	}

}
