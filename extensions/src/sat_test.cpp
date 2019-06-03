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
			//m_Graphics2DManager->DrawVector(meter2pixel(bodies[i]->GetLinearVelocity()), meter2pixel(bodies[i]->GetPosition()), sf::Color::White);
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
									p2Mat22 mtv = p2Mat22(bodyA->GetPosition() - (bodyA->GetPosition() - bodyB->GetPosition()).Normalized() * circleshapeA->GetRadius(), (bodyA->GetPosition() - bodyB->GetPosition()).Normalized() * ((circleshapeB->GetRadius() + circleshapeA->GetRadius() - (bodyA->GetPosition() - bodyB->GetPosition()).GetMagnitude())));
									m_Graphics2DManager->DrawVector(meter2pixel(mtv.rows[1]) * resize, meter2pixel(mtv.rows[0]) * resize, sf::Color::Red);
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
								p2Mat22 mtv = p2Mat22();
								float projSup = 0;
								float projInf = 0;
								float newAngle = bodyB->GetAngle() / 180 * M_PI;
								p2Vec2 axe;
								mtv = p2Mat22(p2Vec2(0, 0), rectShapeB->GetSize());
								float voroy = (p2Vec2::Dot(bodyB->GetPosition() - bodyA->GetPosition(), p2Vec2(rectShapeB->GetSize().x, 0).Rotate(newAngle)) / p2Vec2::Dot(p2Vec2(rectShapeB->GetSize().x, 0).Rotate(newAngle), p2Vec2(rectShapeB->GetSize().x, 0).Rotate(newAngle)));
								float vorox = (p2Vec2::Dot(bodyB->GetPosition() - bodyA->GetPosition(), p2Vec2(0, rectShapeB->GetSize().y).Rotate(newAngle)) / p2Vec2::Dot(p2Vec2(0, rectShapeB->GetSize().y).Rotate(newAngle), p2Vec2(0, rectShapeB->GetSize().y).Rotate(newAngle)));
								if ((vorox >= -1 && vorox <= 1) && (voroy >= -1 && voroy <= 1))
								{
									axe = p2Vec2();
								}
								else if ((vorox > -1 && vorox < 1))
								{
									axe = p2Vec2(rectShapeB->GetSize().x, 0).Rotate(newAngle).Normalized() * circleshapeA->GetRadius();
								}
								else if ((voroy > -1 && voroy < 1))
								{
									axe = p2Vec2(0, rectShapeB->GetSize().y).Rotate(newAngle).Normalized() * circleshapeA->GetRadius();
								}
								else
								{
									p2Vec2 closestCornerB;
									float minDist = p2Vec2::Distance(bodyB->GetPosition(), bodyA->GetPosition()) + rectShapeB->GetSize().GetMagnitude();
									for (p2Vec2 cornerB : rectShapeB->GetCorner())
									{
										if (p2Vec2::Distance(bodyB->GetPosition() + cornerB, bodyA->GetPosition()) <= minDist)
										{
											closestCornerB = cornerB;
											minDist = p2Vec2::Distance(bodyB->GetPosition() + cornerB, bodyA->GetPosition());
										}
									}
									axe = (bodyB->GetPosition() + closestCornerB - bodyA->GetPosition()).Normalized() * circleshapeA->GetRadius();
								}
								for (p2Vec2 cornerB : rectShapeB->GetCorner())
								{
									p2Vec2 u = bodyB->GetPosition() + cornerB - (bodyA->GetPosition());

									float k = (p2Vec2::Dot(u, axe) / p2Vec2::Dot(axe, axe));

									p2Vec2 result = axe * k;

									if (k > 1)
									{
										projSup++;
									}
									if (k < -1)
									{
										projInf++;
									}
									/*
									m_Graphics2DManager->DrawVector(meter2pixel(u) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Green);
									m_Graphics2DManager->DrawVector(meter2pixel(axe) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Red);
									if (k > 1 || k < -1)
									{
										m_Graphics2DManager->DrawVector(meter2pixel(result) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Blue);
									}
									else
									{
										m_Graphics2DManager->DrawVector(meter2pixel(result) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Yellow);
									}
									*/
									if (k < 1 && k > -1)
									{
										if (mtv.rows[1].GetMagnitude() > (axe * -(1 - k)).GetMagnitude())
										{
											mtv.rows[1] = (axe * -(1 - k));
											mtv.rows[0] = axe.Normalized() * circleshapeA->GetRadius() + bodyA->GetPosition();
										}
										if (mtv.rows[1].GetMagnitude() > (axe * (1 + k)).GetMagnitude())
										{
											mtv.rows[1] = (axe * (1 + k));
											mtv.rows[0] = axe.Normalized() * -circleshapeA->GetRadius() + bodyA->GetPosition();
										}
									}
								}

								if (projSup == 4 || projInf == 4)
								{
									continue;
								}
								m_Graphics2DManager->DrawVector(meter2pixel(mtv.rows[1]) * resize, meter2pixel(mtv.rows[0]) * resize, sf::Color::Black);
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
								p2Mat22 mtv = p2Mat22();
								float projSup = 0;
								float projInf = 0;
								float newAngle = bodyA->GetAngle() / 180 * M_PI;
								p2Vec2 axe;
								mtv = p2Mat22(p2Vec2(0, 0), rectShapeA->GetSize());
								float voroy = (p2Vec2::Dot(bodyA->GetPosition() - bodyB->GetPosition(), p2Vec2(rectShapeA->GetSize().x, 0).Rotate(newAngle)) / p2Vec2::Dot(p2Vec2(rectShapeA->GetSize().x, 0).Rotate(newAngle), p2Vec2(rectShapeA->GetSize().x, 0).Rotate(newAngle)));
								float vorox = (p2Vec2::Dot(bodyA->GetPosition() - bodyB->GetPosition(), p2Vec2(0, rectShapeA->GetSize().y).Rotate(newAngle)) / p2Vec2::Dot(p2Vec2(0, rectShapeA->GetSize().y).Rotate(newAngle), p2Vec2(0, rectShapeA->GetSize().y).Rotate(newAngle)));
								if ((vorox >= -1 && vorox <= 1) && (voroy >= -1 && voroy <= 1))
								{
									axe = p2Vec2();
								}
								else if ((vorox > -1 && vorox < 1))
								{
									axe = p2Vec2(rectShapeA->GetSize().x, 0).Rotate(newAngle).Normalized() * circleshapeB->GetRadius();
								}
								else if ((voroy > -1 && voroy < 1))
								{
									axe = p2Vec2(0, rectShapeA->GetSize().y).Rotate(newAngle).Normalized() * circleshapeB->GetRadius();
								}
								else
								{
									p2Vec2 closestCornerA;
									float minDist = p2Vec2::Distance(bodyA->GetPosition(), bodyB->GetPosition()) + rectShapeA->GetSize().GetMagnitude();
									for (p2Vec2 cornerA : rectShapeA->GetCorner())
									{
										if (p2Vec2::Distance(bodyA->GetPosition() + cornerA, bodyB->GetPosition()) <= minDist)
										{
											closestCornerA = cornerA;
											minDist = p2Vec2::Distance(bodyA->GetPosition() + cornerA, bodyB->GetPosition());
										}
									}
									axe = (bodyA->GetPosition() + closestCornerA - bodyB->GetPosition()).Normalized() * circleshapeB->GetRadius();
								}
								for (p2Vec2 cornerA : rectShapeA->GetCorner())
								{
									p2Vec2 u = bodyA->GetPosition() + cornerA - (bodyB->GetPosition());

									float k = (p2Vec2::Dot(u, axe) / p2Vec2::Dot(axe, axe));

									p2Vec2 result = axe * k;

									if (k > 1)
									{
										projSup++;
									}
									if (k < -1)
									{
										projInf++;
									}
									/*
									m_Graphics2DManager->DrawVector(meter2pixel(u) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Green);
									m_Graphics2DManager->DrawVector(meter2pixel(axe) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Red);
									m_Graphics2DManager->DrawVector(meter2pixel(bodyA->GetPosition()- (bodyB->GetPosition())) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::White);
									if (k > 1 || k < -1)
									{
										m_Graphics2DManager->DrawVector(meter2pixel(result) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Blue);
									}
									else
									{
										m_Graphics2DManager->DrawVector(meter2pixel(result) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Yellow);
									}
									*/
									if (k < 1 && k > -1)
									{
										if (mtv.rows[1].GetMagnitude() > (axe * -(1 - k)).GetMagnitude())
										{
											mtv.rows[1] = (axe * -(1 - k));
											mtv.rows[0] = axe.Normalized() * circleshapeB->GetRadius() + bodyB->GetPosition();
										}
										if (mtv.rows[1].GetMagnitude() > (axe * (1 + k)).GetMagnitude())
										{
											mtv.rows[1] = (axe * (1 + k));
											mtv.rows[0] = axe.Normalized() * -circleshapeB->GetRadius() + bodyB->GetPosition();
										}
									}
								}

								if (projSup == 4 || projInf == 4)
								{
									continue;
								}
								m_Graphics2DManager->DrawVector(meter2pixel(mtv.rows[1]) * resize, meter2pixel(mtv.rows[0]) * resize, sf::Color::Black);
							}
						}
					}
					else if (colliderB.GetColliderType() == p2ColliderType::BOX)
					{
						if (p2RectShape* rectShapeA = dynamic_cast<p2RectShape*>(colliderA.GetShape()))
						{
							if (p2RectShape* rectShapeB = dynamic_cast<p2RectShape*>(colliderB.GetShape()))
							{
								float newAngle = bodyB->GetAngle() / 180 * M_PI;
								float voroy = (p2Vec2::Dot(bodyA->GetPosition() - bodyB->GetPosition(), p2Vec2(rectShapeB->GetSize().x, 0).Rotate(newAngle)) / p2Vec2::Dot(p2Vec2(rectShapeB->GetSize().x, 0).Rotate(newAngle), p2Vec2(rectShapeB->GetSize().x, 0).Rotate(newAngle)));
								float vorox = (p2Vec2::Dot(bodyA->GetPosition() - bodyB->GetPosition(), p2Vec2(0, rectShapeB->GetSize().y).Rotate(newAngle)) / p2Vec2::Dot(p2Vec2(0, rectShapeB->GetSize().y).Rotate(newAngle), p2Vec2(0, rectShapeB->GetSize().y).Rotate(newAngle)));
								p2Mat22 mtvAX;
								p2Mat22 mtvAY;
								float projXSup = 0;
								float projXInf = 0;
								float projYInf = 0;
								float projYSup = 0;
								p2Vec2 x = p2Vec2(rectShapeB->GetSize().x, 0).Rotate(newAngle);
								p2Vec2 y = p2Vec2(0, rectShapeB->GetSize().y).Rotate(newAngle);
								mtvAX = p2Mat22(p2Vec2(0, 0), rectShapeA->GetSize() * 2);
								mtvAY = p2Mat22(p2Vec2(0, 0), rectShapeA->GetSize() * 2);
								for (p2Vec2 cornerA : rectShapeA->GetCorner())
								{
									p2Vec2 u = bodyA->GetPosition() + cornerA - (bodyB->GetPosition());

									p2Vec2 resultX = x * (p2Vec2::Dot(u, x) / p2Vec2::Dot(x, x));
									p2Vec2 resultY = y * (p2Vec2::Dot(u, y) / p2Vec2::Dot(y, y));

									float kx = (p2Vec2::Dot(u, x) / p2Vec2::Dot(x, x));
									float ky = (p2Vec2::Dot(u, y) / p2Vec2::Dot(y, y));
									m_Graphics2DManager->DrawVector(meter2pixel(bodyA->GetPosition() - (bodyB->GetPosition())) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Red);
									/*
									m_Graphics2DManager->DrawVector(meter2pixel(u) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Green);
									m_Graphics2DManager->DrawVector(meter2pixel(x) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Red);
									m_Graphics2DManager->DrawVector(meter2pixel(y) * resize, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Red);
									m_Graphics2DManager->DrawVector(meter2pixel(axe) * resize * 10, meter2pixel(bodyB->GetPosition()) * resize, sf::Color::Green);
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
									if ((kx <= 1 && kx >= -1) && (ky <= 1 && ky >= -1))
									{
										p2Mat22 tmpMtvAX;
										if (mtvAX.rows[1].GetMagnitude() >= (x * -(1 + kx)).GetMagnitude())
										{
											if (vorox < -1 || vorox > 1)
											{
												tmpMtvAX.rows[1] = (x * -(1 + kx));
												tmpMtvAX.rows[0] = cornerA + bodyA->GetPosition();
											}
											else
											{
												tmpMtvAX.rows[0] = bodyA->GetPosition() + p2Vec2(rectShapeA->GetSize().x, 0);
												tmpMtvAX.rows[1] = (x * -(1 + kx));
											}
										}
										if (mtvAX.rows[1].GetMagnitude() >= (x * (1 - kx)).GetMagnitude())
										{
											if (vorox < -1 || vorox > 1)
											{
												tmpMtvAX.rows[1] = (x * (1 - kx));
												tmpMtvAX.rows[0] = cornerA + bodyA->GetPosition();
											}
											else
											{
												tmpMtvAX.rows[0] = bodyA->GetPosition() - p2Vec2(rectShapeA->GetSize().x, 0);
												tmpMtvAX.rows[1] = (x * (1 - kx));
											}
										}
										p2Mat22 tmpMtvAY;

										if (mtvAY.rows[1].GetMagnitude() >= (y * -(1 + ky)).GetMagnitude())
										{
											if (voroy < -1 || voroy > 1)
											{
												tmpMtvAY.rows[1] = (y * -(1 + ky));
												tmpMtvAY.rows[0] = cornerA + bodyA->GetPosition();
											}
											else
											{
												tmpMtvAY.rows[0] = bodyA->GetPosition() + p2Vec2(0, rectShapeA->GetSize().y);
												tmpMtvAY.rows[1] = (y * -(1 + ky));
											}
										}
										if (mtvAY.rows[1].GetMagnitude() >= (y * (1 - ky)).GetMagnitude())
										{
											if (voroy < -1 || voroy > 1)
											{
												tmpMtvAY.rows[1] = (y * (1 - ky));
												tmpMtvAY.rows[0] = cornerA + bodyA->GetPosition();
											}
											else
											{
												tmpMtvAY.rows[0] = bodyA->GetPosition() - p2Vec2(0, rectShapeA->GetSize().y);
												tmpMtvAY.rows[1] = (y * (1 - ky));
											}
										}
										if (tmpMtvAX != p2Mat22())
										{
											mtvAX = tmpMtvAX;
										}
										if (tmpMtvAY != p2Mat22())
										{
											mtvAY = tmpMtvAY;
										}
									}
								}
								if ((projXSup == 4 || projXInf == 4 || projYSup == 4 || projYInf == 4))
								{
									continue;
								}

								if (projXInf + projXSup == 0)
								{
									mtvAX = p2Mat22(p2Vec2(0, 0), rectShapeA->GetSize() * 2);
								}
								if (projYInf + projYSup == 0)
								{
									mtvAY = p2Mat22(p2Vec2(0, 0), rectShapeA->GetSize() * 2);
								}

								p2Mat22 mtvBX;
								p2Mat22 mtvBY;

								projXSup = 0;
								projXInf = 0;
								projYInf = 0;
								projYSup = true;
								newAngle = bodyA->GetAngle() / 180 * M_PI;
								x = p2Vec2(rectShapeA->GetSize().x, 0).Rotate(newAngle);
								y = p2Vec2(0, rectShapeA->GetSize().y).Rotate(newAngle);
								mtvBX = p2Mat22(p2Vec2(0, 0), rectShapeB->GetSize() * 2);
								mtvBY = p2Mat22(p2Vec2(0, 0), rectShapeB->GetSize() * 2);
								voroy = (p2Vec2::Dot(bodyB->GetPosition() - bodyA->GetPosition(), p2Vec2(rectShapeA->GetSize().x, 0).Rotate(newAngle)) / p2Vec2::Dot(p2Vec2(rectShapeA->GetSize().x, 0).Rotate(newAngle), p2Vec2(rectShapeA->GetSize().x, 0).Rotate(newAngle)));
								vorox = (p2Vec2::Dot(bodyB->GetPosition() - bodyA->GetPosition(), p2Vec2(0, rectShapeA->GetSize().y).Rotate(newAngle)) / p2Vec2::Dot(p2Vec2(0, rectShapeA->GetSize().y).Rotate(newAngle), p2Vec2(0, rectShapeA->GetSize().y).Rotate(newAngle)));
								for (p2Vec2 cornerB : rectShapeB->GetCorner())
								{
									p2Vec2 u = bodyB->GetPosition() + cornerB - (bodyA->GetPosition());

									p2Vec2 resultX = x * (p2Vec2::Dot(u, x) / p2Vec2::Dot(x, x));
									p2Vec2 resultY = y * (p2Vec2::Dot(u, y) / p2Vec2::Dot(y, y));

									float kx = (p2Vec2::Dot(u, x) / p2Vec2::Dot(x, x));
									float ky = (p2Vec2::Dot(u, y) / p2Vec2::Dot(y, y));

									m_Graphics2DManager->DrawVector(meter2pixel(bodyB->GetPosition() - (bodyA->GetPosition())) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Red);
									/*
									m_Graphics2DManager->DrawVector(meter2pixel(u) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Green);
									m_Graphics2DManager->DrawVector(meter2pixel(x) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Red);
									m_Graphics2DManager->DrawVector(meter2pixel(y) * resize, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Red);
									m_Graphics2DManager->DrawVector(meter2pixel(axe) * resize * 10, meter2pixel(bodyA->GetPosition()) * resize, sf::Color::Green);

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
									if ((kx <= 1 && kx >= -1) && (ky <= 1 && ky >= -1))
									{
										p2Mat22 tmpMtvBX;
										if (mtvBX.rows[1].GetMagnitude() >= (x * -(1 + kx)).GetMagnitude())
										{
											if (vorox < -1 || vorox > 1)
											{
												tmpMtvBX.rows[1] = (x * -(1 + kx));
												tmpMtvBX.rows[0] = cornerB + bodyB->GetPosition();
											}
											else
											{
												tmpMtvBX.rows[0] = bodyB->GetPosition() + p2Vec2(rectShapeB->GetSize().x, 0);
												tmpMtvBX.rows[1] = (x * -(1 + kx));
											}
										}
										if (mtvBX.rows[1].GetMagnitude() >= (x * (1 - kx)).GetMagnitude())
										{
											if (vorox < -1 || vorox > 1)
											{
												tmpMtvBX.rows[1] = (x * (1 - kx));
												tmpMtvBX.rows[0] = cornerB + bodyB->GetPosition();
											}
											else
											{
												tmpMtvBX.rows[0] = bodyB->GetPosition() - p2Vec2(rectShapeB->GetSize().x, 0);
												tmpMtvBX.rows[1] = (x * (1 - kx));
											}
										}
										p2Mat22 tmpMtvBY;

										if (mtvBY.rows[1].GetMagnitude() >= (y * -(1 + ky)).GetMagnitude())
										{
											if (voroy < -1 || voroy > 1)
											{
												tmpMtvBY.rows[1] = (y * -(1 + ky));
												tmpMtvBY.rows[0] = cornerB + bodyB->GetPosition();
											}
											else
											{
												tmpMtvBY.rows[0] = bodyB->GetPosition() + p2Vec2(0, rectShapeB->GetSize().y);
												tmpMtvBY.rows[1] = (y * -(1 + ky));
											}
										}
										if (mtvBY.rows[1].GetMagnitude() >= (y * (1 - ky)).GetMagnitude())
										{
											if (voroy < -1 || voroy > 1)
											{
												tmpMtvBY.rows[1] = (y * (1 - ky));
												tmpMtvBY.rows[0] = cornerB + bodyB->GetPosition();
											}
											else
											{
												tmpMtvBY.rows[0] = bodyB->GetPosition() - p2Vec2(0,rectShapeB->GetSize().y);
												tmpMtvBY.rows[1] = (y * (1 - ky));
											}
										}
										if (tmpMtvBX != p2Mat22())
										{
											mtvBX = tmpMtvBX;
										}
										if (tmpMtvBY != p2Mat22())
										{
											mtvBY = tmpMtvBY;
										}
									}
								}

								if ((projXSup == 4 || projXInf == 4 || projYSup == 4 || projYInf == 4))
								{
									continue;
								}
								if (projXInf + projXSup == 0)
								{
									mtvBX = p2Mat22(p2Vec2(0, 0), rectShapeB->GetSize() * 2);
								}
								if (projYInf + projYSup == 0)
								{
									mtvBY = p2Mat22(p2Vec2(0, 0), rectShapeB->GetSize() * 2);
								}

								if (mtvAX.rows[1].GetMagnitude() > mtvAY.rows[1].GetMagnitude())
								{
									mtvAX = mtvAY;
								}
								if (mtvBX.rows[1].GetMagnitude() > mtvBY.rows[1].GetMagnitude())
								{
									mtvBX = mtvBY;
								}
								if (mtvAX.rows[1].GetMagnitude() > mtvBX.rows[1].GetMagnitude())
								{
									m_Graphics2DManager->DrawVector(meter2pixel(mtvBX.rows[1]) * resize, meter2pixel(mtvBX.rows[0]) * resize, sf::Color::White);
								}
								else
								{
									m_Graphics2DManager->DrawVector(meter2pixel(mtvAX.rows[1]) * resize, meter2pixel(mtvAX.rows[0]) * resize, sf::Color::White);
								}
							}
						}
					}
				}
			}
		}
	}
}
