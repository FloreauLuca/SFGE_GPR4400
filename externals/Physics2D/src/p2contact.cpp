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

#include <p2contact.h>
#include "p2body.h"
#include "p2quadtree.h"
#include "../../Remotery/Remotery.h"
#include <corecrt_math_defines.h>
#include "p2matrix.h"

void p2Contact::Init(p2Collider* colliderA, p2Collider* colliderB)
{
	m_ColliderA = colliderA;
	m_ColliderB = colliderB;
}

p2Collider* p2Contact::GetColliderA()
{
	return m_ColliderA;
}

p2Collider* p2Contact::GetColliderB()
{
	return m_ColliderB;
}

bool p2Contact::CheckSameCollider(p2Collider* colliderA, p2Collider* colliderB)
{
	return ((colliderA == GetColliderA() && colliderB == GetColliderB()) || (colliderA == GetColliderB() && colliderB == GetColliderA()));
}

void p2ContactManager::Init(p2ContactListener* contactListener)
{
	m_ContactListener = contactListener;
	m_Contacts.resize(MAX_CONTACT_LEN);
	m_ContactIndex = 0;
}

p2Contact* p2ContactManager::CreateContact(p2Collider* colliderA, p2Collider* colliderB)
{
	p2Contact& contact = m_Contacts[m_ContactIndex];
	contact.Init(colliderA, colliderB);
	m_ContactIndex++;
	return &contact;
}

void p2ContactManager::RemoveContact(p2Collider* colliderA, p2Collider* colliderB)
{
	for (int i = 0; i < m_ContactIndex; i++)
	{
		if (m_Contacts[i].CheckSameCollider(colliderA, colliderB))
		{
			m_Contacts.erase(m_Contacts.begin() + i);
			m_ContactIndex--;
			m_Contacts.resize(MAX_CONTACT_LEN);
		}
	}
}

void p2ContactManager::CheckContact(std::vector<p2Body>& bodies)
{
	p2AABB rootAABB;
	rootAABB.topRight = SCREEN_SIZE;
	rootAABB.bottomLeft = p2Vec2(0, 0);
	m_RootQuadTree = p2QuadTree(0, rootAABB);
	rmt_ScopedCPUSample(Insert, 0);
	for (p2Body& body : bodies)
	{
		if (body.IsInstantiate())
		{
			m_RootQuadTree.Insert(&body);
		}
	}
	rmt_ScopedCPUSample(Retrieve, 0);
	m_RootQuadTree.Retrieve(this);
}

void p2ContactManager::CheckContactInsideList(std::vector<p2Body*> bodies)
{
	rmt_ScopedCPUSample(CheckContactInsideVector, 0);
	for (int i = 0; i < bodies.size(); i++)
	{
		if (bodies[i] == nullptr)continue;
		if (bodies[i]->GetCollider()->empty())continue;

		for (int j = i + 1; j < bodies.size(); j++)
		{
			if (bodies[j] == nullptr)continue;

			if (bodies[j]->GetCollider()->empty())continue;
			CheckNewContactBetweenBodies(bodies[i], bodies[j]);
		}
	}
}

void p2ContactManager::CheckContactBetweenList(std::vector<p2Body*> bodies1, std::vector<p2Body*> bodies2)
{
	rmt_ScopedCPUSample(CheckContactBetweenList, 0);

	for (int i = 0; i < bodies1.size(); i++)
	{
		if (bodies1[i]->GetCollider()->empty())continue;
		for (int j = i; j < bodies2.size(); j++)
		{
			if (bodies2[j]->GetCollider()->empty())continue;
			if (bodies2[j] == bodies1[i])continue;
			CheckNewContactBetweenBodies(bodies1[i], bodies2[j]);
		}
	}
}


void p2ContactManager::CheckNewContactBetweenBodies(p2Body* body1, p2Body* body2)
{
	rmt_ScopedCPUSample(ContainContact, 0);

	if (!CheckAABBContact(body1, body2))
	{
		p2Contact* containedContact = ContainContact(body1, body2);
		if (containedContact)
		{
			m_ContactListener->EndContact(containedContact);
			RemoveContact(&body1->GetCollider()->at(0), &body2->GetCollider()->at(0));
		}
	}
	if (CheckAABBContact(body1, body2))
	{
		p2Mat22 mtv = CheckSATContact(body1, body2);
		if (mtv == p2Mat22(p2Vec2(0, 0), p2Vec2(0, 0)))
		{
			p2Contact* containedContact = ContainContact(body1, body2);
			if (containedContact)
			{
				m_ContactListener->EndContact(containedContact);
				RemoveContact(&body1->GetCollider()->at(0), &body2->GetCollider()->at(0));
			}
		}
		else
		{
			p2Contact* containedContact = ContainContact(body1, body2);
			if (!containedContact)
			{
				p2Contact* contact = CreateContact(&body1->GetCollider()->at(0), &body2->GetCollider()->at(0));
				m_ContactListener->BeginContact(contact);
			}
			
			
			/*
			p2Vec2 newBody1Velocity;
			p2Vec2 newBody2Velocity;
			
			if ((body1->GetPosition() - mtv.rows[0]).GetMagnitude() < (body1->GetPosition() - mtv.rows[0] + mtv.rows[1]).GetMagnitude())
			{
				if(body1->GetType() != p2BodyType::STATIC)
				{
					body1->SetPosition(body1->GetPosition() + (mtv.rows[1]));
					newBody1Velocity = (body1->GetLinearVelocity() - (mtv.rows[1].Normalized() * p2Vec2::Dot(body1->GetLinearVelocity(), mtv.rows[1].Normalized())) * 2) * body1->GetCollider()->at(0).GetRestitution() + body2->GetLinearVelocity() * body2->GetCollider()->at(0).GetRestitution();
				}
				if (body2->GetType() != p2BodyType::STATIC)
				{
					body2->SetPosition(body2->GetPosition() - (mtv.rows[1]));
					newBody2Velocity = (body2->GetLinearVelocity() - (mtv.rows[1].Normalized() * p2Vec2::Dot(body2->GetLinearVelocity(), mtv.rows[1].Normalized())) * 2) * body2->GetCollider()->at(0).GetRestitution() + body1->GetLinearVelocity() * body1->GetCollider()->at(0).GetRestitution();
				}
				//body1->ApplyForceToCenter(mtv.rows[1]);
				//body2->ApplyForceToCenter(p2Vec2() - mtv.rows[1]);
			}
			else
			{
				if (body1->GetType() != p2BodyType::STATIC)
				{
					body1->SetPosition(body1->GetPosition() - (mtv.rows[1]));
					newBody1Velocity = (body1->GetLinearVelocity() - (mtv.rows[1].Normalized() * p2Vec2::Dot(body1->GetLinearVelocity(), mtv.rows[1].Normalized())) * 2) * body1->GetCollider()->at(0).GetRestitution() + body2->GetLinearVelocity() * body2->GetCollider()->at(0).GetRestitution();
				}
				if (body2->GetType() != p2BodyType::STATIC)
				{
					body2->SetPosition(body2->GetPosition() + (mtv.rows[1]));
					newBody2Velocity = (body2->GetLinearVelocity() - (mtv.rows[1].Normalized() * p2Vec2::Dot(body2->GetLinearVelocity(), mtv.rows[1].Normalized())) * 2) * body2->GetCollider()->at(0).GetRestitution() + body1->GetLinearVelocity() * body1->GetCollider()->at(0).GetRestitution();
				}
				//body2->ApplyForceToCenter(mtv.rows[1]);
				//body1->ApplyForceToCenter(p2Vec2() - mtv.rows[1]);
			}

			body1->SetLinearVelocity(newBody1Velocity);
			body2->SetLinearVelocity(newBody2Velocity);
			*/
			
		}
	}
}


p2QuadTree* p2ContactManager::GetQuadtree()
{
	return &m_RootQuadTree;
}

bool p2ContactManager::CheckAABBContact(p2Body* bodyA, p2Body* bodyB)
{
	p2AABB aabbA = bodyA->GetAABB();
	p2AABB aabbB = bodyB->GetAABB();
	if (aabbA.ContainsPoint(aabbB.topRight) || aabbA.ContainsPoint(aabbB.bottomLeft) || aabbA.ContainsPoint(p2Vec2(aabbB.bottomLeft.x, aabbB.topRight.y)) || aabbA.ContainsPoint(p2Vec2(aabbB.topRight.x, aabbB.bottomLeft.y)))
	{
		return true;
	}
	if (aabbB.ContainsPoint(aabbA.topRight) || aabbB.ContainsPoint(aabbA.bottomLeft) || aabbB.ContainsPoint(p2Vec2(aabbA.bottomLeft.x, aabbA.topRight.y)) || aabbB.ContainsPoint(p2Vec2(aabbA.topRight.x, aabbA.bottomLeft.y)))
	{
		return true;
	}
	return false;
}

p2Contact* p2ContactManager::ContainContact(p2Body* bodyA, p2Body* bodyB)
{
	for (int i = 0; i < m_ContactIndex; i++)
	{
		if (m_Contacts[i].GetColliderA() == nullptr) continue;
		if (m_Contacts[i].CheckSameCollider(&bodyA->GetCollider()->at(0), &bodyB->GetCollider()->at(0)))
		{
			return &m_Contacts[i];
		}
	}
	return nullptr;
}

p2Mat22 p2ContactManager::CheckSATContact(p2Body* bodyA, p2Body* bodyB)
{
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
								p2Mat22 mtv = p2Mat22(bodyA->GetPosition() - (bodyA->GetPosition() - bodyB->GetPosition()).Normalized()*circleshapeA->GetRadius(), (bodyA->GetPosition() - bodyB->GetPosition()).Normalized() * ((circleshapeB->GetRadius() + circleshapeA->GetRadius() - (bodyA->GetPosition() - bodyB->GetPosition()).GetMagnitude())));
								return mtv;
							}
						}
					}
					return p2Mat22(p2Vec2(0, 0), p2Vec2(0, 0));
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
							else
								if ((vorox > -1 && vorox < 1))
								{
									axe = p2Vec2(rectShapeB->GetSize().x, 0).Rotate(newAngle).Normalized()*circleshapeA->GetRadius();
								}
								else if ((voroy > -1 && voroy < 1))
								{
									axe = p2Vec2(0, rectShapeB->GetSize().y).Rotate(newAngle).Normalized()*circleshapeA->GetRadius();
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
									axe = (bodyB->GetPosition() + closestCornerB - bodyA->GetPosition()).Normalized()*circleshapeA->GetRadius();
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
								if (k <= 1 && k >= -1)
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
								return p2Mat22();
							}
							return  mtv;
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
								axe = p2Vec2(rectShapeA->GetSize().x, 0).Rotate(newAngle).Normalized()*circleshapeB->GetRadius();
							}
							else if ((voroy > -1 && voroy < 1))
							{
								axe = p2Vec2(0, rectShapeA->GetSize().y).Rotate(newAngle).Normalized()*circleshapeB->GetRadius();
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
								axe = (bodyA->GetPosition() + closestCornerA - bodyB->GetPosition()).Normalized()*circleshapeB->GetRadius();
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
								if (k <= 1 && k >= -1)
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
								return p2Mat22();
							}
							return  mtv;
						}
					}
				}
				else if (colliderB.GetColliderType() == p2ColliderType::BOX)
				{
					if (p2RectShape* rectShapeA = dynamic_cast<p2RectShape*>(colliderA.GetShape()))
					{
						if (p2RectShape* rectShapeB = dynamic_cast<p2RectShape*>(colliderB.GetShape()))
						{
							p2Mat22 mtv = p2Mat22();
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

								float kx = (p2Vec2::Dot(u, x) / p2Vec2::Dot(x, x));
								float ky = (p2Vec2::Dot(u, y) / p2Vec2::Dot(y, y));

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
										if (mtv.rows[1].GetMagnitude() > (x * (1 - kx)).GetMagnitude())
										{
											mtv.rows[1] = (x * (1 - kx));
											mtv.rows[0] = cornerA + bodyA->GetPosition();
										}
									}
									if (kx > -1)
									{
										if (mtv.rows[1].GetMagnitude() > (x * (-1 - kx)).GetMagnitude())
										{
											mtv.rows[1] = (x * (-1 - kx));
											mtv.rows[0] = cornerA + bodyA->GetPosition();
										}
									}
									if (ky < 1)
									{
										if (mtv.rows[1].GetMagnitude() > (y * (1 - ky)).GetMagnitude())
										{
											mtv.rows[1] = (y * (1 - ky));
											mtv.rows[0] = cornerA + bodyA->GetPosition();
										}
									}
									if (ky > -1)
									{
										if (mtv.rows[1].GetMagnitude() > (y * (-1 - ky)).GetMagnitude())
										{
											mtv.rows[1] = (y * (-1 - ky));
											mtv.rows[0] = cornerA + bodyA->GetPosition();
										}
									}
								}
							}

							if (projXSup == 4 || projXInf == 4 || projYSup == 4 || projYInf == 4)
							{
								return p2Mat22(p2Vec2(0, 0), p2Vec2(0, 0));
							}

							projXSup = 0;
							projXInf = 0;
							projYInf = 0;
							projYSup = 0;
							newAngle = bodyA->GetAngle() / 180 * M_PI;
							x = p2Vec2(rectShapeA->GetSize().x, 0).Rotate(newAngle);
							y = p2Vec2(0, rectShapeA->GetSize().y).Rotate(newAngle);
							for (p2Vec2 cornerB : rectShapeB->GetCorner())
							{
								p2Vec2 u = bodyB->GetPosition() + cornerB - (bodyA->GetPosition());

								float kx = (p2Vec2::Dot(u, x) / p2Vec2::Dot(x, x));
								float ky = (p2Vec2::Dot(u, y) / p2Vec2::Dot(y, y));

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
										if (mtv.rows[1].GetMagnitude() > (x * (1 - kx)).GetMagnitude())
										{
											mtv.rows[1] = (x * (1 - kx));
											mtv.rows[0] = cornerB + bodyB->GetPosition();
										}
									}
									if (kx > -1)
									{
										if (mtv.rows[1].GetMagnitude() > (x * (-1 - kx)).GetMagnitude())
										{
											mtv.rows[1] = (x * (-1 - kx));
											mtv.rows[0] = cornerB + bodyB->GetPosition();
										}
									}
									if (ky < 1)
									{
										if (mtv.rows[1].GetMagnitude() > (y * (1 - kx)).GetMagnitude())
										{
											mtv.rows[1] = (y * (1 - kx));
											mtv.rows[0] = cornerB + bodyB->GetPosition();
										}
									}
									if (ky > -1)
									{
										if (mtv.rows[1].GetMagnitude() > (y * (-1 - kx)).GetMagnitude())
										{
											mtv.rows[1] = (y * (-1 - kx));
											mtv.rows[0] = cornerB + bodyB->GetPosition();
										}
									}
								}
							}

							if (projXSup == 4 || projXInf == 4 || projYSup == 4 || projYInf == 4)
							{
								return p2Mat22(p2Vec2(0, 0), p2Vec2(0, 0));
							}
							return mtv;
						}
					}
				}
			}
		}
	}
	return p2Mat22(p2Vec2(0, 0), p2Vec2(0, 0));
}
