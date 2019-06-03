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
#include "SFML/Window/Keyboard.hpp"

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

void p2ContactManager::RemoveContact(int contactIndex)
{
	m_Contacts.erase(m_Contacts.begin() + contactIndex);
	m_ContactIndex--;
	m_Contacts.resize(MAX_CONTACT_LEN);
}

void p2ContactManager::CheckContact(std::vector<p2Body>& bodies)
{
	p2AABB rootAABB;
	for (p2Body body : bodies)
	{
		if (rootAABB.bottomLeft.y > body.GetAABB().bottomLeft.y)
		{
			rootAABB.bottomLeft.y = body.GetAABB().bottomLeft.y;
		}
		if (rootAABB.bottomLeft.x > body.GetAABB().bottomLeft.x)
		{
			rootAABB.bottomLeft.x = body.GetAABB().bottomLeft.x;
		}
		if (rootAABB.topRight.y < body.GetAABB().topRight.y)
		{
			rootAABB.topRight.y = body.GetAABB().topRight.y;
		}
		if (rootAABB.topRight.x < body.GetAABB().topRight.x)
		{
			rootAABB.topRight.x = body.GetAABB().topRight.x;
		}
	}
	m_RootQuadTree = p2QuadTree(0, rootAABB);
	rmt_ScopedCPUSample(Insert, 0);
	for (p2Body& body : bodies)
	{
		if (body.IsInstantiate() && (body.GetAABB().topRight != p2Vec2()))
		{
			m_RootQuadTree.Insert(&body);
		}
	}
	rmt_ScopedCPUSample(Retrieve, 0);
	for (int i = 0; i < m_ContactIndex; i++)
	{
		m_Contacts[i].updated = false;
	}
	m_RootQuadTree.Retrieve(this);
	rmt_ScopedCPUSample(EndCheckContact, 0);
	for (int i = 0; i < m_ContactIndex; i++)
	{
		if (!m_Contacts[i].updated)
		{
			m_ContactListener->EndContact(&m_Contacts[i]);
			RemoveContact(i);
		}
	}
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
	rmt_ScopedCPUSample(EndContact, 0);
	int contactIndex = -1;
	if (CheckAABBContact(body1, body2))
	{
		p2Mat22 mtv = CheckSATContact(body1, body2);
		if (mtv != p2Mat22(p2Vec2(0, 0), p2Vec2(0, 0)))
		{
			contactIndex = ContainContact(body1, body2);
			if (contactIndex == -1)
			{
				p2Contact* contact = CreateContact(&body1->GetCollider()->at(0), &body2->GetCollider()->at(0));
				m_ContactListener->BeginContact(contact);
			} else
			{
				m_Contacts[contactIndex].updated = true;
			}
			
			rmt_ScopedCPUSample(CorrectContact, 0);
			
			p2Vec2 newBody1Velocity;
			p2Vec2 newBody2Velocity;
			float coeffRestitution = (body1->GetCollider()->at(0).GetRestitution() + body1->GetCollider()->at(0).GetRestitution()) / 2;
			p2Vec2 ua = body1->GetLinearVelocity();
			p2Vec2 ub = body2->GetLinearVelocity();
			float ma = body1->GetMass();
			float mb = body2->GetMass();
			if ((body1->GetPosition() - mtv.rows[0]).GetMagnitude() < (body1->GetPosition() - mtv.rows[0] + mtv.rows[1]).GetMagnitude())
			{
				if(body1->GetType() == p2BodyType::DYNAMIC)
				{
					body1->SetPosition(body1->GetPosition() + (mtv.rows[1]));
					//newBody1Velocity = (body1->GetLinearVelocity() - (mtv.rows[1].Normalized() * p2Vec2::Dot(body1->GetLinearVelocity(), mtv.rows[1].Normalized())) * 2);
					
					newBody1Velocity = (((ub - ua) * mb * coeffRestitution + ua * ma + ub * mb) / (ma + mb));
					float inertia = 0;
					if (p2CircleShape* circleshape = dynamic_cast<p2CircleShape*>(body1->GetCollider()->at(0).GetShape()))
					{
						inertia = 0;
					}
					else if (p2RectShape* rectshape = dynamic_cast<p2RectShape*>(body1->GetCollider()->at(0).GetShape()))
					{
						inertia = (rectshape->GetSize().x * pow(rectshape->GetSize().y, 3)) / 12;
					}
					p2Vec2 newVelocity = body1->GetLinearVelocity() * -1;
					body1->ApplyForceToCorner(inertia, mtv.rows[1] /*newVelocity * (p2Vec2::Dot(mtv.rows[1], newVelocity) / p2Vec2::Dot(newVelocity, newVelocity))*/, mtv.rows[0] - body1->GetPosition());
				}
				if (body2->GetType() == p2BodyType::DYNAMIC)
				{
					body2->SetPosition(body2->GetPosition() - (mtv.rows[1]));
					//newBody2Velocity = (body2->GetLinearVelocity() - (mtv.rows[1].Normalized() * p2Vec2::Dot(body2->GetLinearVelocity(), mtv.rows[1].Normalized())) * 2) * body2->GetCollider()->at(0).GetRestitution() + body1->GetLinearVelocity() * body1->GetCollider()->at(0).GetRestitution();

					newBody2Velocity = (((ua - ub) * ma * coeffRestitution + ua * ma + ub * mb) / (ma + mb));
					float inertia = 0;
					if (p2CircleShape* circleshape = dynamic_cast<p2CircleShape*>(body2->GetCollider()->at(0).GetShape()))
					{
						inertia = 0;
					}
					else if (p2RectShape* rectshape = dynamic_cast<p2RectShape*>(body2->GetCollider()->at(0).GetShape()))
					{
						inertia = (rectshape->GetSize().x * pow(rectshape->GetSize().y, 3)) / 12;
					}
					p2Vec2 newVelocity = body2->GetLinearVelocity() * -1;
					body2->ApplyForceToCorner(inertia, mtv.rows[1]/*newVelocity * (p2Vec2::Dot(mtv.rows[1], newVelocity) / p2Vec2::Dot(newVelocity, newVelocity))*/, mtv.rows[0] - body2->GetPosition());
				}
			}
			else
			{
				if (body1->GetType() == p2BodyType::DYNAMIC)
				{
					body1->SetPosition(body1->GetPosition() - (mtv.rows[1]));
					newBody1Velocity = (body1->GetLinearVelocity() - (mtv.rows[1].Normalized() * p2Vec2::Dot(body1->GetLinearVelocity(), mtv.rows[1].Normalized())) * 2) * body1->GetCollider()->at(0).GetRestitution() + body2->GetLinearVelocity() * body2->GetCollider()->at(0).GetRestitution();
					float inertia = 0;
					if (p2CircleShape* circleshape = dynamic_cast<p2CircleShape*>(body1->GetCollider()->at(0).GetShape()))
					{
						inertia = 0;
					}
					else if (p2RectShape* rectshape = dynamic_cast<p2RectShape*>(body1->GetCollider()->at(0).GetShape()))
					{
						inertia = (rectshape->GetSize().x * pow(rectshape->GetSize().y, 3)) / 12;
					}
					p2Vec2 newVelocity = body1->GetLinearVelocity() * -1;
					body1->ApplyForceToCorner(inertia, mtv.rows[1]/*newVelocity * (p2Vec2::Dot(mtv.rows[1], newVelocity) / p2Vec2::Dot(newVelocity, newVelocity))*/, mtv.rows[0] - body1->GetPosition());
				}
				if (body2->GetType() == p2BodyType::DYNAMIC)
				{
					body2->SetPosition(body2->GetPosition() + (mtv.rows[1]));
					newBody2Velocity = (body2->GetLinearVelocity() - (mtv.rows[1].Normalized() * p2Vec2::Dot(body2->GetLinearVelocity(), mtv.rows[1].Normalized())) * 2) * body2->GetCollider()->at(0).GetRestitution() + body1->GetLinearVelocity() * body1->GetCollider()->at(0).GetRestitution();
					float inertia = 0;
					if (p2CircleShape* circleshape = dynamic_cast<p2CircleShape*>(body2->GetCollider()->at(0).GetShape()))
					{
						inertia = 0;
					}
					else if (p2RectShape* rectshape = dynamic_cast<p2RectShape*>(body2->GetCollider()->at(0).GetShape()))
					{
						inertia = (rectshape->GetSize().x * pow(rectshape->GetSize().y, 3)) / 12;
					}
					p2Vec2 newVelocity = mtv.rows[0] - body2->GetPosition();
					body2->ApplyForceToCorner(inertia, mtv.rows[1] * p2Vec2::Dot(mtv.rows[0] - body2->GetPosition(), mtv.rows[1]) / p2Vec2::Dot(mtv.rows[1], mtv.rows[1]), mtv.rows[0] - body2->GetPosition());
				}
			}
			if (body1->GetType() == p2BodyType::DYNAMIC)
			{
				body1->SetLinearVelocity(newBody1Velocity);

			}
			if (body2->GetType() == p2BodyType::DYNAMIC)
			{
				body2->SetLinearVelocity(newBody2Velocity);
			}
	
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

int p2ContactManager::ContainContact(p2Body* bodyA, p2Body* bodyB)
{
	for (int i = 0; i < m_ContactIndex; i++)
	{
		if (m_Contacts[i].GetColliderA() == nullptr) continue;
		if (m_Contacts[i].CheckSameCollider(&bodyA->GetCollider()->at(0), &bodyB->GetCollider()->at(0)))
		{
			return i;
		}
	}
	return -1;
}

p2Mat22 p2ContactManager::CheckSATContact(p2Body* bodyA, p2Body* bodyB)
{
	rmt_ScopedCPUSample(SATContact, 0);

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
											tmpMtvBY.rows[0] = bodyB->GetPosition() - p2Vec2(0, rectShapeB->GetSize().y);
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
							if (mtvAX.rows[1].GetMagnitude() < mtvBX.rows[1].GetMagnitude())
							{
								return mtvAX;
							}
							else
							{
								return mtvBX;
							}
						}
					}
				}
			}
		}
	}
	return p2Mat22(p2Vec2(0, 0), p2Vec2(0, 0));
}
