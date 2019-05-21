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
	return ((colliderA == GetColliderA() && colliderB == GetColliderB()) || (colliderA == GetColliderB() && colliderB ==
		GetColliderA()));
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
		if(bodies[i]==nullptr)continue;
		if (bodies[i]->GetCollider()->empty())continue;

		for (int j = i + 1; j < bodies.size(); j++)
		{
			if (bodies[j] == nullptr)continue;

			if (bodies[j]->GetCollider()->empty())continue;
			CheckContactBetweenBodies(bodies[i], bodies[j]);
		}
	}
}

void p2ContactManager::CheckContactBetweenList(std::vector<p2Body*> bodies1, std::vector<p2Body*> bodies2)
{
	rmt_ScopedCPUSample(CheckContactBetweenList, 0);

	for (int i = 0; i < bodies1.size(); i++)
	{
		if (bodies1[i]->GetCollider()->empty())continue;

		for (int j = 0; j < bodies2.size(); j++)
		{
			if (bodies2[j]->GetCollider()->empty())continue;
			if (bodies2[j] == bodies1[i])continue;
			CheckContactBetweenBodies(bodies1[i], bodies2[j]);
		}
	}
}


void p2ContactManager::CheckContactBetweenBodies(p2Body* body1, p2Body* body2)
{
	rmt_ScopedCPUSample(ContainContact, 0);
	p2Contact* containedContact = ContainContact(body1, body2);
	if (containedContact)
	{
		if (!CheckAABBContact(body1, body2))
		{
			m_ContactListener->EndContact(containedContact);
			RemoveContact(&body1->GetCollider()->at(0), &body2->GetCollider()->at(0));
		}
		else
		{
			if (!CheckSATContact(body1, body2))
			{
				m_ContactListener->EndContact(containedContact);
				RemoveContact(&body1->GetCollider()->at(0), &body2->GetCollider()->at(0));
			}
		}
	}
	else
	{
		if (CheckAABBContact(body1, body2))
		{
			if (CheckSATContact(body1, body2))
			{
				p2Contact* contact = CreateContact(&body1->GetCollider()->at(0), &body2->GetCollider()->at(0));
				m_ContactListener->BeginContact(contact);
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

/*
bool p2ContactManager::CheckSATContact(p2Body* bodyA, p2Body* bodyB)
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
							if	(p2Vec2::Distance(bodyA->GetPosition(), bodyB->GetPosition()) < circleshapeA->GetRadius()+circleshapeB->GetRadius())
							{
								return true;
							}
						}
					}
					return false;
				}
				else if (colliderB.GetColliderType() == p2ColliderType::BOX)
				{
					if (p2CircleShape* circleshapeA = dynamic_cast<p2CircleShape*>(colliderA.GetShape()))
					{
						if (p2RectShape* rectShapeB = dynamic_cast<p2RectShape*>(colliderB.GetShape()))
						{
							if (bodyB->GetPosition().x - (bodyA->GetPosition().x+circleshapeA->GetRadius()) < rectShapeB->GetSize().x && bodyB->GetPosition().x - (bodyA->GetPosition().x - circleshapeA->GetRadius()) > -rectShapeB->GetSize().x)
							{
								return true;
							}

							if (bodyB->GetPosition().y - (bodyA->GetPosition().y + circleshapeA->GetRadius()) < rectShapeB->GetSize().y && bodyB->GetPosition().y - (bodyA->GetPosition().y - circleshapeA->GetRadius()) > -rectShapeB->GetSize().y)
							{
								return true;
							}
						}
					}
					return false;
					
				}
			} else if (colliderA.GetColliderType() == p2ColliderType::BOX)
			{
				if (colliderB.GetColliderType() == p2ColliderType::CIRCLE)
				{
					if (p2CircleShape* circleshapeB = dynamic_cast<p2CircleShape*>(colliderB.GetShape()))
					{
						if (p2RectShape* rectShapeA = dynamic_cast<p2RectShape*>(colliderA.GetShape()))
						{
							if (bodyA->GetPosition().x - (bodyB->GetPosition().x + circleshapeB->GetRadius()) < rectShapeA->GetSize().x && bodyA->GetPosition().x - (bodyB->GetPosition().x - circleshapeB->GetRadius()) > -rectShapeA->GetSize().x)
							{
								return true;
							}

							if (bodyA->GetPosition().y - (bodyB->GetPosition().y + circleshapeB->GetRadius()) < rectShapeA->GetSize().y && bodyA->GetPosition().y - (bodyB->GetPosition().y - circleshapeB->GetRadius()) > -rectShapeA->GetSize().y)
							{
								return true;
							}
						}
					}
					return false;
				}
				else if (colliderB.GetColliderType() == p2ColliderType::BOX)
				{
					if (p2RectShape* rectShapeA = dynamic_cast<p2RectShape*>(colliderA.GetShape()))
					{
						if (p2RectShape* rectShapeB = dynamic_cast<p2RectShape*>(colliderB.GetShape()))
						{
							
							int axeInfX = 0;
							int axeSupX = 0;
							int axeInfY = 0;
							int axeSupY = 0;
							
							for (p2Vec2 cornerA : rectShapeA->GetCorner())
							{
								float projAxB = p2Vec2::Dot(bodyB->GetPosition() - cornerA + bodyA->GetPosition(), p2Vec2(rectShapeB->GetSize().x, 0)) / p2Vec2::Dot(p2Vec2(rectShapeB->GetSize().x, 0), p2Vec2(rectShapeB->GetSize().x, 0));
								if (projAxB > rectShapeB->GetSize().x)
								{
									axeSupX = true;
								}
								if (projAxB < -rectShapeB->GetSize().x)
								{
									axeInfX = true;

								}

								float projAyB = p2Vec2::Dot(bodyB->GetPosition() - cornerA + bodyA->GetPosition(), p2Vec2(0, rectShapeB->GetSize().y)) / p2Vec2::Dot(p2Vec2(0, rectShapeB->GetSize().y), p2Vec2(0, rectShapeB->GetSize().y));
								if (projAyB > rectShapeB->GetSize().y || projAyB < -rectShapeB->GetSize().y)
								{
									axeInfY = true;
								}
							}
							if (!axeInfX || !axeInfY)
							{
								return false;
							}
							
							axeInfX = 0;
							axeSupX = 0;
							axeInfY = 0;
							axeSupY = 0;
							for (p2Vec2 cornerB : rectShapeB->GetCorner())
							{
								float projBxA = p2Vec2::Dot(bodyA->GetPosition() - cornerB - bodyB->GetPosition(), p2Vec2(rectShapeA->GetSize().Rotate(bodyA->GetAngle()))) / p2Vec2(rectShapeA->GetSize().x, 0).GetMagnitude();
								float projByA = p2Vec2::Dot(bodyA->GetPosition() - cornerB - bodyB->GetPosition(), p2Vec2(0, rectShapeA->GetSize().y).Rotate(bodyA->GetAngle())) / p2Vec2(0, rectShapeA->GetSize().y).GetMagnitude();
								p2Vec2 size = rectShapeA->GetSize().Rotate(bodyA->GetAngle());
								if (abs(projByA) > abs(rectShapeA->GetSize().Rotate(bodyA->GetAngle()).x))
								{
									axeSupX++;
								}
								if (abs(projByA) < abs(rectShapeA->GetSize().Rotate(bodyA->GetAngle()).x))
								{
									axeInfX++;
								}
								if (abs(projBxA) > abs(rectShapeA->GetSize().Rotate(bodyA->GetAngle()).y))
								{
									axeSupY++;
								}
								if (abs(projBxA) < abs(rectShapeA->GetSize().Rotate(bodyA->GetAngle()).y))
								{
									axeInfY++;
								}
							}
							if (axeInfX == 4 || axeSupX == 4 || axeInfY == 4 || axeSupY == 4)
							{
								return false;
							}
						}
					}
					return true;
				}
			}
		}
	}
	return false;
}


*/

bool p2ContactManager::CheckSATContact(p2Body* bodyA, p2Body* bodyB)
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
								return true;
							}
						}
					}
					return false;
				}
				else if (colliderB.GetColliderType() == p2ColliderType::BOX)
				{
					if (p2CircleShape* circleshapeA = dynamic_cast<p2CircleShape*>(colliderA.GetShape()))
					{
						if (p2RectShape* rectShapeB = dynamic_cast<p2RectShape*>(colliderB.GetShape()))
						{
							float newAngle = bodyB->GetAngle() / 180 * M_PI;
							p2Vec2 u = bodyA->GetPosition() - (bodyB->GetPosition());
							if (u.GetMagnitude() < circleshapeA->GetRadius()) return true;
							u = u.Normalized() * (u.GetMagnitude() - circleshapeA->GetRadius());
							p2Vec2 x = p2Vec2(rectShapeB->GetSize().x, 0).Rotate(newAngle);
							p2Vec2 y = p2Vec2(0, rectShapeB->GetSize().y).Rotate(newAngle);

							float kx = (p2Vec2::Dot(u, x) / p2Vec2::Dot(x, x));
							float ky = (p2Vec2::Dot(u, y) / p2Vec2::Dot(y, y));

							return !((kx > 1) || (kx < -1) || (ky > 1) || (ky < -1));
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
							float newAngle = bodyA->GetAngle() / 180 * M_PI;
							p2Vec2 u = bodyB->GetPosition() - (bodyA->GetPosition());
							if (u.GetMagnitude() < circleshapeB->GetRadius()) return true;

							u = u.Normalized() * (u.GetMagnitude() - circleshapeB->GetRadius());
							p2Vec2 x = p2Vec2(rectShapeA->GetSize().x, 0).Rotate(newAngle);
							p2Vec2 y = p2Vec2(0, rectShapeA->GetSize().y).Rotate(newAngle);

							float kx = (p2Vec2::Dot(u, x) / p2Vec2::Dot(x, x));
							float ky = (p2Vec2::Dot(u, y) / p2Vec2::Dot(y, y));

							return !((kx > 1) || (kx < -1) || (ky > 1) || (ky < -1));
						}
					}
					return false;
				}
				else if (colliderB.GetColliderType() == p2ColliderType::BOX)
				{
					if (p2RectShape* rectShapeA = dynamic_cast<p2RectShape*>(colliderA.GetShape()))
					{
						if (p2RectShape* rectShapeB = dynamic_cast<p2RectShape*>(colliderB.GetShape()))
						{
							float projXSup = 0;
							float projXInf = 0;
							float projYInf = 0;
							float projYSup = 0;
							float newAngle = bodyB->GetAngle() / 180 * M_PI;
							p2Vec2 x = p2Vec2(rectShapeB->GetSize().x, 0).Rotate(newAngle);
							p2Vec2 y = p2Vec2(0, rectShapeB->GetSize().y).Rotate(newAngle);

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
							}

							if (projXSup == 4 || projXInf == 4 || projYSup == 4 || projYInf == 4)
							{
								return false;
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
							}

							if (projXSup == 4 || projXInf == 4 || projYSup == 4 || projYInf == 4)
							{
								return false;
							}
						}
					}
					return true;
				}
			}
		}
	}
	return false;
}
