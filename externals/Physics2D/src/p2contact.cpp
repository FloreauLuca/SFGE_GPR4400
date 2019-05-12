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

void p2Contact::Init(p2Collider* colliderA, p2Collider* colliderB)
{
	m_colliderA = colliderA;
	m_colliderB = colliderB;
}

p2Collider* p2Contact::GetColliderA()
{
	return m_colliderA;
}

p2Collider* p2Contact::GetColliderB()
{
	return m_colliderB;
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
	for (int i = 0; i < m_Contacts.size(); i++)
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
	m_RootQuadTree = p2QuadTree(0, rootAABB );
	for (p2Body& body : bodies)
	{
		if (body.GetType() != p2BodyType::STATIC)
		{
			m_RootQuadTree.Insert(&body);
		}
	}
	m_RootQuadTree.Split();
	rmt_ScopedCPUSample(CheckContact, 0);
	m_RootQuadTree.Retrieve(this);
}

void p2ContactManager::CheckContactInsideVector(std::vector<p2Body*> bodies)
{
	rmt_ScopedCPUSample(CheckContactInsideVector, 0);
	for (int i = 0; i < bodies.size(); i++)
	{
		if (bodies[i]->GetCollider()->empty())continue;

		for (int j = i; j < bodies.size(); j++)
		{
			if (bodies[j]->GetCollider()->empty())continue;
			p2Contact* containedContact = ContainContact(bodies[i], bodies[j]);
			if (containedContact)
			{
				if (!CheckAABBContact(bodies[i], bodies[j]))
				{
					m_ContactListener->EndContact(containedContact);
					RemoveContact(&bodies[i]->GetCollider()->at(0), &bodies[j]->GetCollider()->at(0));
				}
			}
			else
			{
				if (CheckAABBContact(bodies[i], bodies[j]))
				{
					p2Contact* contact = CreateContact(&bodies[i]->GetCollider()->at(0), &bodies[j]->GetCollider()->at(0));
					m_ContactListener->BeginContact(contact);
				}
			}
		}
	}
}

void p2ContactManager::CheckContactBetweenVector(std::vector<p2Body*> bodies1, std::vector<p2Body*> bodies2)
{
	rmt_ScopedCPUSample(CheckContactBetweenVector, 0);

	for (int i = 0; i < bodies1.size(); i++)
	{
		if (bodies1[i]->GetCollider()->empty())continue;

		for (int j = 0; j < bodies2.size(); j++)
		{
			if (bodies2[j]->GetCollider()->empty())continue;
			if (bodies2[j] == bodies1[i])continue;
			p2Contact* containedContact = ContainContact(bodies1[i], bodies2[j]);
			if (containedContact)
			{
				if (!CheckAABBContact(bodies1[i], bodies2[j]))
				{
					m_ContactListener->EndContact(containedContact);
					RemoveContact(&bodies1[i]->GetCollider()->at(0), &bodies2[j]->GetCollider()->at(0));
				}
			}
			else
			{
				if (CheckAABBContact(bodies1[i], bodies2[j]))
				{
					p2Contact* contact = CreateContact(&bodies1[i]->GetCollider()->at(0), &bodies2[j]->GetCollider()->at(0));
					m_ContactListener->BeginContact(contact);
				}
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
	if (aabbA.ContainsPoint(aabbB.topRight)|| aabbA.ContainsPoint(aabbB.bottomLeft) || aabbA.ContainsPoint(p2Vec2(aabbB.bottomLeft.x, aabbB.topRight.y)) || aabbA.ContainsPoint(p2Vec2(aabbB.topRight.x, aabbB.bottomLeft.y)))
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
	for (p2Contact& m_contact : m_Contacts)
	{
		if (m_contact.CheckSameCollider(&bodyA->GetCollider()->at(0), &bodyB->GetCollider()->at(0)))
		{
			return &m_contact;
		}
	}
	return nullptr;
}


