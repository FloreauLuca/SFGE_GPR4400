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

p2Contact::p2Contact(p2Collider* colliderA, p2Collider* colliderB)
{
	m_colliderA = colliderA;
	m_colliderB = colliderB;
}

p2Collider * p2Contact::GetColliderA()
{
	return nullptr;
}

p2Collider * p2Contact::GetColliderB()
{
	return nullptr;
}

void p2ContactManager::Init(p2ContactListener* contactListener)
{
	m_ContactListener = contactListener;
}

void p2ContactManager::CheckContact(std::vector<p2Body> & bodies)
{
	for (int i = 0; i < bodies.size(); i++)
	{
		for (int j = i; j < bodies.size(); i++)
		{
			CheckAABBContact(&bodies[i], &bodies[j]);
		}
	}
}

void p2ContactManager::CheckAABBContact(p2Body* bodyA, p2Body* bodyB)
{
	p2Contact contact = p2Contact(&bodyA->GetCollider().at(0), &bodyB->GetCollider().at(0));
	p2AABB aabbA = bodyA->GetAABB();
	p2AABB aabbB = bodyB->GetAABB();
	if (aabbA.bottomLeft.x > aabbB.bottomLeft.x && aabbA.bottomLeft.x < aabbB.topRight.x)
	{
		if (aabbA.bottomLeft.y > aabbB.bottomLeft.y && aabbA.bottomLeft.y < aabbB.topRight.y)
		{
			m_ContactListener->BeginContact(&contact);
		}
		if (aabbA.topRight.y > aabbB.bottomLeft.y && aabbA.topRight.y < aabbB.topRight.y)
		{
			m_ContactListener->BeginContact(&contact);
		}
	}
	if (aabbA.topRight.x > aabbB.bottomLeft.x && aabbA.topRight.x < aabbB.topRight.x)
	{
		if (aabbA.bottomLeft.y > aabbB.bottomLeft.y && aabbA.bottomLeft.y < aabbB.topRight.y)
		{
			m_ContactListener->BeginContact(&contact);
		}
		if (aabbA.topRight.y > aabbB.bottomLeft.y && aabbA.topRight.y < aabbB.topRight.y)
		{
			m_ContactListener->BeginContact(&contact);
		}
	}
	//m_ContactListener->EndContact(&contact);
}


