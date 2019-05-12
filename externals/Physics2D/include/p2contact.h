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

#ifndef SFGE_P2CONTACT_H
#define SFGE_P2CONTACT_H

#include <p2collider.h>
#include "p2body.h"
#include "p2quadtree.h"

const size_t MAX_CONTACT_LEN = 100;

/**
* \brief Representation of a contact given as argument in a p2ContactListener
*/
class p2Contact
{
public:
	void Init(p2Collider * colliderA, p2Collider * colliderB);
	p2Collider* GetColliderA();
	p2Collider* GetColliderB();
	bool CheckSameCollider(p2Collider* colliderA, p2Collider* colliderB);
private :
	p2Collider* m_colliderA;
	p2Collider* m_colliderB;
	int m_ContactIndex;
};

/**
* \brief Listener of contacts happening in an attached p2World
*/
class p2ContactListener
{
public:
	virtual void BeginContact(p2Contact* contact) = 0;
	virtual void EndContact(p2Contact* contact) = 0;
};

/**
* \brief Managing the creation and destruction of contact between colliders
*/
class p2ContactManager
{
public:
	void Init(p2ContactListener* contactListener);
	p2Contact* CreateContact(p2Collider* colliderA, p2Collider* colliderB);
	void RemoveContact(p2Collider* colliderA, p2Collider* colliderB);
	void CheckContact(std::vector<p2Body>& bodies);
	void CheckContactInsideVector(std::vector<p2Body> & m_Bodies);
	void CheckContactBetweenVector(std::vector<p2Body>& bodies1, std::vector<p2Body>& bodies2);
private:
	bool CheckAABBContact(p2Body* bodyA, p2Body* bodyB);
	p2Contact* ContainContact(p2Body* bodyA, p2Body* bodyB);
	p2ContactListener* m_ContactListener;
	std::vector<p2Contact> m_Contacts;
	int m_ContactIndex = 0;
	const int SCREEN_SIZE = 100;
	p2QuadTree m_RootQuadTree;

};
#endif