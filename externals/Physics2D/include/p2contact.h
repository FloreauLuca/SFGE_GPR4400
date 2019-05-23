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
#include "p2matrix.h"

const size_t MAX_CONTACT_LEN = 1000;

/**
* \brief Representation of a contact given as argument in a p2ContactListener
*/
class p2Contact
{
public:
	/**
	 * \brief Set the colliders ptr in contact
	 */
	void Init(p2Collider * colliderA, p2Collider * colliderB);
	p2Collider* GetColliderA();
	p2Collider* GetColliderB();
	/**
	 * \brief Check if the contact contain the same collider
	 */
	bool CheckSameCollider(p2Collider* colliderA, p2Collider* colliderB);
private :
	p2Collider* m_ColliderA = nullptr;
	p2Collider* m_ColliderB = nullptr;
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
* \brief Managing the creation and destruction of contact between colliders and correct the position and the speed
*/
class p2ContactManager
{
public:
	/**
	 * \brief Save the contactListener adress
	 */
	void Init(p2ContactListener* contactListener);
	/**
	 * \brief Factory method creating a p2Contact and save it
	 */
	p2Contact* CreateContact(p2Collider* colliderA, p2Collider* colliderB);
	/**
	 * \brief Factory method removing a p2Contact
	 */
	void RemoveContact(p2Collider* colliderA, p2Collider* colliderB);
	/**
	 * \brief Construct, split and retrieve the QuadTree
	 */
	void CheckContact(std::vector<p2Body>& bodies);
	/**
	 * \brief Check contact inside a vector of p2Body
	 */
	void CheckContactInsideList(std::vector<p2Body*> m_Bodies);
	/**
	 * \brief Check contact between 2 vector of p2Body
	 */
	void CheckContactBetweenList(std::vector<p2Body*> bodies1, std::vector<p2Body*> bodies2);
	/**
	 * \brief Check contact between 2 p2Body
	 */
	void CheckContactBetweenBodies(p2Body* body1, p2Body* body2);
	/**
	 * \brief Return the QuadTree
	 */
	p2QuadTree* GetQuadtree();
private:
	/**
	 * \brief Check the AABB contact between 2 p2Body
	 */
	bool CheckAABBContact(p2Body* bodyA, p2Body* bodyB);
	/**
	 * \brief Check the SAT contact between 2 p2Body
	 */
	p2Mat22 CheckSATContact(p2Body* bodyA, p2Body* bodyB);
	/**
	 * \brief Check if 2 p2Body are alread in contact
	 */
	p2Contact* ContainContact(p2Body* bodyA, p2Body* bodyB);
	p2ContactListener* m_ContactListener = nullptr;
	std::vector<p2Contact> m_Contacts;
	int m_ContactIndex = 0;
	const p2Vec2 SCREEN_SIZE = p2Vec2(13,7.5); //TODO set dynamic quadtree
	p2QuadTree m_RootQuadTree;

};
#endif