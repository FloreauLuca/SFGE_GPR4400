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

#ifndef SFGE_P2BODY_H
#define SFGE_P2BODY_H

#include <p2aabb.h>
#include <vector>
#include <p2collider.h>

class p2Collider;
struct p2ColliderDef;

enum class p2BodyType
{
	STATIC,
	KINEMATIC,
	DYNAMIC
};

/**
* \brief Struct Defining the p2Body when creating it
*/
struct p2BodyDef
{
	p2BodyType type;
	p2Vec2 position;
	p2Vec2 linearVelocity;
	float gravityScale;
	float mass;
};

const size_t MAX_COLLIDER_LEN = 8;

/**
* \brief Rigidbody representation
*/
class p2Body
{
public:
	void Init(p2BodyDef* bodyDef);
	p2Vec2 GetLinearVelocity() const;
	
	void SetLinearVelocity(p2Vec2 velocity);

	float GetAngularVelocity();
	
	p2Vec2 GetPosition();
	/**
	* \brief Factory method creating a p2Collider
	* \param colliderDef p2ColliderDef definition of the collider
	* \return p2Collider collider attached to the p2Body
	*/
	p2Collider* CreateCollider(p2ColliderDef* colliderDef);
	p2Shape GetShape();
	void ApplyForceToCenter(const p2Vec2& force);
	p2BodyType GetType() const;
	float GetMass() const;
	void Move(float dt);
private:
	p2AABB aabb;
	p2Vec2 position;
	p2Vec2 linearVelocity;
	float angularVelocity;
	p2BodyType bodyType;
	/**
	 * \brief in KG
	 */
	float mass;

	int m_ColliderIndex = 0;
	std::vector<p2Collider> m_Colliders;
};

#endif