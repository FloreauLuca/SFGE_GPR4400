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
	p2Vec2 angularVelocity;
	float angle; // in degree
	float gravityScale; // influence de la gravity
	float mass;
};

const size_t MAX_COLLIDER_LEN = 8;

/**
* \brief Rigidbody representation
*/
class p2Body
{
public:
	/**
	 * \brief Save the bodyDef information
	 * \param bodyDef p2BodyDef definition of the body
	 */
	void Init(p2BodyDef* bodyDef);
	/**
	 * \return true if the body has been instantiate
	 */
	bool IsInstantiate();
	/**
	* \brief Factory method creating a p2Collider
	* \param colliderDef p2ColliderDef definition of the collider
	* \return p2Collider collider attached to the p2Body
	*/
	p2Collider* CreateCollider(p2ColliderDef* colliderDef);
	/**
	 * \brief Calculate the velocity with a force 
	 */
	void ApplyForceToCenter(const p2Vec2& force);
	/**
	 * \brief Calculate the angular velocity with inertia, force and dist from the center
	 */
	void ApplyForceToCorner(float inertia, const p2Vec2& force, p2Vec2 distCenter);
	/**
	 * \brief Move position and angle according to the velocity and the angular velocity 
	 * \param dt Time
	 */
	void Move(float dt);
	/**
	 * \brief Build collider AABB and calculate Body AABB
	 */
	void BuildAABB();
	
	p2Vec2 GetLinearVelocity() const;
	void SetLinearVelocity(p2Vec2 velocity);
	void SetAngle(float angle);
	float GetAngle();
	float GetAngularVelocity();
	p2Vec2 GetPosition();
	void SetPosition(p2Vec2 newPosition);
	p2AABB GetAABB();
	std::vector<p2Collider>* GetCollider();
	p2BodyType GetType() const;
	float GetMass() const;

private:
	p2AABB m_Aabb;
	p2Vec2 m_Position = p2Vec2(0,0);
	p2Vec2 m_LinearVelocity = p2Vec2(0, 0);
	float m_AngularVelocity;
	p2BodyType m_BodyType;
	/** \brief in KG */
	float m_Mass;
	float m_Angle; // in degree
	bool m_IsInstantiate = false;
	float m_GravityScale; // influence de la gravity
	int m_ColliderIndex = 0;
	std::vector<p2Collider> m_Colliders;
};

#endif