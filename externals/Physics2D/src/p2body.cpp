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
#include <p2body.h>
#include <iostream>

void p2Body::Init(p2BodyDef* bodyDef)
{
	m_Colliders.resize(MAX_COLLIDER_LEN);
	m_Position = bodyDef->position;
	m_LinearVelocity = bodyDef->linearVelocity;
	m_BodyType = bodyDef->type;
	m_Mass = bodyDef->mass;
	m_Angle = bodyDef->angle;
	m_GravityScale = bodyDef->gravityScale;
	m_IsInstantiate = true;
}

bool p2Body::IsInstantiate()
{
	return m_IsInstantiate;
}

p2Vec2 p2Body::GetLinearVelocity() const
{
	return m_LinearVelocity;
}

void p2Body::SetLinearVelocity(p2Vec2 velocity)
{
	m_LinearVelocity = velocity;
}

/**
 * \brief 
 * \param angle in degree
 */
void p2Body::SetAngle(float angle)
{
	m_Angle = angle;
}

float p2Body::GetAngle()
{
	return m_Angle;
}

float p2Body::GetAngularVelocity()
{
	return m_AngularVelocity;
}

p2Vec2 p2Body::GetPosition()
{
	return m_Position;
}


void p2Body::SetPosition(p2Vec2 newPosition)
{
	m_Position = newPosition;
}

p2Collider * p2Body::CreateCollider(p2ColliderDef * colliderDef)
{
	p2Collider& collider = m_Colliders[m_ColliderIndex];
	collider.Init(colliderDef);
	BuildAABB();
	m_ColliderIndex++;
	return &collider;
}


void p2Body::ApplyForceToCenter(const p2Vec2& force)
{
	m_LinearVelocity += force*m_GravityScale/m_Mass;
}

p2BodyType p2Body::GetType() const
{
	return m_BodyType;
}


float p2Body::GetMass() const
{
	return m_Mass;
}

void p2Body::Move(float dt)
{
	m_Position += m_LinearVelocity * dt*1/2;
	//std::cout << "position : " + std::to_string(position.x) + " , " + std::to_string(position.y) + " dt : " + std::to_string(dt) << std::endl; //Debug
}

void p2Body::BuildAABB()
{
	float bottom = m_Position.y;
	float top = m_Position.y;
	float right = m_Position.x;
	float left = m_Position.x;
	for (p2Collider m_collider : m_Colliders)
	{
		if (m_collider.GetUserData() == nullptr) continue;
		p2AABB colliderAABB = m_collider.GetAABB(m_Position, m_Angle);

		if (colliderAABB.bottomLeft.y < bottom)
		{
			m_Aabb.bottomLeft.y = colliderAABB.bottomLeft.y;
		}
		if (colliderAABB.topRight.y > top)
		{
			m_Aabb.topRight.y = colliderAABB.topRight.y;
		}
		if (colliderAABB.bottomLeft.x < left)
		{
			m_Aabb.bottomLeft.x = colliderAABB.bottomLeft.x;
		}
		if (colliderAABB.topRight.x > right)
		{
			m_Aabb.topRight.x = colliderAABB.topRight.x;
		}
	}
}

p2AABB p2Body::GetAABB()
{
	return m_Aabb;
}

std::vector<p2Collider>* p2Body::GetCollider()
{
	return &m_Colliders;
}
