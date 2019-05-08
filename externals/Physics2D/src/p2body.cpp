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
	position = bodyDef->position;
	linearVelocity = bodyDef->linearVelocity;
	bodyType = bodyDef->type;
	m_Mass = bodyDef->mass;
	m_Angle = bodyDef->angle;
	
}



p2Vec2 p2Body::GetLinearVelocity() const
{
	return linearVelocity;
}

void p2Body::SetLinearVelocity(p2Vec2 velocity)
{
	linearVelocity = velocity;
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
	return angularVelocity;
}

p2Vec2 p2Body::GetPosition()
{
	return position;
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
	if (m_Mass == 0) m_Mass = 1;
	linearVelocity += force/m_Mass;
}

p2BodyType p2Body::GetType() const
{
	return bodyType;
}


float p2Body::GetMass() const
{
	return m_Mass;
}

void p2Body::Move(float dt)
{
	position += linearVelocity * dt*1/2;
	//std::cout << "position : " + std::to_string(position.x) + " , " + std::to_string(position.y) + " dt : " + std::to_string(dt) << std::endl; //Debug
}

void p2Body::BuildAABB()
{
	float bottom = position.y;
	float top = position.y;
	float right = position.x;
	float left = position.x;
	for (p2Collider m_collider : m_Colliders)
	{
		if (m_collider.GetUserData() == nullptr) continue;
		p2AABB colliderAABB = m_collider.GetAABB(position, m_Angle);

		if (colliderAABB.GetBottom() < bottom)
		{
			bottom = colliderAABB.GetBottom();
		}
		if (colliderAABB.GetTop() > top)
		{
			top = colliderAABB.GetTop();
		}
		if (colliderAABB.GetLeft() < left)
		{
			left = colliderAABB.GetLeft();
		}
		if (colliderAABB.GetRight() > right)
		{
			right = colliderAABB.GetRight();
		}
	}
	aabb.SetSide(top, bottom, right, left);
	std::cout << "top : " + std::to_string(aabb.GetTop()) + " bottom : " + std::to_string(aabb.GetBottom()) + " right : " + std::to_string(aabb.GetRight()) + " left : " + std::to_string(aabb.GetLeft()) << std::endl; // Debug
}

p2AABB p2Body::GetAABB()
{
	return aabb;
}

std::vector<p2Collider>* p2Body::GetCollider()
{
	return &m_Colliders;
}
