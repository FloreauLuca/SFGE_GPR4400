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
	mass = bodyDef->mass;
	
}

p2Vec2 p2Body::GetLinearVelocity() const
{
	return linearVelocity;
}

void p2Body::SetLinearVelocity(p2Vec2 velocity)
{
	linearVelocity = velocity;
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
	m_ColliderIndex++;
	collider.Init(*colliderDef);
	return &collider;
}

void p2Body::DisplayShape()
{
	return m_Colliders[0].DisplayShape();
}

void p2Body::ApplyForceToCenter(const p2Vec2& force)
{
	if (mass == 0) mass = 1;
	linearVelocity += force/mass;
}

p2BodyType p2Body::GetType() const
{
	return bodyType;
}


float p2Body::GetMass() const
{
	return mass;
}

void p2Body::SetPosition(float dt)
{
	position += linearVelocity * dt*1/2;
	std::cout << "position : " + std::to_string(position.x) + " , " + std::to_string(position.y) + " dt : " + std::to_string(dt) << std::endl; //Debug
}

void p2Body::BuildAABB()
{
	p2AABB p2_aabb;
	p2_aabb.SetAABB(position, p2Vec2(0, 0));
	for (p2Collider m_collider : m_Colliders)
	{
		if (m_collider.GetUserData() == nullptr) continue;
		p2AABB colliderAABB = m_collider.GetAABB(position);
		if (colliderAABB.bottomLeft.x < p2_aabb.bottomLeft.x)
		{
			p2_aabb.bottomLeft.x = colliderAABB.bottomLeft.x;
		}
		if (colliderAABB.bottomLeft.y < p2_aabb.bottomLeft.y)
		{
			p2_aabb.bottomLeft.y = colliderAABB.bottomLeft.y;
		}
		if (colliderAABB.topRight.x > p2_aabb.topRight.x)
		{
			p2_aabb.topRight.x = colliderAABB.topRight.x;
		}
		if (colliderAABB.topRight.y > p2_aabb.topRight.y)
		{
			p2_aabb.topRight.y = colliderAABB.topRight.y;
		}

	}
	aabb = p2_aabb;
	std::cout << "top : " + std::to_string(aabb.topRight.y) + " bottom : " + std::to_string(aabb.bottomLeft.y) + " right : " + std::to_string(aabb.topRight.x) + " left : " + std::to_string(aabb.bottomLeft.x) << std::endl; // Debug
}

p2AABB p2Body::GetAABB()
{
	BuildAABB();
	return aabb;
}
