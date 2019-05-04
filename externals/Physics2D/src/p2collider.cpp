#include "..\include\p2collider.h"
#include <iostream>

void p2Collider::Init(p2ColliderDef colliderDef)
{
	m_colliderDef = colliderDef;
}


bool p2Collider::IsSensor()
{
	return m_colliderDef.isSensor;
}

void* p2Collider::GetUserData()
{
	return m_colliderDef.userData;
}

void p2Collider::SetUserData(void* colliderData)
{
	m_colliderDef.userData = colliderData;
}

p2Shape* p2Collider::GetShape()
{
	return m_colliderDef.shape;
}

p2ColliderType p2Collider::GetColliderType()
{
	return m_colliderDef.colliderType;
}

p2AABB p2Collider::GetAABB(p2Vec2 position)
{
	p2AABB aabb;
	p2Vec2 extend;

	switch (m_colliderDef.colliderType)
	{
	case p2ColliderType::NONE:
		{
			extend = p2Vec2(0, 0);
			std::cout << "none " + std::to_string(extend.x) << std::endl; //Debug
			break;
		}
	case p2ColliderType::CIRCLE:
		{
			p2CircleShape* circle_shape = static_cast<p2CircleShape*>(m_colliderDef.shape);
			extend = p2Vec2(circle_shape->GetRadius(), circle_shape->GetRadius());
			std::cout << "circle " + std::to_string(circle_shape->GetRadius()) << std::endl; //Debug
			break;
		}
	case p2ColliderType::BOX:
		{
			p2RectShape* rect_shape = static_cast<p2RectShape*>(m_colliderDef.shape);
			extend = rect_shape->GetSize();
			std::cout << "box " + std::to_string(extend.x) << std::endl; //Debug
			break;
		}
	case p2ColliderType::POLYGON:
		extend = p2Vec2(0, 0);
		break;;
	}

	aabb.SetAABB(position, extend);
	return aabb;
}


