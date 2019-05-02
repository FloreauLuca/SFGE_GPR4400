#include "..\include\p2collider.h"
#include <iostream>

void p2Collider::Init(p2ColliderDef* colliderDef)
{
	userData = colliderDef->userData;
	isSensor = colliderDef->isSensor;
	shape = *colliderDef->shape;
	colliderType = colliderDef->colliderType;
	
}


bool p2Collider::IsSensor()
{
	return isSensor;
}

void* p2Collider::GetUserData()
{
	return userData;
}

void p2Collider::SetUserData(void* colliderData)
{
	userData = colliderData;
}

p2Shape p2Collider::GetShape()
{
	return shape;
}

p2AABB p2Collider::GetAABB(p2Vec2 position)
{
	p2AABB aabb;
	p2Vec2 extend;
	switch (colliderType) {
	case p2ColliderType::NONE:
	{
		std::cout << "none " << std::endl; //Debug

		extend = p2Vec2(0, 0);
		break;
	}
	case p2ColliderType::CIRCLE:
	{
		std::cout << "circle" << std::endl; //Debug

		p2CircleShape* circle_shape = static_cast<p2CircleShape*>(&shape);
		extend = p2Vec2(circle_shape->GetRadius(), circle_shape->GetRadius());
		break;
	}
	case p2ColliderType::BOX:
	{
		std::cout << "box" << std::endl; //Debug
		p2RectShape* rect_shape = static_cast<p2RectShape*>(&shape);
		extend = rect_shape->GetSize();
		break;
	}
	case p2ColliderType::POLYGON:
		extend = p2Vec2(0, 0);
		break;;
	}
	aabb.SetAABB(position, extend);
	return  aabb;
}

