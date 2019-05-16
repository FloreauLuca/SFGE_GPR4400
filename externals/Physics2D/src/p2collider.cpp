#include "..\include\p2collider.h"
#include <iostream>

void p2Collider::Init(const p2ColliderDef* colliderDef)
{
	m_UserData = colliderDef->userData;
	m_ColliderType = colliderDef->colliderType;
	m_IsSensor = colliderDef->isSensor;
	m_Shape = colliderDef->shape;
	m_Restitution = colliderDef->restitution;
}


bool p2Collider::IsSensor()
{
	return m_IsSensor;
}

void* p2Collider::GetUserData()
{
	return m_UserData;
}

void p2Collider::SetUserData(void* colliderData)
{
	m_UserData = colliderData;
}

p2Shape* p2Collider::GetShape()
{
	return m_Shape;
}

p2ColliderType p2Collider::GetColliderType()
{
	return m_ColliderType;
}

p2AABB p2Collider::GetAABB(p2Vec2 position, float angle)
{
	if (m_Shape) m_Shape->Rotate(angle);
	m_Aabb.SetCenter(position);
	m_Aabb.SetShape(m_Shape);
	return m_Aabb;
}


