#include "..\include\p2collider.h"

void p2Collider::Init(p2ColliderDef* colliderDef)
{
	userData = colliderDef->userData;
	isSensor = colliderDef->isSensor;
	shape = *colliderDef->shape;
}


bool p2Collider::IsSensor()
{
	return isSensor;
}

void * p2Collider::GetUserData()
{
	return userData;
}

void p2Collider::SetUserData(void* colliderData)
{
	userData = colliderData;
}
