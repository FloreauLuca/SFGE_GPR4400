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


#ifndef SFGE_P2COLLIDER_H
#define SFGE_P2COLLIDER_H

#include <p2shape.h>
#include "engine/entity.h"
#include "p2aabb.h"
enum class p2ColliderType
{
	NONE,
	CIRCLE,
	BOX,
	POLYGON
};


/**
* \brief Struct defining a p2Collider when creating one
*/
struct p2ColliderDef
{
	
	p2ColliderDef()
	{
		shape = nullptr;
		userData = nullptr;
		restitution = 0.0f;
		isSensor = false;
	}
	
	void* userData;
	p2Shape* shape;
	float restitution;
	bool isSensor;
	p2ColliderType colliderType;
};

/**
* \brief Representation of a Collider attached to a p2Body
*/

class p2Collider
{
public:
	void Init(const p2ColliderDef* colliderDef);

	/**
	* \brief Check if the p2Collider is a sensor
	*/
	bool IsSensor();
	/**
	* \brief Return the userData
	*/
	void* GetUserData();
	p2Shape* GetShape();
	p2ColliderType GetColliderType();
	void DisplayShape();
	p2AABB GetAABB(p2Vec2 position, float angle);
	void SetUserData(void* colliderData);
private:
	void* m_UserData;
	p2Shape* m_Shape;
	float m_Restitution;
	bool m_IsSensor;
	p2ColliderType m_ColliderType;
	p2AABB aabb;


};


#endif