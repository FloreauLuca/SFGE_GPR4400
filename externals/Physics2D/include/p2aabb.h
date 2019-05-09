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

#ifndef SFGE_P2AABB_H
#define SFGE_P2AABB_H

#include <p2vector.h>
#include "p2shape.h"

/**
* \brief Struct representing a Axis Aligned Bounding Box
*/
struct p2AABB
{

	/**
	* \brief Calculate the center and return it
	*/
	p2Vec2 GetCenter();
	/**
	* \brief Calculate the extends and return it
	*/
	p2Vec2 GetExtends();

	float GetBottom();
	float GetTop();
	float GetRight();
	float GetLeft();

	void SetSide(float top, float bottom, float right, float left);
	void SetShape(p2Shape* shape);
	void SetExtends(p2Vec2 extends);
	void SetCenterExtend(p2Vec2 center, p2Vec2 extends);
	bool ContainsPoint(p2Vec2 point);
	bool ContainsAABB(p2AABB aabb);
	void Rotate(float angle);
	void SetCenter(p2Vec2 center);
	
	p2Vec2 topRight;
	p2Vec2 bottomLeft;

};
#endif // !SFGE_P2AABB:H
