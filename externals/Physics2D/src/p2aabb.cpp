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

#include <p2aabb.h>
#include <iostream>
#include <corecrt_math_defines.h>
#include <vector>
#include <algorithm>
#include "p2shape.h"

p2Vec2 p2AABB::GetCenter()
{
	return Center;
}

p2Vec2 p2AABB::GetExtends()
{
	return Extends;
}

float p2AABB::GetBottom()
{
	return Bottom;
}

float p2AABB::GetTop()
{
	return Top;
}

float p2AABB::GetRight()
{
	return Right;
}

float p2AABB::GetLeft()
{
	return Left;
}

void p2AABB::SetSide(float top, float bottom, float right, float left)
{
	if (top<bottom)
	{
		Top = bottom;
		Bottom = top;
	} else
	{
		Top = top;
		Bottom = bottom;
	}
	if (left < right)
	{
		Left = left;
		Right = right;
	} else
	{
		Left = right;
		Right = left;
	}
	Extends = p2Vec2(top - bottom, right - left);
}


void p2AABB::SetShape(p2Shape* shape)
{
	if (p2CircleShape* circleshape = dynamic_cast<p2CircleShape*>(shape))
	{
		SetExtends(p2Vec2(circleshape->GetRadius(), circleshape->GetRadius()));
	}
	else if (p2RectShape* rectshape = dynamic_cast<p2RectShape*>(shape))
	{
		Top= 0;
		Bottom = 0;
		Left = 0;
		Right = 0;
		for (p2Vec2 corner : rectshape->GetCorner())
		{
			if (corner.x > Right)
			{
				Right = corner.x;
			}
			if (corner.x < Left)
			{
				Left = corner.x;
			}
			if (corner.y > Top)
			{
				Top = corner.y;
			}
			if (corner.y < Bottom)
			{
				Bottom = corner.y;
			}
		}
		SetSide(Top + Center.y, Bottom + Center.y, Right + Center.x, Left + Center.x);
	}
	else {
		SetExtends(p2Vec2(0, 0));
	}
}

void p2AABB::SetExtends(p2Vec2 extends)
{
	
	Extends = extends;
	Top = Center.y + Extends.y;
	Bottom = Center.y - Extends.y;
	Right = Center.x + Extends.x;
	Left = Center.x - Extends.x;
	
}

void p2AABB::SetCenter(p2Vec2 center)
{
	Center = center;
}


void p2AABB::SetCenterExtend(p2Vec2 center, p2Vec2 extends)
{
	Center = center;
	if (extends.x < 0) extends.x -= extends.x;
	if (extends.y < 0) extends.y -= extends.y;
	Extends = extends;
	Top = Center.y + Extends.y;
	Bottom = Center.y - Extends.y;
	Right = Center.x + Extends.x;
	Left = Center.x - Extends.x;
}



