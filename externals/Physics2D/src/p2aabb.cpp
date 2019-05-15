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
#include <string>

p2Vec2 p2AABB::GetCenter()
{
	return (topRight+bottomLeft)/2;
}

p2Vec2 p2AABB::GetExtends()
{
	return (topRight - bottomLeft)/2;
}

void p2AABB::SetShape(p2Shape* shape)
{
	if (p2CircleShape* circleshape = dynamic_cast<p2CircleShape*>(shape))
	{
		SetExtends(p2Vec2(circleshape->GetRadius(), circleshape->GetRadius()));
	}
	else if (p2RectShape* rectshape = dynamic_cast<p2RectShape*>(shape))
	{
		p2Vec2 center = GetCenter();
		topRight = center;
		bottomLeft = center;
		for (p2Vec2 corner : rectshape->GetCorner())
		{
			corner += center;
			if (corner.x > topRight.x)
			{
				topRight.x = corner.x;
			}
			if (corner.x < bottomLeft.x)
			{
				bottomLeft.x = corner.x;
			}
			if (corner.y > topRight.y)
			{
				topRight.y = corner.y;
			}
			if (corner.y < bottomLeft.y)
			{
				bottomLeft.y = corner.y;
			}
		}
	}
	else
	{
		SetExtends(p2Vec2(0, 0));
	}
}


void p2AABB::SetExtends(p2Vec2 extends)
{
	p2Vec2 center = GetCenter();
	topRight = center + extends;
	bottomLeft = center - extends;
	
}

void p2AABB::SetCenter(p2Vec2 center)
{
	p2Vec2 extends = GetExtends();
	topRight = center + extends;
	bottomLeft = center - extends;
}


void p2AABB::SetCenterExtend(p2Vec2 center, p2Vec2 extends)
{
	p2Vec2 Extend = extends;
	if (Extend.x < 0) Extend.x -= Extend.x;
	if (Extend.y < 0) Extend.y -= Extend.y;
	topRight = center + Extend;
	bottomLeft = center - Extend;

}

bool p2AABB::ContainsPoint(p2Vec2 point)
{
	return  (point<=topRight && point>=bottomLeft);
}

bool p2AABB::ContainsAABB(p2AABB aabb)
{
	return (ContainsPoint(aabb.topRight) && ContainsPoint(aabb.bottomLeft));
}

void p2AABB::Write()
{
	std::cout << "Top : " + std::to_string(topRight.y) + "Bottom : " + std::to_string(bottomLeft.y) + "Right : " + std::to_string(topRight.x) + "Left : " + std::to_string(bottomLeft.x) << std::endl;
}



