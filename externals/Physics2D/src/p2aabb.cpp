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

void p2AABB::SetCorner(float top, float bottom, float right, float left)
{
	Top = top;
	Bottom = bottom;
	Left = left;
	Right = right;
	Extends = p2Vec2(top - bottom, right - left);
}

void p2AABB::SetCenterExtend(p2Vec2 center, p2Vec2 extends)
{
	Center = center;
	Extends = extends;
	Top = Center.y + Extends.y;
	Bottom = Center.y - Extends.y;
	Right = Center.x + Extends.x;
	Left = Center.x - Extends.x;
}


