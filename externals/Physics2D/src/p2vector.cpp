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

#include <p2vector.h>
#include <cmath>
#include "p2matrix.h"

p2Vec2::p2Vec2()
{
}

p2Vec2::p2Vec2(float x, float y)
{
	this->x = x;
	this->y = y;
}

p2Vec2 p2Vec2::operator+(const p2Vec2& v) const
{
	return p2Vec2(x + v.x, y + v.y);
}

p2Vec2& p2Vec2::operator+=(const p2Vec2& v)
{
	x += v.x;
	y += v.y;
	return *this;
}

p2Vec2 p2Vec2::operator-(const p2Vec2& v) const
{
	return p2Vec2(x - v.x, y - v.y);
}

p2Vec2& p2Vec2::operator-=(const p2Vec2& v)
{
	x -= v.x;
	y -= v.y;
	return *this;
}

p2Vec2& p2Vec2::operator*=(float f)
{
	x *= f;
	y *= f;
	return (*this);
}

p2Vec2 p2Vec2::operator/(float f) const
{
	if (f == 0) f = 1;
	return p2Vec2(x / f, y / f);
}

p2Vec2 p2Vec2::operator*(float f) const
{
	return p2Vec2(x*f, y*f);
}

float p2Vec2::Dot(p2Vec2 v1, p2Vec2 v2)
{
	// DONE
	return v1.x*v2.x + v1.y*v2.y;
}
p2Vec3 p2Vec2::Cross(p2Vec2 v1, p2Vec2 v2)
{
	// DONE
	return p2Vec3(0, 0, v1.x*v2.y - v1.y - v2.x);
}
float p2Vec2::GetMagnitude() const
{
	// DONE
	return sqrt(x*x + y * y);
}

p2Vec2 p2Vec2::Normalized()
{
	// DONE
	if (GetMagnitude() == 0) return *this;
	return p2Vec2(x / GetMagnitude(), y / GetMagnitude());
}

void p2Vec2::NormalizeSelf()
{
	// DONE
	if (GetMagnitude()==0) return;
	x /= GetMagnitude();
	y /= GetMagnitude();
}

p2Vec2 p2Vec2::Rotate(float angle) const
{
	return p2Mat22(p2Vec2(cos(angle), sin(angle)), p2Vec2(-sin(angle), cos(angle))) *p2Vec2(x, y);
}

/**
 * \brief Lerp
 * \param v1 start vector
 * \param v2 end vector
 * \param t lerp percent
 * \return p2Vec2 Lerp
 */
p2Vec2 p2Vec2::Lerp(const p2Vec2& v1, const p2Vec2& v2, float t)
{
	// DONE
	
	return v1 + (v2-v1)*t;
}

float p2Vec2::AngleBetween(const p2Vec2& v1, const p2Vec2& v2)
{
	// DONE
	if ((v1.GetMagnitude()*v2.GetMagnitude()) == 0) return 0;
	return acos(Dot(v1, v2) / (v1.GetMagnitude()*v2.GetMagnitude()));
}


p2Vec3 p2Vec2::to3()
{
	return p2Vec3(x, y, 0.0f);
}

/**
 * \brief if x AND y inferior
 */
bool p2Vec2::operator<(const p2Vec2& v) const
{
	return  (x < v.x && y < v.y);
}

/**
 * \brief if x AND y superior
 */
bool p2Vec2::operator>(const p2Vec2& v) const
{
	return (x > v.x && y > v.y);
}

p2Vec3::p2Vec3()
{
}

p2Vec3::p2Vec3(float x, float y, float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

p2Vec3 p2Vec3::operator+(const p2Vec3& v)const
{
	return p2Vec3(x + v.x, y + v.y, z + v.z);
}

p2Vec3& p2Vec3::operator+=(const p2Vec3& v)
{
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}

p2Vec3 p2Vec3::operator-(const p2Vec3& v) const
{
	return p2Vec3(x - v.x, y - v.y, z - v.z);
}

p2Vec3& p2Vec3::operator-=(const p2Vec3& v)
{
	x -= v.x;
	y -= v.y;
	z -= v.z;
	return *this;
}

p2Vec3& p2Vec3::operator*=(float f)
{
	x *= f;
	y *= f;
	z *= z;
	return (*this);
}

p2Vec3 p2Vec3::operator/(float f) const
{
	return p2Vec3(x / f, y / f, z / f);
}

p2Vec3 p2Vec3::operator*(float f) const
{
	return p2Vec3(x*f, y*f, z*f);
}

float p2Vec3::Dot(p2Vec3 v1, p2Vec3 v2)
{
	//DONE
	return v1.x*v2.x+ v1.y*v2.y + v1.z*v2.z;
}

p2Vec3 p2Vec3::Cross(p2Vec3 v1, p2Vec3 v2)
{
	//DONE
	return p2Vec3(v1.y*v2.z-v1.z*v2.y, v1.z*v2.x-v1.x*v2.z, v1.x*v2.y-v1.y*v2.x);
}

p2Vec3 p2Vec3::Rotate(float angle) const
{
	//TODO quel axe?
	return p2Vec3();
}

p2Vec3 p2Vec3::Lerp(const p2Vec3& v1, const p2Vec3& v2, float t)
{
	return v1 + (v2 - v1)*t;
}

float p2Vec3::AngleBetween(const p2Vec3& v1, const p2Vec3& v2)
{
	//TODO
	return 0.0f;
}

float p2Vec3::GetMagnitude()
{
	return sqrt(x*x + y * y + z*z);
}

p2Vec3 p2Vec3::Normalized()
{
	return p2Vec3(x / GetMagnitude(), y / GetMagnitude(), z/GetMagnitude());
}

void p2Vec3::NormalizeSelf()
{
	//DONE
	x /= GetMagnitude();
	y /= GetMagnitude();
	z /= GetMagnitude();
}
