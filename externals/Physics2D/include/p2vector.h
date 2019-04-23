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

#ifndef SFGE_P2VECTOR_H
#define SFGE_P2VECTOR_H

struct p2Vec3;

/**
* \brief Vector class
*/
struct p2Vec2
{
	p2Vec2();

	p2Vec2(float x, float y);

	p2Vec2 operator+(p2Vec2 v);
	p2Vec2 operator+=(p2Vec2 v);
	p2Vec2 operator-(p2Vec2 v);
	p2Vec2 operator-=(p2Vec2 v);
	p2Vec2 operator*=(float f);
	p2Vec2 operator /(float f);
	p2Vec2 operator *(float f);
	/**
	* \brief Dot product of two vectors
	*/
	static float Dot(p2Vec2 v1, p2Vec2 v2);
	/**
	* \brief Cross product of two vectors
	*/
	static p2Vec3 Cross(p2Vec2 v1, p2Vec2 v2);
	/**
	* \brief Calculate the magnitude of the p2Vec2
	*/
	float GetMagnitude();
	/**
	* \brief Calculate a normalized version of the p2Vec2
	*/
	p2Vec2 Normalized();
	/**
	* \brief Normalize the p2Vec2
	*/
	void NormalizeSelf();

	p2Vec2 Rotate(float angle) const;
	static p2Vec2 Lerp(const p2Vec2& v1, const p2Vec2& v2, float t);
	static float AngleBetween(const p2Vec2& v1, const p2Vec2& v2);

	/**
	* \brief 
	*/
	p2Vec3 to3();

	float x = 0.0f;
	float y = 0.0f;

};

struct p2Vec3
{
	p2Vec3();
	p2Vec3(float x, float y, float z);

	p2Vec3 operator+(p2Vec3 v);
	p2Vec3 operator+=(p2Vec3 v);
	p2Vec3 operator-(p2Vec3 v);
	p2Vec3 operator-=(p2Vec3 v);
	p2Vec3 operator*=(float f);
	p2Vec3 operator /(float f);
	p2Vec3 operator *(float f);
	/**
	* \brief Dot product of two vectors
	*/
	static float Dot(p2Vec3 v1, p2Vec3 v2);
	/**
	* \brief Cross product of two vectors
	*/
	static p2Vec3 Cross(p2Vec3 v1, p2Vec3 v2);
	p2Vec3 Rotate(float angle) const;
	static p2Vec3 Lerp(const p2Vec3& v1, const p2Vec3& v2, float t);
	static float AngleBetween(const p2Vec3& v1, const p2Vec3& v2);
	/**
	* \brief Calculate the magnitude of the p2Vec2
	*/
	float GetMagnitude();
	/**
	* \brief Calculate a normalized version of the p2Vec2
	*/
	p2Vec3 Normalized();
	/**
	* \brief Normalize the p2Vec2
	*/
	void NormalizeSelf();
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
};

#endif