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

#ifndef SFGE_P2MATRIX_H
#define SFGE_P2MATRIX_H

#include <p2vector.h>

struct p2Mat22
{
	p2Mat22();
	p2Mat22(p2Vec2 r1, p2Vec2 r2) ;

	bool operator==(p2Mat22 m1);
	bool operator!=(p2Mat22 m1);
	p2Mat22 operator+(p2Mat22 m1);
	p2Mat22 operator-(p2Mat22 m1);
	p2Mat22 operator*(p2Mat22 m1);
	p2Vec2 operator*(p2Vec2 v);
	p2Mat22 operator*(float f);
	p2Mat22 operator/(float f);
	/**
	* \brief Calculate the invert of the 2x2 matrix
	*/
	p2Mat22 Invert();
	/**
	* \brief Calculate the determinant
	*/
	float GetDeterminant();

	p2Vec2 rows[2] = {};
};

struct p2Mat33
{
	p2Mat33();
	p2Mat33(p2Vec3 r1, p2Vec3 r2, p2Vec3 r3);

	p2Mat33 operator+(p2Mat33 m1);
	p2Mat33 operator-(p2Mat33 m1);
	p2Mat33 operator*(p2Mat33 m1);
	p2Vec3 operator*(p2Vec3 v);
	p2Mat33 operator*(float f);
	p2Mat33 operator/(float f);
	/**
	* \brief Calculate the invert of the 3x3 matrix
	*/
	p2Mat33 Invert();
	/**
	* \brief Calculate the determinant
	*/
	float GetDeterminant();

	p2Vec3 rows[3] = {};
};

#endif