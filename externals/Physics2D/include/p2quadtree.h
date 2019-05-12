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

#ifndef SFGE_P2QUADTREE_H
#define SFGE_P2QUADTREE_H

#include <list>

#include <p2vector.h>
#include <p2aabb.h>
#include <p2body.h>

class p2ContactManager;

/**
* \brief Representation of a tree with 4 branches containing p2Body defined by their p2AABB
*/
class p2QuadTree
{
public:
	p2QuadTree();
	p2QuadTree(int nodeLevel, p2AABB bounds, p2ContactManager contactManager);
	~p2QuadTree();

	/**
	* Remove all objects leafs and quadtrees children
	*/
	void Clear();
	/**
	* Called when node have too much objects and split the current node into four
	*/
	void Split();

	/**
	* Get the index of the child trees of the p2Body
	*/
	int GetIndex(p2Body* rect);
	p2AABB GetBounds();
	void GetChild(std::vector<p2QuadTree>& node);
	/**
	* Insert a new p2Body in the tree
	*/
	void Insert(p2Body* obj);
	/**
	* Return a list of all the p2Body that might collide
	*/
	void Retrieve();
	
private:


	static const int MAX_OBJECTS = 10;
	static const int MAX_LEVELS = 5;
	static const int CHILD_TREE_NMB = 4;
	int m_NodeLevel = -1;
	p2QuadTree* nodes[CHILD_TREE_NMB] = { nullptr };
	std::list<p2Body*> m_Objects = {nullptr};
	std::list<p2Body*> m_ChildOject = {nullptr};
	p2AABB m_Bounds;
	p2ContactManager* m_ContactManager;
};

#endif