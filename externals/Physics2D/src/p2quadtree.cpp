#include "..\include\p2quadtree.h"

p2QuadTree::p2QuadTree(int nodeLevel, p2AABB bounds)
{
	m_NodeLevel = nodeLevel;
	m_Bounds = bounds;
}

p2QuadTree::~p2QuadTree()
{
	Clear();
}

void p2QuadTree::Clear()
{
	for (p2QuadTree* node : nodes)
	{
		delete node;
	}
}

void p2QuadTree::Split()
{
	p2AABB aabb;
	aabb.topRight = p2Vec2(m_Bounds.GetCenter().x, m_Bounds.topRight.y);
	aabb.bottomLeft = p2Vec2(m_Bounds.bottomLeft.x, m_Bounds.GetCenter().y);
	nodes[0] = new p2QuadTree(0, aabb);
	aabb.topRight = aabb.topRight;
	aabb.bottomLeft = aabb.GetCenter();
	nodes[1] = new p2QuadTree(1, aabb);
	aabb.topRight = aabb.GetCenter();
	aabb.bottomLeft = aabb.bottomLeft;
	nodes[2] = new p2QuadTree(2, aabb);
	aabb.topRight = p2Vec2(m_Bounds.topRight.x, m_Bounds.GetCenter().y);
	aabb.bottomLeft = p2Vec2(m_Bounds.GetCenter().x, m_Bounds.bottomLeft.y);
	nodes[3] = new p2QuadTree(3, aabb);
}

int p2QuadTree::GetIndex(p2Body * rect)
{
	return m_NodeLevel;
}

void p2QuadTree::Insert(p2Body * obj)
{
	m_Objects.push_back(obj);
}

std::list<p2Body*> p2QuadTree::Retrieve()
{
	return m_Objects;
}
