#include "..\include\p2quadtree.h"
#include "p2contact.h"

p2QuadTree::p2QuadTree()
{
	
}

p2QuadTree::p2QuadTree(int nodeLevel, p2AABB bounds, p2ContactManager contactManager)
{
	m_NodeLevel = nodeLevel;
	m_Bounds = bounds;
	m_ContactManager = &contactManager;
}

p2QuadTree::~p2QuadTree()
{
	Clear();
}

void p2QuadTree::Clear()
{
	if (nodes == nullptr) return;
	if (nodes[0] == nullptr) return;
	for (p2QuadTree* node : nodes)
	{
		node->Clear();
		delete node;
		node = nullptr;
	}
}

void p2QuadTree::Split()
{
	if (m_NodeLevel<MAX_LEVELS && m_Objects.size()>MAX_OBJECTS)
	{
		int nodeLevel = m_NodeLevel + 1;
		p2AABB aabb;
		aabb.topRight = p2Vec2(m_Bounds.GetCenter().x, m_Bounds.topRight.y);
		aabb.bottomLeft = p2Vec2(m_Bounds.bottomLeft.x, m_Bounds.GetCenter().y);
		nodes[0] = new p2QuadTree(nodeLevel, aabb, *m_ContactManager);
		aabb.topRight = aabb.topRight;
		aabb.bottomLeft = aabb.GetCenter();
		nodes[1] = new p2QuadTree(nodeLevel, aabb, *m_ContactManager);
		aabb.topRight = aabb.GetCenter();
		aabb.bottomLeft = aabb.bottomLeft;
		nodes[2] = new p2QuadTree(nodeLevel, aabb, *m_ContactManager);
		aabb.topRight = p2Vec2(m_Bounds.topRight.x, m_Bounds.GetCenter().y);
		aabb.bottomLeft = p2Vec2(m_Bounds.GetCenter().x, m_Bounds.bottomLeft.y);
		nodes[3] = new p2QuadTree(nodeLevel, aabb, *m_ContactManager);
		for (int i = 0; i < CHILD_TREE_NMB; i++)
		{
			for (p2Body* object : m_Objects)
			{
				if(object == nullptr) continue; 
				if (nodes[i]->GetBounds().ContainsAABB(object->GetAABB()))
				{
					nodes[i]->Insert(object);
					m_Objects.remove(object);
					m_ChildOject.push_back(object);
					nodes[i]->Split();
				}
			}
		}
	} else
	{
		m_Bounds.Write();
	}
}

int p2QuadTree::GetIndex(p2Body * rect)
{
	return m_NodeLevel;
}

p2AABB p2QuadTree::GetBounds()
{
	return m_Bounds;
}

void p2QuadTree::GetChild(std::vector<p2QuadTree>& nodes)
{
	nodes.push_back(*this);
	for (p2QuadTree& quad_tree : nodes)
	{
		quad_tree.GetChild(nodes);
	}
}

void p2QuadTree::Insert(p2Body * obj)
{
	m_Objects.push_back(obj);
}

void p2QuadTree::Retrieve()
{
	if (nodes[0] == nullptr)
	{
		std::vector<p2Body> objects;// (m_Objects.begin(), m_Objects.end());
		m_ContactManager->CheckContactInsideVector(objects);
	} else
	{
		for (p2QuadTree* node : nodes)
		{
			node->Retrieve();
		}
		std::vector<p2Body> objects;//(m_Objects.begin(), m_Objects.end());
		std::vector<p2Body> childObjects;// (m_Objects.begin(), m_Objects.end());
		m_ContactManager->CheckContactBetweenVector(objects, childObjects);
	}
}
