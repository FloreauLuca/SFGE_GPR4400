#include "..\include\p2quadtree.h"
#include "p2contact.h"

p2QuadTree::p2QuadTree()
{
	
}

p2QuadTree::p2QuadTree(int nodeLevel, p2AABB bounds)
{
	m_NodeLevel = nodeLevel;
	m_Bounds = bounds;
	m_Objects = std::list<p2Body*>();
}

p2QuadTree::~p2QuadTree()
{
	Clear();
}

void p2QuadTree::Clear()
{
	m_Objects.clear();
	for (int i = 0; i < CHILD_TREE_NMB; i++)
	{
		if(nodes[i] != nullptr)
		{
			nodes[i]->Clear();
			delete nodes[i];
			nodes[i] = nullptr;
		}
	}
}

void p2QuadTree::Split()
{
	int nodeLevel = m_NodeLevel + 1;
	p2AABB aabb;
	aabb.topRight = p2Vec2(m_Bounds.GetCenter().x, m_Bounds.topRight.y);
	aabb.bottomLeft = p2Vec2(m_Bounds.bottomLeft.x, m_Bounds.GetCenter().y);
	nodes[0] = new p2QuadTree(nodeLevel, aabb);
	aabb.topRight = m_Bounds.topRight;
	aabb.bottomLeft = m_Bounds.GetCenter();
	nodes[1] = new p2QuadTree(nodeLevel, aabb);
	aabb.topRight = m_Bounds.GetCenter();
	aabb.bottomLeft = m_Bounds.bottomLeft;
	nodes[2] = new p2QuadTree(nodeLevel, aabb);
	aabb.topRight = p2Vec2(m_Bounds.topRight.x, m_Bounds.GetCenter().y);
	aabb.bottomLeft = p2Vec2(m_Bounds.GetCenter().x, m_Bounds.bottomLeft.y);
	nodes[3] = new p2QuadTree(nodeLevel, aabb);
	
}

int p2QuadTree::GetLevel()
{
	return m_NodeLevel;
}

p2AABB p2QuadTree::GetBounds()
{
	return m_Bounds;
}

p2QuadTree** p2QuadTree::GetChild()
{
	return nodes;
}

int p2QuadTree::GetIndex(p2AABB aabb)
{
	int index = -1;
	p2Vec2 center = m_Bounds.GetCenter();

	bool leftPart = (aabb.topRight.x < center.x);
	bool rightPart = (aabb.bottomLeft.x > center.x);
	bool topPart = (aabb.bottomLeft.y > center.y);
	bool bottomPart = (aabb.topRight.y < center.y);

	if(rightPart)
	{
		if (topPart)
		{
			index = 1;
		} else if (bottomPart)
		{
			index = 3;
		}
	} else if (leftPart)
	{
		if (topPart)
		{
			index = 0;
		}
		else if (bottomPart)
		{
			index = 2;
		}
	}

	return index;
}

void p2QuadTree::Insert(p2Body * obj)
{
	if (nodes[0] != nullptr)
	{
		int index = GetIndex(obj->GetAABB());

		if (index != -1)
		{
			nodes[index]->Insert(obj);
			return;
		}
	}

	m_Objects.push_back(obj);

	if(m_Objects.size() > MAX_OBJECTS && m_NodeLevel < MAX_LEVELS)
	{
		if (nodes[0] == nullptr)
		{
			Split();
		}
		int i = 0;
		std::list<p2Body*> objectToDelete;
		for (p2Body* object : m_Objects)
		{
			int index = GetIndex(object->GetAABB());
			if (index !=-1)
			{
				nodes[index]->Insert(object);
				objectToDelete.push_back(object);
			}
		}
		for (p2Body* object_to_delete : objectToDelete)
		{
			m_Objects.remove(object_to_delete);
		}
	}
}

std::list<p2Body*> p2QuadTree::GetObjects()
{
	return m_Objects;
}

std::list<p2Body*> p2QuadTree::GetChildObjects()
{
	std::list<p2Body*> childObject;
	for (p2Body* m_object : m_Objects)
	{
		childObject.push_back(m_object);
	}
	if (nodes[0] != nullptr)
	{
		for (p2QuadTree* node : nodes)
		{
			for (p2Body* child_object : node->GetChildObjects())
			{
				childObject.push_back(child_object);
			}
		}
	}
	return childObject;
}

void p2QuadTree::Retrieve(std::list<p2Body*>& returnedObject, p2Body* object)
{
	int index = GetIndex(object->GetAABB());

		if (index != -1 && nodes[0] != nullptr)
		{
			nodes[index]->Retrieve(returnedObject, object);
		}

	for (p2Body* m_object : m_Objects)
	{
		returnedObject.push_back(m_object);
	}
}
