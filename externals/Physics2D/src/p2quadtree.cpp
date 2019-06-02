#include "..\include\p2quadtree.h"
#include "p2contact.h"

p2QuadTree::p2QuadTree()
{
}

p2QuadTree::p2QuadTree(int nodeLevel, p2AABB bounds)
{
	m_NodeLevel = nodeLevel;
	m_Bounds = bounds;
	m_Objects.resize(MAX_OBJECTS+1);
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
		if (nodes[i] != nullptr)
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

void p2QuadTree::Insert(p2Body* obj)
{
	if (obj == nullptr) return;
	bool inserted = false;
	if(nodes[0] == nullptr)
	{
		if (m_ObjectIndex>= MAX_OBJECTS)
		{
			Split();
			std::vector<p2Body*> objects = GetObjects();
			m_Objects.clear();
			m_Objects.resize(MAX_OBJECTS);
			m_ObjectIndex = 0;
			for (p2Body* object : objects)
			{
				Insert(object);
			}
		}
	}
	if (nodes[0] != nullptr)
	{
		for (p2QuadTree* node : nodes)
		{
			if (node->GetBounds().ContainsAABB(obj->GetAABB()))
			{
				node->Insert(obj);
				inserted = true;
				break;
			}
		}
	}
	
	if (!inserted)
	{
		if (m_ObjectIndex>=m_Objects.size())
		{
			m_Objects.push_back(obj);
			m_ObjectIndex++;
		}
		else
		{
			m_Objects[m_ObjectIndex] = obj;
			m_ObjectIndex++;
		}
	}
}

std::vector<p2Body*> p2QuadTree::GetObjects()
{
	std::vector<p2Body*> returnedObjects = m_Objects;
	returnedObjects.resize(m_ObjectIndex);
	return returnedObjects;
}

std::vector<p2Body*> p2QuadTree::GetChildObjects()
{
	std::vector<p2Body*> childObjects = GetObjects();
	if (nodes[0] != nullptr)
	{
		for (p2QuadTree* node : nodes)
		{
			std::vector<p2Body*> nodeChildObject = node->GetChildObjects();
			if (!nodeChildObject.empty())
			{
				childObjects.insert(childObjects.end(), nodeChildObject.begin(), nodeChildObject.end());
			}
		}
	}
	return childObjects;
}

void p2QuadTree::Retrieve(std::list<p2Body*>& returnedObject, p2Body* object)
{
 	if (nodes[0] == nullptr)
	{

		if (m_ObjectIndex >= 2)
		{
			contact_manager->CheckContactInsideList(GetObjects());
		}

	for (p2Body* m_object : m_Objects)
	{
		for (p2QuadTree* node : nodes)
		{
			node->Retrieve(contact_manager);
		}
		std::vector<p2Body*> childObjects = GetChildObjects();
		if (!childObjects.empty() && !GetObjects().empty())
		{
			contact_manager->CheckContactBetweenList(GetObjects(), childObjects);
		}
	}
}
