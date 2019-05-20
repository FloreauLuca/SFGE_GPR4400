#include "..\include\p2quadtree.h"
#include "p2contact.h"

p2QuadTree::p2QuadTree()
{
}

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
	if (m_NodeLevel < MAX_LEVELS && m_Objects.size() > MAX_OBJECTS) // && m_Objects.size()>MAX_OBJECTS)
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

		for (int i = 0; i < CHILD_TREE_NMB; i++)
		{
			for (p2Body* object : m_Objects)
			{
				if (object == nullptr) continue;
				if (nodes[i]->GetBounds().ContainsAABB(object->GetAABB()))
				{
					nodes[i]->Insert(object);
					m_ChildObjects.push_back(object);
				}
			}
			nodes[i]->Split();
		}
		for (p2Body* childObject : m_ChildObjects)
		{
			m_Objects.remove(childObject);
		}
	}
	else
	{
		//m_Bounds.Write();
	}
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
	m_Objects.push_back(obj);
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

void p2QuadTree::Retrieve(p2ContactManager* contact_manager)
{
	std::vector<p2Body*> objects;
	for (p2Body* m_object : m_Objects)
	{
		objects.push_back(m_object);
	}

	if (nodes[0] == nullptr)
	{
		if (objects.size() >= 2)
		{
			contact_manager->CheckContactInsideList(objects);
		}
	}
	else
	{
		for (p2QuadTree* node : nodes)
		{
			node->Retrieve(contact_manager);
		}
		std::vector<p2Body*> childObjects;
		for (p2Body* child_object : GetChildObjects())
		{
			childObjects.push_back(child_object);
		}
		if (!childObjects.empty() && !objects.empty())
		{
			contact_manager->CheckContactBetweenList(objects, childObjects);
		}
	}
}
