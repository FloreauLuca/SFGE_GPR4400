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

#include <graphics/graphics2d.h>
#include <graphics/shape2d.h>
#include <utility/json_utility.h>
#include <utility/log.h>
#include <engine/transform2d.h>
#include <engine/engine.h>
#include <imgui.h>
#include <imgui-SFML.h>

namespace sfge
{

Shape::Shape (): Shape (nullptr, sf::Vector2f())
{

}
Shape::Shape(Transform2d * transform, const sf::Vector2f offset) : 
	Offsetable(offset)
{
	
}

Shape::~Shape()
{
	m_Shape = nullptr;
}

void Shape::Draw(sf::RenderWindow& window) const
{
	if (m_Shape != nullptr)
	{
		window.draw(*m_Shape);
	}
}

void Shape::SetFillColor(sf::Color color) const
{
	if (m_Shape)
		m_Shape->setFillColor(color);
}

void Shape::Update() const
{
	if(m_Shape != nullptr)
	{
		m_Shape->setPosition(transform.Position + m_Offset);
		m_Shape->setRotation(transform.EulerAngle);
		m_Shape->setScale(transform.Scale);
	}
}
void Shape::SetShape (std::unique_ptr<sf::Shape> shape)
{
	m_Shape = std::move(shape);
}
sf::Shape *Shape::GetShape ()
{
	return m_Shape.get ();
}

void editor::ShapeInfo::DrawOnInspector ()
{
	auto* shapePtr = shapeManager->GetComponentPtr(m_Entity);
	if(shapePtr != nullptr && shapePtr->GetShape() != nullptr)
	{
		ImGui::Separator();
		ImGui::Text("Shape");

		float offset[2] =
		{
			shapePtr->GetOffset().x,
			shapePtr->GetOffset().y
		};

		ImGui::InputFloat2("Offset", offset);
		auto circleShape = dynamic_cast<sf::CircleShape*>(shapePtr->GetShape());
		if(circleShape != nullptr)
		{
			float radius = circleShape->getRadius ();
			ImGui::InputFloat ("Radius", &radius);
		}

		auto rectShape = dynamic_cast<sf::RectangleShape*>(shapePtr->GetShape ());
		if(rectShape != nullptr)
		{
			float size[2] =
			{
				rectShape->getSize ().x,
				rectShape->getSize ().y
			};
			ImGui::InputFloat2("Size", size);
		}
	}
}

void ShapeManager::OnEngineInit()
{
	SingleComponentManager::OnEngineInit();
	m_Transform2dManager = m_Engine.GetTransform2dManager();
}


void ShapeManager::DrawShapes(sf::RenderWindow &window)
{

	rmt_ScopedCPUSample(ShapeDraw,0)
	for(auto i = 0u; i < m_Components.size(); i++)
	{
		if(m_EntityManager->HasComponent(i + 1, ComponentType::SHAPE2D))
		{
			m_Components[i].Draw(window);
		}
	}
}

void ShapeManager::OnUpdate(const float dt)
{

	(void)dt;
	rmt_ScopedCPUSample(ShapeUpdate, 0);
	auto* transformManager = m_Engine.GetTransform2dManager();
	for (auto i = 0u; i < m_Components.size(); i++)
	{
		if (m_EntityManager->HasComponent(i + 1, ComponentType::SHAPE2D))
		{
			if(m_EntityManager->HasComponent(i+1, ComponentType::TRANSFORM2D))
			{
				m_Components[i].transform = transformManager->GetComponentRef(i + 1);
			}
			m_Components[i].Update();
		}
	}
	
}

void ShapeManager::OnBeforeSceneLoad()
{
	m_Components.clear ();
	m_Components.resize (INIT_ENTITY_NMB);

}



Shape *ShapeManager::AddComponent (Entity entity)
{
	auto shapePtr = GetComponentPtr (entity);
	auto& shapeInfo = GetComponentInfo(entity);
	shapeInfo.SetEntity(entity);
	shapeInfo.shapeManager = this;

	float radius = 50.0f;
	
	auto circleShape = std::make_unique <sf::CircleShape>();
	circleShape->setRadius(radius);
	circleShape->setOrigin(radius, radius);
	shapePtr->SetShape(std::move(circleShape));
	shapePtr->Update();

	m_Engine.GetEntityManager()->AddComponentType(entity, ComponentType::SHAPE2D);
	return shapePtr;
}

void ShapeManager::CreateComponent(json& componentJson, Entity entity)
{
	//Log::GetInstance()->Msg("Create component Shape");
	sf::Vector2f offset;
	if (CheckJsonExists(componentJson, "offset"))
	{
		offset = GetVectorFromJson(componentJson, "offset");
	}

	auto& shape = m_Components[entity-1];
	shape.SetOffset(offset);

	auto& shapeInfo = m_ComponentsInfo[entity - 1];
	shapeInfo.shapeManager = this;
	shapeInfo.SetEntity(entity);

	if (CheckJsonNumber(componentJson, "shape_type"))
	{
		const ShapeType shapeType = componentJson["shape_type"];
		switch (shapeType)
		{
		case ShapeType::CIRCLE:
		{

			float radius = 10.0f;
			if (CheckJsonNumber(componentJson, "radius"))
			{
				radius = componentJson["radius"];
			}

			auto circleShape = std::make_unique <sf::CircleShape>();
			circleShape->setRadius (radius);
			circleShape->setOrigin (radius, radius);
			shape.SetShape (std::move(circleShape));
			shape.Update ();
		}
			break;
		case ShapeType::RECTANGLE:
		{
			sf::Vector2f offset;
			if (CheckJsonExists(componentJson, "offset"))
			{
				offset = GetVectorFromJson(componentJson, "offset");
			}
			sf::Vector2f size = sf::Vector2f();
			if (CheckJsonExists(componentJson, "size"))
			{
				size = GetVectorFromJson(componentJson, "size");
			}
			auto rect = std::make_unique<sf::RectangleShape>();
			rect->setSize (size);
			rect->setOrigin (size.x/2.0f, size.y/2.0f);
            shape.SetShape (std::move (rect));
            shape.Update ();
			
		}
			break;
		default:
			Log::GetInstance()->Error("Invalid shape type in ShapeManager Component Creation");
			break;
		}
	}
	else
	{
		std::ostringstream oss;
		oss << "[Error] No shape_type defined in json:  "<<componentJson;
		Log::GetInstance()->Error(oss.str());
	}
	
	
}

void ShapeManager::DestroyComponent(Entity entity)
{
	(void) entity;
}

void ShapeManager::OnResize(size_t new_size)
{
	m_Components.resize(new_size);
	m_ComponentsInfo.resize(new_size);
}

}


