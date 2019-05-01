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
#include <imgui.h>
#include <physics/collider2d.h>
#include <engine/globals.h>
#include <engine/component.h>
#include <physics/physics2d.h>
#include <engine/engine.h>
namespace sfge
{
void editor::ColliderInfo::DrawOnInspector()
{
  ImGui::Separator();
  ImGui::Text("Collider");
  if(data != nullptr)
  {
    if(data->fixture != nullptr)
    {
		/*
      switch (data->fixture->GetShape()->m_type)
      {
      	case b2Shape::e_circle:
      		ImGui::LabelText("Shape", "Circle");
      		break;
        case b2Shape::e_polygon:
        	ImGui::LabelText("Shape", "Polygon");
        	break;
      	case b2Shape::e_chain:
      		ImGui::LabelText("Shape", "Chain");
			break;
		case b2Shape::e_edge:
			ImGui::LabelText("Shape", "Edge");
		  	break;
		  default:
		  	break;
      }
		*/
    }
  }
}


void ColliderManager::OnEngineInit()
{
	MultipleComponentManager::OnEngineInit();
	m_BodyManager = m_Engine.GetPhysicsManager()->GetBodyManager();
}

void ColliderManager::CreateComponent(json& componentJson, Entity entity)
{
	Log::GetInstance()->Msg("Create component Collider");
	if (m_EntityManager->HasComponent(entity, ComponentType::BODY2D))
	{
		auto & body = m_BodyManager->GetComponentRef(entity);

		p2ColliderDef fixtureDef;

		if (CheckJsonExists(componentJson, "sensor"))
		{
			fixtureDef.isSensor = componentJson["sensor"];
		}

		std::unique_ptr<p2Shape> shape = nullptr;

		if (CheckJsonExists(componentJson, "collider_type"))
		{
			ColliderType colliderType = static_cast<ColliderType>(componentJson["collider_type"]);
			switch (colliderType)
			{
			case ColliderType::CIRCLE:
				fixtureDef.colliderType = p2ColliderType::CIRCLE;
				shape = std::make_unique<p2CircleShape>();
				if (CheckJsonNumber(componentJson, "radius"))
				{
					//shape->m_radius = pixel2meter(static_cast<float>(componentJson["radius"]));
					p2CircleShape circle_shape;
					circle_shape.SetRadius(pixel2meter(static_cast<float>(componentJson["radius"])));
					shape = std::make_unique<p2Shape>(static_cast<p2Shape>(circle_shape));
				}
				break;
			case ColliderType::BOX:
			{
				fixtureDef.colliderType = p2ColliderType::BOX;

				auto boxShape = std::make_unique<p2RectShape>();
				if (CheckJsonExists(componentJson, "size"))
				{
					auto size = pixel2meter(GetVectorFromJson(componentJson, "size"));
					{
						std::ostringstream oss;
						oss << "Box physics size: " << size.x << ", " << size.y;
						Log::GetInstance()->Msg(oss.str());
					}
					//boxShape->SetAsBox(size.x / 2.0f, size.y / 2.0f);
					p2RectShape rect_shape;
					rect_shape.SetSize(p2Vec2(size.x / 2.0f, size.y / 2.0f));
					shape = std::make_unique<p2Shape>(static_cast<p2Shape>(rect_shape));

				}
				shape = std::move(boxShape);
			}	
			break;
			default:
			{
				fixtureDef.colliderType = p2ColliderType::NONE;
				std::ostringstream oss;
				oss << "[Error] Collider of type: " << static_cast<int>(colliderType) << " could not be loaded from json: " << componentJson;
				Log::GetInstance()->Error(oss.str());
			}
				break;
			}
		}
		if(CheckJsonNumber(componentJson, "bouncing"))
		{
			fixtureDef.restitution = componentJson["bouncing"];
		}
		if (shape != nullptr)
		{
			fixtureDef.shape = shape.get();
			auto index = GetFreeComponentIndex();
			if(index != -1)
			{
				auto* fixture = body.GetBody()->CreateCollider(&fixtureDef);


				ColliderData& colliderData = m_Components[index];
				colliderData.entity = entity;
				colliderData.fixture = fixture;
				colliderData.body = body.GetBody();
				m_ComponentsInfo[index].data = &colliderData;
				m_ComponentsInfo[index].SetEntity(entity);
				fixture->SetUserData(&colliderData);
			}
		}
	}
}
int ColliderManager::GetFreeComponentIndex()
{
	for(auto i = 0u; i < m_Components.size();i++)
	{
		if(m_Components[i].entity == INVALID_ENTITY)
			return i;
	}
	return -1;
}
ColliderData *ColliderManager::AddComponent(Entity entity)
{
	(void) entity;
	return nullptr;
}
void ColliderManager::DestroyComponent(Entity entity)
{
	(void) entity;
}
ColliderData *ColliderManager::GetComponentPtr(Entity entity)
{
	(void)entity;
	return nullptr;
}
}
