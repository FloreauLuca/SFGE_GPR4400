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

#include <physics/body2d.h>
#include <physics/physics2d.h>
#include <engine/transform2d.h>
#include <engine/entity.h>
#include <imgui.h>
#include <imgui-SFML.h>
#include <engine/engine.h>
namespace sfge
{
Body2d::Body2d() : Offsetable(sf::Vector2f())
{
}

Body2d::Body2d(Transform2d *transform, Vec2f offset) : Offsetable(offset)
{
	
}

p2Vec2 Body2d::GetLinearVelocity() const
{
	if (m_Body != nullptr)
		return m_Body->GetLinearVelocity();
		
	return p2Vec2();
}

void Body2d::SetLinearVelocity(p2Vec2 velocity)
{
	if (m_Body != nullptr)
		m_Body->SetLinearVelocity(velocity);

}

void Body2d::ApplyForce(p2Vec2 force)
{
	if (m_Body != nullptr)
		m_Body->ApplyForceToCenter(force);
}

p2BodyType Body2d::GetType()
{
	if(m_Body != nullptr)
		return m_Body->GetType();
	return p2BodyType::STATIC;
}


float Body2d::GetMass()
{
	if (m_Body)
		return m_Body->GetMass();
	return 0.0f;
}

void Body2d::SetBody(p2Body * body)
{
	m_Body = body;
}

p2Body * Body2d::GetBody() const
{
	return m_Body;
}

void editor::Body2dInfo::DrawOnInspector()
{
	const auto& body = bodyManager->GetComponentRef(m_Entity);
	const auto* b2Body = body.GetBody();
	ImGui::Separator();
	ImGui::Text("Body2d");
	if(b2Body != nullptr)
	{
		switch (b2Body->GetType())
		{
		case p2BodyType::STATIC:
			ImGui::LabelText("Type", "Static");
			break;
		case p2BodyType::DYNAMIC:
			ImGui::LabelText("Type", "Dynamic");
			break;
		case p2BodyType::KINEMATIC:
			ImGui::LabelText("Type", "Kinematic");
			break;
		}
		float velocity[2] =
		{
			meter2pixel(b2Body->GetLinearVelocity().x),
			meter2pixel(b2Body->GetLinearVelocity().y)
		};
		ImGui::InputFloat2("Velocity", velocity);
		if (ImGui::IsItemHovered())
		{
			auto& velocities = m_Velocities;
			std::vector<float> xValues(velocities.size());
			std::vector<float> yValues(velocities.size());
			for (auto vIndex = 0u; vIndex < velocities.size(); vIndex++)
			{
				xValues[vIndex] = velocities[vIndex].x;
				yValues[vIndex] = velocities[vIndex].y;
			}
			//Plot last second velocities
			ImGui::BeginTooltip();
			ImGui::PlotLines("X", &xValues[0], xValues.size(), 0, "", -10.0f, 10.0f, ImVec2(0, 120));
			ImGui::PlotLines("Y", &yValues[0], yValues.size(), 0, "", -10.0f, 10.0f, ImVec2(0, 120));
			ImGui::EndTooltip();
		}
	}
}

void editor::Body2dInfo::AddVelocity(p2Vec2 velocity)
{
	m_Velocities.push_back(velocity);
	if(m_Velocities.size() > m_VelocitiesMaxSize)
	{
		m_Velocities.pop_front();
	}
}

std::deque<p2Vec2>& editor::Body2dInfo::GetVelocities()
{
	return m_Velocities;
}



void Body2dManager::OnEngineInit()
{
	SingleComponentManager::OnEngineInit();
	m_Transform2dManager = m_Engine.GetTransform2dManager();
	m_WorldPtr = m_Engine.GetPhysicsManager()->GetWorld();
}

void Body2dManager::OnFixedUpdate()
{
	for (auto i = 0u; i < m_Components.size(); i++)
	{
		const Entity entity = i + 1;
		if (m_EntityManager->HasComponent(entity, ComponentType::BODY2D) &&
			m_EntityManager->HasComponent(entity, ComponentType::TRANSFORM2D))
		{
			auto & transform = m_Transform2dManager->GetComponentRef(entity);
			auto & body2d = GetComponentRef(entity);
			m_ComponentsInfo[i].AddVelocity(body2d.GetLinearVelocity());
			transform.Position = meter2pixel(body2d.GetBody()->GetPosition()) - static_cast<sf::Vector2f>(body2d.GetOffset());
		}
	}
}

Body2d* Body2dManager::AddComponent(Entity entity)
{
	if (auto world = m_WorldPtr.lock())
	{
		p2BodyDef bodyDef;
		bodyDef.type = p2BodyType::STATIC;

		auto* transform = m_Transform2dManager->GetComponentPtr(entity);
		const auto pos = transform->Position;
		bodyDef.position = pixel2meter(pos);
		bodyDef.mass = 1.0f;

		auto* body = world->CreateBody(&bodyDef);
		m_Components[entity - 1] = Body2d(transform, sf::Vector2f());
		m_Components[entity - 1].SetBody(body);

		auto& componentInfo = m_ComponentsInfo[entity-1];
		componentInfo.bodyManager = this;
		componentInfo.SetEntity(entity);
		componentInfo.name = "Body";

		m_EntityManager->AddComponentType(entity, ComponentType::BODY2D);
		return &m_Components[entity - 1];
	}
	return nullptr;
}

void Body2dManager::CreateComponent(json& componentJson, Entity entity)
{
	//Log::GetInstance()->Msg("Create component Transform");
	if (auto world = m_WorldPtr.lock())
	{
		p2BodyDef bodyDef;
		if (CheckJsonExists(componentJson, "body_type"))
		{
			bodyDef.type = componentJson["body_type"];
		}
		if (CheckJsonNumber(componentJson, "gravity_scale"))
		{
			bodyDef.gravityScale = componentJson["gravity_scale"];
		}

		const auto offset = GetVectorFromJson(componentJson, "offset");
		const auto velocity = GetVectorFromJson(componentJson, "velocity");

		auto* transform = m_Transform2dManager->GetComponentPtr(entity);
		const auto pos = transform->Position + offset;
		bodyDef.position = pixel2meter(pos);
		bodyDef.angle = transform->EulerAngle;
		bodyDef.mass = 1.0f;
		
		auto* body = world->CreateBody(&bodyDef);
		body->SetLinearVelocity(pixel2meter(velocity));
		m_Components[entity - 1] = Body2d(transform, offset);
		m_Components[entity - 1].SetBody(body);


		m_ComponentsInfo[entity - 1].bodyManager = this;
		m_ComponentsInfo[entity - 1].SetEntity(entity);
	}
}

void Body2dManager::DestroyComponent(Entity entity)
{
	(void) entity;
}

void Body2dManager::OnResize(size_t new_size)
{
	m_Components.resize(new_size);
	m_ComponentsInfo.resize(new_size);
}
}

