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

#ifndef SFGE_BODY2D_H
#define SFGE_BODY2D_H

#include <engine/component.h>
#include <engine/transform2d.h>
#include "p2body.h"
#include "p2world.h"

namespace sfge
{

class Body2d: public Offsetable
{
public:
	Body2d();
	Body2d(Transform2d *transform, Vec2f offset);

	p2Vec2 GetLinearVelocity() const;
	void SetLinearVelocity(p2Vec2 velocity);
	void ApplyForce(p2Vec2 force);
	p2BodyType GetType();
	p2Vec2 GetAABBTopRight();
	p2Vec2 GetAABBBottomLeft();
	float GetMass();
	void SetBody(p2Body* body);
	p2Body* GetBody() const;
	p2Body Getp2Body() const;
private:
	p2Body * m_Body = nullptr;
};

class BodyManager;

namespace editor
{
struct Body2dInfo : ComponentInfo
{
	void DrawOnInspector() override;
	void AddVelocity(p2Vec2 velocity);
	std::deque<p2Vec2>& GetVelocities();

	Body2dManager* bodyManager = nullptr;
private:
	std::deque<p2Vec2> m_Velocities;
	const size_t m_VelocitiesMaxSize = 120;
};
}

class Body2dManager : public SingleComponentManager<Body2d, editor::Body2dInfo, ComponentType::BODY2D>
{
public:
	using SingleComponentManager::SingleComponentManager;
	void OnEngineInit() override;
	void OnFixedUpdate() override;
	Body2d* AddComponent(Entity entity) override;
	void CreateComponent(json& componentJson, Entity entity) override;
	void DestroyComponent(Entity entity) override;

	void OnResize(size_t new_size) override;

private:
	Transform2dManager* m_Transform2dManager;
	std::weak_ptr<p2World> m_WorldPtr;
};



}

#endif