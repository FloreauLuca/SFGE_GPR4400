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
#include <engine/engine.h>
#include <engine/scene.h>
#include <gtest/gtest.h>
#include "graphics/shape2d.h"
#include "physics/collider2d.h"

#pragma region Test

TEST(Physics, TestBallFallingToGround)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->devMode = false;
	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Ball Falling To Ground";

	json entityBody1;
	entityBody1["name"] = "Body1";

	json transformJson1;
	transformJson1["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson1["position"] = {300, 300};
	transformJson1["scale"] = {1.0, 1.0};
	transformJson1["angle"] = 0.0;

	json circleShapeJson;
	circleShapeJson["name"] = "Circle Shape Component";
	circleShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	circleShapeJson["shape_type"] = sfge::ShapeType::CIRCLE;
	circleShapeJson["radius"] = 50;

	json rigidBodyJson1;
	rigidBodyJson1["name"] = "Rigidbody";
	rigidBodyJson1["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson1["body_type"] = p2BodyType::DYNAMIC;

	json circleColliderJson;
	circleColliderJson["name"] = "Circle Collider";
	circleColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	circleColliderJson["collider_type"] = sfge::ColliderType::CIRCLE;
	circleColliderJson["radius"] = 50;
	circleColliderJson["bouncing"] = 0.5;

	entityBody1["components"] = {transformJson1, circleShapeJson, rigidBodyJson1, circleColliderJson};

	json entityBody2;
	entityBody2["name"] = "Ground";

	json transformJson2;
	transformJson2["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson2["position"] = {400, 600};
	transformJson2["scale"] = {1.0, 1.0};
	transformJson2["angle"] = 0.0;

	json rectShapeJson;
	rectShapeJson["name"] = "Rect Shape Component";
	rectShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson["size"] = {800, 200};

	json rigidBodyJson2;
	rigidBodyJson2["name"] = "Rigidbody";
	rigidBodyJson2["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson2["body_type"] = p2BodyType::STATIC;

	json rectColliderJson;
	rectColliderJson["name"] = "Rect Collider";
	rectColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson["size"] = {800, 200};

	entityBody2["components"] = {transformJson2, rectShapeJson, rigidBodyJson2, rectColliderJson};

	sceneJson["entities"] = {entityBody1, entityBody2};
	json contactDebugSystem = {
		{
			"script_path",
			"scripts/contact_debug_system.py"
			//"nothing"
		}
	};
	json raycastDebugJson =
	{
		{
			"script_path",
			//"scripts/mouse_raycast_system.py" 
			"nothing"
		}
	};
	json gizmoCollider =
	{
		{
			"script_path",
			//"scripts/gizmo_collider.py"
			"nothing"
		}
	};
	json aabbTest =
	{

		{"systemClassName", "AabbTest"}

	};
	sceneJson["systems"] = json::array({contactDebugSystem, raycastDebugJson, gizmoCollider});
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}


TEST(Physics, TestAABB)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->gravity = p2Vec2(0.0f, 0.0f);
	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Contacts";

	const int entitiesNmb = 5;
	json entities[entitiesNmb];

	for (int i = 0; i < entitiesNmb; i++)
	{
		int sizeX = (rand() % 100) + 50;
		int sizeY = (rand() % 100) + 50;
		int radius = (rand() % 75) + 50;
		json shapes[] =
		{
			{
				{"name", "Rect Shape Component"},
				{"type", sfge::ComponentType::SHAPE2D},
				{"shape_type", sfge::ShapeType::RECTANGLE},
				{"size", {sizeX, sizeY}}
			},
			{
				{"name", "Rect Shape Component"},
				{"type", sfge::ComponentType::SHAPE2D},
				{"shape_type", sfge::ShapeType::CIRCLE},
				{"radius", radius}
			}
		};
		json colliders[] =
		{
			{
				{"name", "Rect Collider"},
				{"type", sfge::ComponentType::COLLIDER2D},
				{"collider_type", sfge::ColliderType::BOX},
				{"size", {sizeX, sizeY}},
			},
			{
				{"name", "Circle Collider"},
				{"type", sfge::ComponentType::COLLIDER2D},
				{"collider_type", sfge::ColliderType::CIRCLE},
				{"radius", radius},
			}
		};

		json& entityJson = entities[i];

		json transformJson =
		{
			{"position", {rand() % 800, rand() % 600}},
			{"type", sfge::ComponentType::TRANSFORM2D},
			{"angle", rand() % 360}
		};

		json rigidbody =
		{
			{"name", "Rigidbody"},
			{"type", sfge::ComponentType::BODY2D},
			{"body_type", p2BodyType::DYNAMIC},
			{"velocity", {rand() % 200, rand() % 200}}
		};

		int randShapeIndex = rand() % 2;
		entityJson["components"] = {transformJson, shapes[randShapeIndex], rigidbody, colliders[randShapeIndex]};
	}
	sceneJson["entities"] = entities;
	sceneJson["systems"] = json::array({
			{
				{
					"script_path",
					"scripts/contact_debug_system.py"
					//"nothing"
				}
			},
			{
				{"script_path", "scripts/stay_onscreen_system.py"}
			},
			{
				{
					"script_path",
					//"scripts/mouse_raycast_system.py" 
					"nothing"
				}
			},
			{

				{"systemClassName", "AabbTest"}

			},
			{

				{"systemClassName", "SatTest"}

			}
		}
	);
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}

TEST(Physics, TestAABBRotation)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->devMode = false;
	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Ball Falling To Ground";

	json entityBody1;
	entityBody1["name"] = "Body1";

	json transformJson1;
	transformJson1["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson1["position"] = {700, 300};
	transformJson1["scale"] = {1.0, 1.0};
	transformJson1["angle"] = 0;

	json rectShapeJson;
	rectShapeJson["name"] = "Circle Shape Component";
	rectShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson["shape_type"] = sfge::ShapeType::CIRCLE;
	rectShapeJson["radius"] = 100;

	json rigidBodyJson1;
	rigidBodyJson1["name"] = "Rigidbody";
	rigidBodyJson1["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson1["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson1["mass"] = 1;

	json rectColliderJson;
	rectColliderJson["name"] = "Circle Collider";
	rectColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson["collider_type"] = sfge::ColliderType::CIRCLE;
	rectColliderJson["radius"] = 100;
	rectColliderJson["bouncing"] = 0.5;

	entityBody1["components"] = {transformJson1, rectShapeJson, rigidBodyJson1, rectColliderJson};

	json entityBody2;
	entityBody2["name"] = "Ground";

	json transformJson2;
	transformJson2["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson2["position"] = {700, 800};
	transformJson2["scale"] = {1.0, 1.0};
	transformJson2["angle"] = 0.0;

	json circleShapeJson;
	circleShapeJson["name"] = "Rect Shape Component";
	circleShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	circleShapeJson["shape_type"] = sfge::ShapeType::CIRCLE;
	circleShapeJson["radius"] = 250;

	json rigidBodyJson2;
	rigidBodyJson2["name"] = "Rigidbody";
	rigidBodyJson2["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson2["body_type"] = p2BodyType::STATIC;

	json circleColliderJson;
	circleColliderJson["name"] = "Rect Collider";
	circleColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	circleColliderJson["collider_type"] = sfge::ColliderType::CIRCLE;
	circleColliderJson["radius"] = 250;

	entityBody2["components"] = {transformJson2, circleShapeJson, rigidBodyJson2, circleColliderJson};

	sceneJson["entities"] = {entityBody1, entityBody2};
	json contactDebugSystem = {
		{
			"script_path",
			"scripts/contact_debug_system.py"
			//"nothing"
		}
	};
	json raycastDebugJson =
	{
		{
			"script_path",
			//"scripts/mouse_raycast_system.py" 
			"nothing"
		}
	};
	json gizmoCollider =
	{
		{
			"script_path",
			//"scripts/gizmo_collider.py"
			"nothing"
		}
	};
	json aabbTest =
	{

		{"systemClassName", "AabbTest"}

	};
	json quadTest =
	{

		{"systemClassName", "QuadTreeTest"}

	};
	json satTest =
	{

		{"systemClassName", "SatTest"}

	};
	sceneJson["systems"] = json::array({contactDebugSystem, raycastDebugJson, gizmoCollider, aabbTest, quadTest, satTest});
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}

TEST(Physics, TestQuadTree)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->gravity = p2Vec2(0.0f, 0.0f);
	config->devMode = false;

	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Contacts";

	const int entitiesNmb = 1000;
	json entities[entitiesNmb];

	for (int i = 0; i < entitiesNmb; i++)
	{
		int sizeX = (rand() % 10) + 5;
		int sizeY = (rand() % 10) + 5;
		int radius = (rand() % 10) + 5;
		json shapes[] =
		{
			{
				{"name", "Rect Shape Component"},
				{"type", sfge::ComponentType::SHAPE2D},
				{"shape_type", sfge::ShapeType::RECTANGLE},
				{"size", {sizeX, sizeY}}
			},
			{
				{"name", "Rect Shape Component"},
				{"type", sfge::ComponentType::SHAPE2D},
				{"shape_type", sfge::ShapeType::CIRCLE},
				{"radius", radius}
			}
		};
		json colliders[] =
		{
			{
				{"name", "Rect Collider"},
				{"type", sfge::ComponentType::COLLIDER2D},
				{"collider_type", sfge::ColliderType::BOX},
				{"size", {sizeX, sizeY}},
			},
			{
				{"name", "Circle Collider"},
				{"type", sfge::ComponentType::COLLIDER2D},
				{"collider_type", sfge::ColliderType::CIRCLE},
				{"radius", radius},
			}
		};

		json& entityJson = entities[i];

		json transformJson =
		{
			{"position", {rand() % 800, rand() % 600}},
			{"type", sfge::ComponentType::TRANSFORM2D}
		};

		json rigidbody =
		{
			{"name", "Rigidbody"},
			{"type", sfge::ComponentType::BODY2D},
			{"body_type", p2BodyType::KINEMATIC},
			{"velocity", {rand() % 100, rand() % 100}}
		};

		int randShapeIndex = rand() % 2;
		entityJson["components"] = {transformJson, shapes[randShapeIndex], rigidbody, colliders[randShapeIndex]};
	}
	sceneJson["entities"] = entities;
	sceneJson["systems"] = json::array({
			{
				{"systemClassName", "ContactDebug"}
			},
			{
				{"systemClassName", "StayOnScreen"}
			},
			{
				{
					"script_path",
					//"scripts/mouse_raycast_system.py" 
					"nothing"
				}
			},
			{

				{"systemClassName", "AabbTest"}

			},
			{

				{"systemClassName", "QuadTreeTest"}

			}
		}
	);
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}


TEST(Physics, TestSatDetect)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->devMode = false;
	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Ball Falling To Ground";

	json entityBody1;
	entityBody1["name"] = "Body1";

	json transformJson1;
	transformJson1["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson1["position"] = {500, 200};
	transformJson1["scale"] = {1.0, 1.0};
	transformJson1["angle"] = 0.0;

	json circleShapeJson;
	circleShapeJson["name"] = "Circle Shape Component";
	circleShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	circleShapeJson["shape_type"] = sfge::ShapeType::CIRCLE;
	circleShapeJson["radius"] = 50;

	json rigidBodyJson1;
	rigidBodyJson1["name"] = "Rigidbody";
	rigidBodyJson1["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson1["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson1["gravity_scale"] = 1;

	json circleColliderJson;
	circleColliderJson["name"] = "Circle Collider";
	circleColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	circleColliderJson["collider_type"] = sfge::ColliderType::CIRCLE;
	circleColliderJson["radius"] = 50;
	circleColliderJson["bouncing"] = 0.5;

	entityBody1["components"] = {transformJson1, circleShapeJson, rigidBodyJson1, circleColliderJson};

	json entityBody4;
	entityBody4["name"] = "Body4";

	json transformJson4;
	transformJson4["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson4["position"] = {200, 200};
	transformJson4["scale"] = {1.0, 1.0};
	transformJson4["angle"] = 0.0;

	json rigidBodyJson4;
	rigidBodyJson4["name"] = "Rigidbody";
	rigidBodyJson4["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson4["body_type"] = p2BodyType::STATIC;
	rigidBodyJson4["gravity_scale"] = 1;

	json circleColliderJson2;
	circleColliderJson2["name"] = "Circle Collider";
	circleColliderJson2["type"] = sfge::ComponentType::COLLIDER2D;
	circleColliderJson2["collider_type"] = sfge::ColliderType::CIRCLE;
	circleColliderJson2["radius"] = 50;
	circleColliderJson2["bouncing"] = 1;

	json circleShapeJson2;
	circleShapeJson2["name"] = "Circle Shape Component";
	circleShapeJson2["type"] = sfge::ComponentType::SHAPE2D;
	circleShapeJson2["shape_type"] = sfge::ShapeType::CIRCLE;
	circleShapeJson2["radius"] = 50;

	entityBody4["components"] = {transformJson4, circleShapeJson2, rigidBodyJson4, circleColliderJson2};


	json entityBody3;
	entityBody3["name"] = "Body3";

	json transformJson3;
	transformJson3["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson3["position"] = {200, 200};
	transformJson3["scale"] = {1.0, 1.0};
	transformJson3["angle"] = 45;

	json rectShapeJson2;
	rectShapeJson2["name"] = "Circle Shape Component";
	rectShapeJson2["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson2["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson2["size"] = {100, 100};

	json rigidBodyJson3;
	rigidBodyJson3["name"] = "Rigidbody";
	rigidBodyJson3["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson3["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson3["gravity_scale"] = 1;

	json rectColliderJson2;
	rectColliderJson2["name"] = "Circle Collider";
	rectColliderJson2["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson2["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson2["size"] = {100, 100};
	rectColliderJson2["bouncing"] = 1;

	entityBody3["components"] = {transformJson3, rectShapeJson2, rigidBodyJson3, rectColliderJson2};


	json entityBody2;
	entityBody2["name"] = "Ground";

	json transformJson2;
	transformJson2["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson2["position"] = {200, 500};
	transformJson2["scale"] = {1.0, 1.0};
	transformJson2["angle"] = 0.0;

	json rectShapeJson;
	rectShapeJson["name"] = "Rect Shape Component";
	rectShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson["size"] = {100, 100};

	json rigidBodyJson2;
	rigidBodyJson2["name"] = "Rigidbody";
	rigidBodyJson2["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson2["body_type"] = p2BodyType::STATIC;

	json rectColliderJson;
	rectColliderJson["name"] = "Rect Collider";
	rectColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson["size"] = {100, 100};

	entityBody2["components"] = {transformJson2, rectShapeJson, rigidBodyJson2, rectColliderJson};

	sceneJson["entities"] = {entityBody4, entityBody1, entityBody2}; // , entityBody1};
	json contactDebugSystem = {

		{"systemClassName", "ContactDebug"}

	};
	json raycastDebugJson =
	{
		{
			"script_path",
			//"scripts/mouse_raycast_system.py" 
			"nothing"
		}
	};
	json gizmoCollider =
	{
		{
			"script_path",
			//"scripts/gizmo_collider.py"
			//"nothing"
		}
	};
	json aabbTest =
	{

		{"systemClassName", "AabbTest"}

	};
	json quadTest =
	{

		{"systemClassName", "QuadTreeTest"}

	};
	json satTest =
	{

		{"systemClassName", "SatTest"}

	};
	json mouseController =
	{

		{"systemClassName", "MouseController"}

	};
	sceneJson["systems"] = json::array({contactDebugSystem, raycastDebugJson, gizmoCollider, aabbTest, quadTest, satTest, mouseController});
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}

TEST(Physics, TestFlipper)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->devMode = false;
	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Ball Falling To Ground";

	json entityBody1;
	entityBody1["name"] = "Body1";

	json transformJson1;
	transformJson1["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson1["position"] = {450, 200};
	transformJson1["scale"] = {1.0, 1.0};
	transformJson1["angle"] = 0.0;

	json circleShapeJson;
	circleShapeJson["name"] = "Circle Shape Component";
	circleShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	circleShapeJson["shape_type"] = sfge::ShapeType::CIRCLE;
	circleShapeJson["radius"] = 50;

	json rigidBodyJson1;
	rigidBodyJson1["name"] = "Rigidbody";
	rigidBodyJson1["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson1["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson1["gravity_scale"] = 1;

	json circleColliderJson;
	circleColliderJson["name"] = "Circle Collider";
	circleColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	circleColliderJson["collider_type"] = sfge::ColliderType::CIRCLE;
	circleColliderJson["radius"] = 50;
	circleColliderJson["bouncing"] = 1;

	entityBody1["components"] = {transformJson1, circleShapeJson, rigidBodyJson1, circleColliderJson};


	json entityBody2;
	entityBody2["name"] = "Bounce1";

	json transformJson2;
	transformJson2["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson2["position"] = {400, 400};
	transformJson2["scale"] = {1.0, 1.0};
	transformJson2["angle"] = 45.0;

	json rectShapeJson;
	rectShapeJson["name"] = "Rect Shape Component";
	rectShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson["size"] = {200, 50};

	json rigidBodyJson2;
	rigidBodyJson2["name"] = "Rigidbody";
	rigidBodyJson2["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson2["body_type"] = p2BodyType::STATIC;

	json rectColliderJson;
	rectColliderJson["name"] = "Rect Collider";
	rectColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson["size"] = {200, 50};

	entityBody2["components"] = {transformJson2, rectShapeJson, rigidBodyJson2, rectColliderJson};

	json entityBody3;
	entityBody3["name"] = "Bounce2";

	json transformJson3;
	transformJson3["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson3["position"] = {800, 400};
	transformJson3["scale"] = {1.0, 1.0};
	transformJson3["angle"] = -45.0;

	json rectShapeJson2;
	rectShapeJson2["name"] = "Rect Shape Component";
	rectShapeJson2["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson2["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson2["size"] = {200, 50};

	json rigidBodyJson3;
	rigidBodyJson3["name"] = "Rigidbody";
	rigidBodyJson3["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson3["body_type"] = p2BodyType::STATIC;

	json rectColliderJson2;
	rectColliderJson2["name"] = "Rect Collider";
	rectColliderJson2["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson2["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson2["size"] = {200, 50};

	entityBody3["components"] = {transformJson3, rectShapeJson2, rigidBodyJson3, rectColliderJson2};

	sceneJson["entities"] = {entityBody1, entityBody2, entityBody3}; // , entityBody1};
	json contactDebugSystem = {

		{"systemClassName", "ContactDebug"}

	};
	json raycastDebugJson =
	{
		{
			"script_path",
			//"scripts/mouse_raycast_system.py" 
			"nothing"
		}
	};
	json gizmoCollider =
	{
		{
			"script_path",
			//"scripts/gizmo_collider.py"
			//"nothing"
		}
	};
	json aabbTest =
	{

		{"systemClassName", "AabbTest"}

	};
	json quadTest =
	{

		{"systemClassName", "QuadTreeTest"}

	};
	json satTest =
	{

		{"systemClassName", "SatTest"}

	};
	json mouseController =
	{

		{"systemClassName", "MouseController"}

	};
	sceneJson["systems"] = json::array({contactDebugSystem, raycastDebugJson, gizmoCollider, aabbTest, quadTest, satTest, mouseController});
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}

#pragma endregion Test

#pragma region Presentation

TEST(Presentation, TestPlanet)
{
	sfge::Engine engine;
	std::unique_ptr<sfge::Configuration> initConfig = std::make_unique<sfge::Configuration>();
	initConfig->gravity = p2Vec2();
	initConfig->devMode = false;
	initConfig->maxFramerate = 0;
	engine.Init(std::move(initConfig));
	json sceneJson = {
		{"name", "Test Planet Component"}
	};
	json systemJson = {
		{"systemClassName", "PlanetSystem"}
	};
	json quadTest =
	{

		{"systemClassName", "QuadTreeTest"}

	};

	sceneJson["systems"] = json::array({ systemJson, quadTest });
	auto* sceneManager = engine.GetSceneManager();
	sceneManager->LoadSceneFromJson(sceneJson);

	engine.Start();
}

TEST(Presentation, TestSatDetect)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->devMode = false;
	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Ball Falling To Ground";

	json entityBody1;
	entityBody1["name"] = "Body1";

	json transformJson1;
	transformJson1["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson1["position"] = { 500, 200 };
	transformJson1["scale"] = { 1.0, 1.0 };
	transformJson1["angle"] = 0.0;

	json circleShapeJson;
	circleShapeJson["name"] = "Circle Shape Component";
	circleShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	circleShapeJson["shape_type"] = sfge::ShapeType::CIRCLE;
	circleShapeJson["radius"] = 25;

	json rigidBodyJson1;
	rigidBodyJson1["name"] = "Rigidbody";
	rigidBodyJson1["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson1["body_type"] = p2BodyType::KINEMATIC;
	rigidBodyJson1["gravity_scale"] = 1;

	json circleColliderJson;
	circleColliderJson["name"] = "Circle Collider";
	circleColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	circleColliderJson["collider_type"] = sfge::ColliderType::CIRCLE;
	circleColliderJson["radius"] = 25;
	circleColliderJson["bouncing"] = 0.5;

	entityBody1["components"] = { transformJson1, circleShapeJson, rigidBodyJson1, circleColliderJson };

	json entityBody4;
	entityBody4["name"] = "Body4";

	json transformJson4;
	transformJson4["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson4["position"] = { 200, 200 };
	transformJson4["scale"] = { 1.0, 1.0 };
	transformJson4["angle"] = 0.0;

	json rigidBodyJson4;
	rigidBodyJson4["name"] = "Rigidbody";
	rigidBodyJson4["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson4["body_type"] = p2BodyType::KINEMATIC;
	rigidBodyJson4["gravity_scale"] = 1;

	json circleColliderJson2;
	circleColliderJson2["name"] = "Circle Collider";
	circleColliderJson2["type"] = sfge::ComponentType::COLLIDER2D;
	circleColliderJson2["collider_type"] = sfge::ColliderType::CIRCLE;
	circleColliderJson2["radius"] = 50;
	circleColliderJson2["bouncing"] = 1;

	json circleShapeJson2;
	circleShapeJson2["name"] = "Circle Shape Component";
	circleShapeJson2["type"] = sfge::ComponentType::SHAPE2D;
	circleShapeJson2["shape_type"] = sfge::ShapeType::CIRCLE;
	circleShapeJson2["radius"] = 50;

	entityBody4["components"] = { transformJson4, circleShapeJson2, rigidBodyJson4, circleColliderJson2 };


	json entityBody3;
	entityBody3["name"] = "Body3";

	json transformJson3;
	transformJson3["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson3["position"] = { 700, 200 };
	transformJson3["scale"] = { 1.0, 1.0 };
	transformJson3["angle"] = 45;

	json rectShapeJson2;
	rectShapeJson2["name"] = "Circle Shape Component";
	rectShapeJson2["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson2["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson2["size"] = { 100, 100 };

	json rigidBodyJson3;
	rigidBodyJson3["name"] = "Rigidbody";
	rigidBodyJson3["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson3["body_type"] = p2BodyType::KINEMATIC;
	rigidBodyJson3["gravity_scale"] = 1;

	json rectColliderJson2;
	rectColliderJson2["name"] = "Circle Collider";
	rectColliderJson2["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson2["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson2["size"] = { 100, 100 };
	rectColliderJson2["bouncing"] = 1;

	entityBody3["components"] = { transformJson3, rectShapeJson2, rigidBodyJson3, rectColliderJson2 };


	json entityBody2;
	entityBody2["name"] = "Ground";

	json transformJson2;
	transformJson2["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson2["position"] = { 700, 500 };
	transformJson2["scale"] = { 1.0, 1.0 };
	transformJson2["angle"] = 0.0;

	json rectShapeJson;
	rectShapeJson["name"] = "Rect Shape Component";
	rectShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson["size"] = { 500, 100 };

	json rigidBodyJson2;
	rigidBodyJson2["name"] = "Rigidbody";
	rigidBodyJson2["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson2["body_type"] = p2BodyType::KINEMATIC;

	json rectColliderJson;
	rectColliderJson["name"] = "Rect Collider";
	rectColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson["size"] = { 500, 100 };

	entityBody2["components"] = { transformJson2, rectShapeJson, rigidBodyJson2, rectColliderJson };

	sceneJson["entities"] = { entityBody1, entityBody2, entityBody3, entityBody4 }; // , entityBody1};
	json contactDebugSystem = {

		{"systemClassName", "ContactDebug"}

	};
	json raycastDebugJson =
	{
		{
			"script_path",
			//"scripts/mouse_raycast_system.py" 
			"nothing"
		}
	};
	json gizmoCollider =
	{
		{
			"script_path",
			//"scripts/gizmo_collider.py"
			//"nothing"
		}
	};
	json aabbTest =
	{

		{"systemClassName", "AabbTest"}

	};
	json quadTest =
	{

		{"systemClassName", "QuadTreeTest"}

	};
	json satTest =
	{

		{"systemClassName", "SatTest"}

	};
	json mouseController =
	{

		{"systemClassName", "MouseController"}

	};
	sceneJson["systems"] = json::array({ contactDebugSystem, raycastDebugJson, gizmoCollider, aabbTest, quadTest, satTest, mouseController });
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}

TEST(Presentation, TestShapesContact)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->gravity = p2Vec2(0.0f, 0.0f);
	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Contacts";

	const int entitiesNmb = 10;
	json entities[entitiesNmb];

	json shapes[] =
	{
		{
			{"name", "Rect Shape Component"},
			{"type", sfge::ComponentType::SHAPE2D},
			{"shape_type", sfge::ShapeType::RECTANGLE},
			{"size", {100, 100}}
		},
		{
			{"name", "Rect Shape Component"},
			{"type", sfge::ComponentType::SHAPE2D},
			{"shape_type", sfge::ShapeType::CIRCLE},
			{"radius", 100}
		}
	};
	json colliders[] =
	{
		{
			{"name", "Rect Collider"},
			{"type", sfge::ComponentType::COLLIDER2D},
			{"collider_type", sfge::ColliderType::BOX},
			{"size", {100, 100}},
		},
		{
			{"name", "Circle Collider"},
			{"type", sfge::ComponentType::COLLIDER2D},
			{"collider_type", sfge::ColliderType::CIRCLE},
			{"radius", 100},
		}
	};

	for (int i = 0; i < entitiesNmb; i++)
	{
		json& entityJson = entities[i];

		json transformJson =
		{
			{"position", {rand() % 800, rand() % 600}},
			{"type", sfge::ComponentType::TRANSFORM2D}
		};

		json rigidbody =
		{
			{"name", "Rigidbody"},
			{"type", sfge::ComponentType::BODY2D},
			{"body_type", p2BodyType::KINEMATIC},
			{"velocity", {rand() % 400, rand() % 400}}
		};

		int randShapeIndex = rand() % 2;
		entityJson["components"] = { transformJson, shapes[randShapeIndex], rigidbody, colliders[randShapeIndex] };
	}
	sceneJson["entities"] = entities;
	sceneJson["systems"] = json::array({
			{

				{"systemClassName", "ContactDebug"}
			},
			{
				{"script_path", "scripts/stay_onscreen_system.py"}
			},
			{

				{"systemClassName", "AabbTest"}

			},
			{

				{"systemClassName", "QuadTreeTest"}

			}
		}
	);
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}

TEST(Presentation, TestQuadTree)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->gravity = p2Vec2(0.0f, 0.0f);
	config->devMode = false;

	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Contacts";

	const int entitiesNmb = 1000;
	json entities[entitiesNmb];

	for (int i = 0; i < entitiesNmb; i++)
	{
		int sizeX = (rand() % 10) + 5;
		int sizeY = (rand() % 10) + 5;
		int radius = (rand() % 10) + 5;
		json shapes[] =
		{
			{
				{"name", "Rect Shape Component"},
				{"type", sfge::ComponentType::SHAPE2D},
				{"shape_type", sfge::ShapeType::RECTANGLE},
				{"size", {sizeX, sizeY}}
			},
			{
				{"name", "Rect Shape Component"},
				{"type", sfge::ComponentType::SHAPE2D},
				{"shape_type", sfge::ShapeType::CIRCLE},
				{"radius", radius}
			}
		};
		json colliders[] =
		{
			{
				{"name", "Rect Collider"},
				{"type", sfge::ComponentType::COLLIDER2D},
				{"collider_type", sfge::ColliderType::BOX},
				{"size", {sizeX, sizeY}},
			},
			{
				{"name", "Circle Collider"},
				{"type", sfge::ComponentType::COLLIDER2D},
				{"collider_type", sfge::ColliderType::CIRCLE},
				{"radius", radius},
			}
		};

		json& entityJson = entities[i];

		json transformJson =
		{
			{"position", {rand() % 1200, rand() % 700}},
			{"type", sfge::ComponentType::TRANSFORM2D}
		};

		json rigidbody =
		{
			{"name", "Rigidbody"},
			{"type", sfge::ComponentType::BODY2D},
			{"body_type", p2BodyType::KINEMATIC},
			{"velocity", {rand() % 100, rand() % 100}}
		};

		int randShapeIndex = rand() % 2;
		entityJson["components"] = { transformJson, shapes[randShapeIndex], rigidbody, colliders[randShapeIndex] };
	}
	sceneJson["entities"] = entities;
	sceneJson["systems"] = json::array({
			{
				{"systemClassName", "ContactDebug"}
			},
			{
				{"systemClassName", "StayOnScreen"}
			},
			{
				{
					"script_path",
					//"scripts/mouse_raycast_system.py" 
					"nothing"
				}
			},
			{

				{"systemClassName", "AabbTest"}

			},
			{

				{"systemClassName", "QuadTreeTest"}

			}
		}
	);
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}

TEST(Presentation, TestBounciness)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->devMode = false;
	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Ball Falling To Ground";


	const int entitiesNmb = 7;
	json entities[entitiesNmb];


	json entityBody3;
	entityBody3["name"] = "Body3";

	json transformJson3;
	transformJson3["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson3["position"] = {200, 200};
	transformJson3["scale"] = {1.0, 1.0};
	transformJson3["angle"] = 0;

	json rectShapeJson2;
	rectShapeJson2["name"] = "Rect Shape Component";
	rectShapeJson2["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson2["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson2["size"] = {100, 100};

	json rigidBodyJson3;
	rigidBodyJson3["name"] = "Rigidbody";
	rigidBodyJson3["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson3["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson3["gravity_scale"] = 1;

	json rectColliderJson2;
	rectColliderJson2["name"] = "Rect Collider";
	rectColliderJson2["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson2["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson2["size"] = {100, 100};
	rectColliderJson2["bouncing"] = 1;

	entityBody3["components"] = {transformJson3, rectShapeJson2, rigidBodyJson3, rectColliderJson2};
	entities[0] = entityBody3;


	json entityBody2;
	entityBody2["name"] = "Ground";

	json transformJson2;
	transformJson2["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson2["position"] = {500, 700};
	transformJson2["scale"] = {1.0, 1.0};
	transformJson2["angle"] = 0.0;

	json rectShapeJson;
	rectShapeJson["name"] = "Rect Shape Component";
	rectShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson["size"] = {2000, 200};

	json rigidBodyJson2;
	rigidBodyJson2["name"] = "Rigidbody";
	rigidBodyJson2["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson2["body_type"] = p2BodyType::STATIC;
	rigidBodyJson2["mass"] = 1000;

	json rectColliderJson;
	rectColliderJson["name"] = "Rect Collider";
	rectColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson["size"] = { 2000, 200 };
	rectColliderJson["bouncing"] = 1.0;

	entityBody2["components"] = {transformJson2, rectShapeJson, rigidBodyJson2, rectColliderJson};

	entities[1] = entityBody2;

	for (int i = 2; i < entitiesNmb; i++)
	{
		json& entityJson = entities[i];

		float bouncing = 1 - (i - 2.0f) / (entitiesNmb - 3);
		json circleShapeJson;
		circleShapeJson["name"] = "Circle Shape Component";
		circleShapeJson["type"] = sfge::ComponentType::SHAPE2D;
		circleShapeJson["shape_type"] = sfge::ShapeType::CIRCLE;
		circleShapeJson["radius"] = 50;

		json circleColliderJson;
		circleColliderJson["name"] = "Circle Collider";
		circleColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
		circleColliderJson["collider_type"] = sfge::ColliderType::CIRCLE;
		circleColliderJson["radius"] = 50;
		circleColliderJson["bouncing"] = bouncing;

		json transformJson1;
		transformJson1["type"] = sfge::ComponentType::TRANSFORM2D;
		transformJson1["position"] = {150 * i + 100, 200};
		transformJson1["scale"] = {1.0, 1.0};
		transformJson1["angle"] = 0.0;

		json rigidBodyJson1;
		rigidBodyJson1["name"] = "Rigidbody";
		rigidBodyJson1["type"] = sfge::ComponentType::BODY2D;
		rigidBodyJson1["body_type"] = p2BodyType::DYNAMIC;
		rigidBodyJson1["gravity_scale"] = 1;

		int randShapeIndex = rand() % 2;
		entityJson["components"] = {transformJson1, circleShapeJson, rigidBodyJson1, circleColliderJson};
	}

	sceneJson["entities"] = entities;
	json contactDebugSystem = {

		{"systemClassName", "ContactDebug"}

	};
	json sat =
	{

		{"systemClassName", "SatTest"}

	};
	json mouseController =
	{

		{"systemClassName", "MouseController"}

	};
	sceneJson["systems"] = json::array({contactDebugSystem, sat, mouseController});
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}

TEST(Presentation, TestRestitution)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->devMode = false;
	config->gravity = p2Vec2(0, 0);
	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Ball Falling To Ground";


	json entities[8];


	json entityBody1;
	entityBody1["name"] = "Wall1";

	json transformJson1;
	transformJson1["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson1["position"] = { 100, 400 };
	transformJson1["scale"] = { 1.0, 1.0 };
	transformJson1["angle"] = 0;

	json rectShapeJson1;
	rectShapeJson1["name"] = "Rect Shape Component";
	rectShapeJson1["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson1["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson1["size"] = { 200, 2000 };

	json rigidBodyJson1;
	rigidBodyJson1["name"] = "Rigidbody";
	rigidBodyJson1["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson1["body_type"] = p2BodyType::STATIC;
	rigidBodyJson1["gravity_scale"] = 1;
	rigidBodyJson1["mass"] = 1000;

	json rectColliderJson1;
	rectColliderJson1["name"] = "Rect Collider";
	rectColliderJson1["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson1["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson1["size"] = { 200, 2000 };
	rectColliderJson1["bouncing"] = 1;

	entityBody1["components"] = { transformJson1};
	entities[0] = entityBody1;

	json transformJson2;
	transformJson2["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson2["position"] = { 1250, 400 };
	transformJson2["scale"] = { 1.0, 1.0 };
	transformJson2["angle"] = 0;

	json entityBody2;
	entityBody2["name"] = "Wall2";

	entityBody2["components"] = { transformJson2};
	entities[1] = entityBody2;

	   
	json entityBody3;
	entityBody3["name"] = "Starting Rect1";

	json rectShapeJson;
	rectShapeJson["name"] = "Rect Shape Component";
	rectShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson["size"] = { 100, 100 };

	json rectColliderJson;
	rectColliderJson["name"] = "Rect Collider";
	rectColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson["size"] = { 100, 100 };
	rectColliderJson["bouncing"] = 1;

	json transformJson3;
	transformJson3["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson3["position"] = { 300, 200 };
	transformJson3["scale"] = { 1.0, 1.0 };
	transformJson3["angle"] = 0.0;

	json rigidBodyJson3;
	rigidBodyJson3["name"] = "Rigidbody";
	rigidBodyJson3["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson3["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson3["gravity_scale"] = 1;
	rigidBodyJson3["velocity"] = { 200, 0 };

	entityBody3["components"] = { transformJson3, rectShapeJson, rigidBodyJson3, rectColliderJson };
	entities[2] = entityBody3;


	json entityBody4;
	entityBody4["name"] = "Rect1";

	
	json transformJson4;
	transformJson4["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson4["position"] = { 600, 200 };
	transformJson4["scale"] = { 1.0, 1.0 };
	transformJson4["angle"] = 0.0;

	json rigidBodyJson4;
	rigidBodyJson4["name"] = "Rigidbody";
	rigidBodyJson4["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson4["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson4["gravity_scale"] = 1;

	entityBody4["components"] = { transformJson4, rectShapeJson, rigidBodyJson4, rectColliderJson };
	entities[3] = entityBody4;

	entityBody3["name"] = "Starting Rect0.5";

	rectShapeJson["name"] = "Rect Shape Component";
	rectShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson["size"] = { 100, 100 };

	rectColliderJson["name"] = "Rect Collider";
	rectColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson["size"] = { 100, 100 };
	rectColliderJson["bouncing"] = 0.5;

	transformJson3["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson3["position"] = { 300, 400 };
	transformJson3["scale"] = { 1.0, 1.0 };
	transformJson3["angle"] = 0.0;

	rigidBodyJson3["name"] = "Rigidbody";
	rigidBodyJson3["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson3["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson3["gravity_scale"] = 1;
	rigidBodyJson3["velocity"] = { 200, 0 };

	entityBody3["components"] = { transformJson3, rectShapeJson, rigidBodyJson3, rectColliderJson };
	entities[4] = entityBody3;


	entityBody4["name"] = "Rect1";


	transformJson4["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson4["position"] = { 600, 400 };
	transformJson4["scale"] = { 1.0, 1.0 };
	transformJson4["angle"] = 0.0;

	rigidBodyJson4["name"] = "Rigidbody";
	rigidBodyJson4["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson4["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson4["gravity_scale"] = 1;
	
	entityBody4["components"] = { transformJson4, rectShapeJson, rigidBodyJson4, rectColliderJson };
	entities[5] = entityBody4;

	entityBody3["name"] = "Starting Rect0";

	rectShapeJson["name"] = "Rect Shape Component";
	rectShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson["size"] = { 100, 100 };

	rectColliderJson["name"] = "Circle Collider";
	rectColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson["size"] = { 100, 100 };
	rectColliderJson["bouncing"] = 0;

	transformJson3["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson3["position"] = { 300, 600 };
	transformJson3["scale"] = { 1.0, 1.0 };
	transformJson3["angle"] = 0.0;

	rigidBodyJson3["name"] = "Rigidbody";
	rigidBodyJson3["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson3["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson3["gravity_scale"] = 1;
	rigidBodyJson3["velocity"] = { 200, 0 };

	entityBody3["components"] = { transformJson3, rectShapeJson, rigidBodyJson3, rectColliderJson };
	entities[6] = entityBody3;


	entityBody4["name"] = "Rect0";


	transformJson4["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson4["position"] = { 600, 600 };
	transformJson4["scale"] = { 1.0, 1.0 };
	transformJson4["angle"] = 0.0;

	rigidBodyJson4["name"] = "Rigidbody";
	rigidBodyJson4["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson4["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson4["gravity_scale"] = 1;

	entityBody4["components"] = { transformJson4, rectShapeJson, rigidBodyJson4, rectColliderJson };
	entities[7] = entityBody4;

	sceneJson["entities"] = entities;
	json contactDebugSystem = {

		{"systemClassName", "ContactDebug"}

	};
	json mouseController =
	{

		{"systemClassName", "MouseController"}

	};
	json satTest =
	{

		{"systemClassName", "SatTest"}

	};
	sceneJson["systems"] = json::array({mouseController, satTest });
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}

TEST(Presentation, TestPendule)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->devMode = false;
	config->gravity = p2Vec2(0, 0);
	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Ball Falling To Ground";


	const int entitiesNmb = 5;
	json entities[entitiesNmb * 2 + 4];


	json entityBody1;
	entityBody1["name"] = "Wall1";

	json transformJson1;
	transformJson1["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson1["position"] = { 100, 400 };
	transformJson1["scale"] = { 1.0, 1.0 };
	transformJson1["angle"] = 0;

	json rectShapeJson1;
	rectShapeJson1["name"] = "Rect Shape Component";
	rectShapeJson1["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson1["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson1["size"] = { 200, 2000 };

	json rigidBodyJson1;
	rigidBodyJson1["name"] = "Rigidbody";
	rigidBodyJson1["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson1["body_type"] = p2BodyType::STATIC;
	rigidBodyJson1["gravity_scale"] = 1;
	rigidBodyJson1["mass"] = 1000;

	json rectColliderJson1;
	rectColliderJson1["name"] = "Rect Collider";
	rectColliderJson1["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson1["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson1["size"] = { 200, 2000 };
	rectColliderJson1["bouncing"] = 1;

	entityBody1["components"] = { transformJson1, rectShapeJson1, rigidBodyJson1, rectColliderJson1 };
	entities[0] = entityBody1;

	json transformJson2;
	transformJson2["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson2["position"] = { 1250, 400 };
	transformJson2["scale"] = { 1.0, 1.0 };
	transformJson2["angle"] = 0;

	json entityBody2;
	entityBody2["name"] = "Wall2";

	entityBody2["components"] = { transformJson2, rectShapeJson1, rigidBodyJson1, rectColliderJson1 };
	entities[1] = entityBody2;
	
	for (int i = 0; i < entitiesNmb; i++)
	{

		json& entityJson = entities[i + 4];

		json circleShapeJson;
		circleShapeJson["name"] = "Circle Shape Component";
		circleShapeJson["type"] = sfge::ComponentType::SHAPE2D;
		circleShapeJson["shape_type"] = sfge::ShapeType::CIRCLE;
		circleShapeJson["radius"] = 50;

		json circleColliderJson;
		circleColliderJson["name"] = "Circle Collider";
		circleColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
		circleColliderJson["collider_type"] = sfge::ColliderType::CIRCLE;
		circleColliderJson["radius"] = 50;
		circleColliderJson["bouncing"] = 1;

		json transformJson3;
		transformJson3["type"] = sfge::ComponentType::TRANSFORM2D;
		transformJson3["position"] = { 110 * i + 300, 200 };
		transformJson3["scale"] = { 1.0, 1.0 };
		transformJson3["angle"] = 0.0;

		json rigidBodyJson3;
		rigidBodyJson3["name"] = "Rigidbody";
		rigidBodyJson3["type"] = sfge::ComponentType::BODY2D;
		rigidBodyJson3["body_type"] = p2BodyType::DYNAMIC;
		rigidBodyJson3["gravity_scale"] = 1;

		entityJson["components"] = { transformJson3, circleShapeJson, rigidBodyJson3, circleColliderJson };
	}


	json entityBody3;
	entityBody3["name"] = "Starting Ball";

	json circleShapeJson;
	circleShapeJson["name"] = "Circle Shape Component";
	circleShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	circleShapeJson["shape_type"] = sfge::ShapeType::CIRCLE;
	circleShapeJson["radius"] = 50;

	json circleColliderJson;
	circleColliderJson["name"] = "Circle Collider";
	circleColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	circleColliderJson["collider_type"] = sfge::ColliderType::CIRCLE;
	circleColliderJson["radius"] = 50;
	circleColliderJson["bouncing"] = 1;

	json transformJson3;
	transformJson3["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson3["position"] = { 1000 , 200 };
	transformJson3["scale"] = { 1.0, 1.0 };
	transformJson3["angle"] = 0.0;

	json rigidBodyJson3;
	rigidBodyJson3["name"] = "Rigidbody";
	rigidBodyJson3["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson3["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson3["gravity_scale"] = 1;
	rigidBodyJson3["velocity"] = {200, 0};

	entityBody3["components"] = { transformJson3, circleShapeJson, rigidBodyJson3, circleColliderJson };
	entities[2] = entityBody3;


	for (int i = 0; i < entitiesNmb; i++)
	{

		json& entityJson = entities[i + 4 + entitiesNmb];

		json rectShapeJson;
		rectShapeJson["name"] = "Rect Shape Component";
		rectShapeJson["type"] = sfge::ComponentType::SHAPE2D;
		rectShapeJson["shape_type"] = sfge::ShapeType::RECTANGLE;
		rectShapeJson["size"] = { 100, 100 };

		json rectColliderJson;
		rectColliderJson["name"] = "Circle Collider";
		rectColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
		rectColliderJson["collider_type"] = sfge::ColliderType::BOX;
		rectColliderJson["size"] = { 100, 100 };
		rectColliderJson["bouncing"] = 1;

		json transformJson4;
		transformJson4["type"] = sfge::ComponentType::TRANSFORM2D;
		transformJson4["position"] = { 110 * i + 300, 500 };
		transformJson4["scale"] = { 1.0, 1.0 };
		transformJson4["angle"] = 0.0;

		json rigidBodyJson4;
		rigidBodyJson4["name"] = "Rigidbody";
		rigidBodyJson4["type"] = sfge::ComponentType::BODY2D;
		rigidBodyJson4["body_type"] = p2BodyType::DYNAMIC;
		rigidBodyJson4["gravity_scale"] = 1;

		entityJson["components"] = { transformJson4, rectShapeJson, rigidBodyJson4, rectColliderJson };
	}


	json entityBody4;
	entityBody4["name"] = "Starting Ball";

	json rectShapeJson;
	rectShapeJson["name"] = "Rect Shape Component";
	rectShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson["size"] = { 100, 100 };

	json rectColliderJson;
	rectColliderJson["name"] = "Circle Collider";
	rectColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson["size"] = { 100, 100 };
	rectColliderJson["bouncing"] = 1;

	json transformJson4;
	transformJson4["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson4["position"] = { 1000, 500 };
	transformJson4["scale"] = { 1.0, 1.0 };
	transformJson4["angle"] = 0.0;

	json rigidBodyJson4;
	rigidBodyJson4["name"] = "Rigidbody";
	rigidBodyJson4["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson4["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson4["gravity_scale"] = 1;
	rigidBodyJson4["velocity"] = { 200, 0 };

	entityBody4["components"] = { transformJson4, rectShapeJson, rigidBodyJson4, rectColliderJson };
	entities[3] = entityBody4;

	sceneJson["entities"] = entities;
	json contactDebugSystem = {

		{"systemClassName", "ContactDebug"}

	};
	json mouseController =
	{

		{"systemClassName", "MouseController"}

	};
	json satTest =
	{

		{"systemClassName", "SatTest"}

	};
	sceneJson["systems"] = json::array({ contactDebugSystem, mouseController, satTest });
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}

TEST(Presentation, TestFlipper)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->devMode = false;
	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Ball Falling To Ground";


	const int entitiesNmb = 10;
	json entities[10];


	json entityBody1;
	entityBody1["name"] = "Wall1";

	json transformJson1;
	transformJson1["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson1["position"] = { 100, 400 };
	transformJson1["scale"] = { 1.0, 1.0 };
	transformJson1["angle"] = 0;

	json rectShapeJson1;
	rectShapeJson1["name"] = "Rect Shape Component";
	rectShapeJson1["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson1["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson1["size"] = { 200, 2000 };

	json rigidBodyJson1;
	rigidBodyJson1["name"] = "Rigidbody";
	rigidBodyJson1["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson1["body_type"] = p2BodyType::STATIC;
	rigidBodyJson1["gravity_scale"] = 1;
	rigidBodyJson1["mass"] = 1000;

	json rectColliderJson1;
	rectColliderJson1["name"] = "Rect Collider";
	rectColliderJson1["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson1["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson1["size"] = { 200, 2000 };
	rectColliderJson1["bouncing"] = 1;

	entityBody1["components"] = { transformJson1, rectShapeJson1, rigidBodyJson1, rectColliderJson1 };
	entities[0] = entityBody1;

	json entityBody2;
	entityBody2["name"] = "Wall2";

	json transformJson2;
	transformJson2["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson2["position"] = { 1200, 400 };
	transformJson2["scale"] = { 1.0, 1.0 };
	transformJson2["angle"] = 0;

	entityBody2["components"] = { transformJson2, rectShapeJson1, rigidBodyJson1, rectColliderJson1 };
	entities[1] = entityBody2;

	json entityBody3;
	entityBody3["name"] = "Wall3";

	json transformJson3;
	transformJson3["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson3["position"] = { 1200, 750 };
	transformJson3["scale"] = { 0.0, 0.0 };
	transformJson3["angle"] = 45;

	entityBody3["components"] = { transformJson3};
	entities[2] = entityBody3;

	json entityBody4;
	entityBody4["name"] = "Wall4";

	json transformJson4;
	transformJson4["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson4["position"] = { 100, 750 };
	transformJson4["scale"] = { 0.0, 0.0 };
	transformJson4["angle"] = -45;

	entityBody4["components"] = { transformJson4};
	entities[3] = entityBody4;

	
	json entityBody5;
	entityBody5["name"] = "Ball";

	json circleShapeJson;
	circleShapeJson["name"] = "Circle Shape Component";
	circleShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	circleShapeJson["shape_type"] = sfge::ShapeType::CIRCLE;
	circleShapeJson["radius"] = 50;

	json circleColliderJson;
	circleColliderJson["name"] = "Circle Collider";
	circleColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	circleColliderJson["collider_type"] = sfge::ColliderType::CIRCLE;
	circleColliderJson["radius"] = 50;
	circleColliderJson["bouncing"] = 0.75;

	json transformJson5;
	transformJson5["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson5["position"] = { rand() % 500 +400 , 0 };
	transformJson5["scale"] = { 1.0, 1.0 };
	transformJson5["angle"] = 0.0;

	json rigidBodyJson3;
	rigidBodyJson3["name"] = "Rigidbody";
	rigidBodyJson3["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson3["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson3["gravity_scale"] = 1;
	rigidBodyJson3["velocity"] = { 0, 0 };

	entityBody5["components"] = { transformJson5, circleShapeJson, rigidBodyJson3, circleColliderJson };
	entities[4] = entityBody5;


	json entityBody6;
	entityBody6["name"] = "Bounce1";

	json rectShapeJson2;
	rectShapeJson2["name"] = "Rect Shape Component";
	rectShapeJson2["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson2["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson2["size"] = { 100, 100 };

	json rectColliderJson2;
	rectColliderJson2["name"] = "Circle Collider";
	rectColliderJson2["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson2["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson2["size"] = { 100, 100 };
	rectColliderJson2["bouncing"] = 1;

	json transformJson6;
	transformJson6["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson6["position"] = { 450, 200 };
	transformJson6["scale"] = { 1.0, 1.0 };
	transformJson6["angle"] = 45;

	json rigidBodyJson4;
	rigidBodyJson4["name"] = "Rigidbody";
	rigidBodyJson4["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson4["body_type"] = p2BodyType::STATIC;
	rigidBodyJson4["gravity_scale"] = 1;

	entityBody6["components"] = { transformJson6, rectShapeJson2, rigidBodyJson4, rectColliderJson2 };
	entities[5] = entityBody6;

	json entityBody7;
	entityBody7["name"] = "Bounce2";

	json transformJson7;
	transformJson7["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson7["position"] = { 850, 200 };
	transformJson7["scale"] = { 1.0, 1.0 };
	transformJson7["angle"] = 45;

	entityBody7["components"] = { transformJson7, rectShapeJson2, rigidBodyJson4, rectColliderJson2 };
	entities[6] = entityBody7;

	json entityBody8;
	entityBody8["name"] = "Bounce3";

	json transformJson8;
	transformJson8["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson8["position"] = { 450, 450 };
	transformJson8["scale"] = { 1.0, 1.0 };
	transformJson8["angle"] = 45;

	entityBody8["components"] = { transformJson8, rectShapeJson2, rigidBodyJson4, rectColliderJson2 };
	entities[7] = entityBody8;

	json entityBody9;
	entityBody9["name"] = "Bounce4";

	json transformJson9;
	transformJson9["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson9["position"] = { 850, 450 };
	transformJson9["scale"] = { 1.0, 1.0 };
	transformJson9["angle"] = 45;

	entityBody9["components"] = { transformJson9, rectShapeJson2, rigidBodyJson4, rectColliderJson2 };
	entities[8] = entityBody9;

	json entityBody10;
	entityBody10["name"] = "Bounce5";

	json circleShapeJson2;
	circleShapeJson2["name"] = "Circle Shape Component";
	circleShapeJson2["type"] = sfge::ComponentType::SHAPE2D;
	circleShapeJson2["shape_type"] = sfge::ShapeType::CIRCLE;
	circleShapeJson2["radius"] = 50;

	json circleColliderJson2;
	circleColliderJson2["name"] = "Circle Collider";
	circleColliderJson2["type"] = sfge::ComponentType::COLLIDER2D;
	circleColliderJson2["collider_type"] = sfge::ColliderType::CIRCLE;
	circleColliderJson2["radius"] =  50 ;
	circleColliderJson2["bouncing"] = 1;

	json transformJson10;
	transformJson10["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson10["position"] = { 650, 300 };
	transformJson10["scale"] = { 1.0, 1.0 };
	transformJson10["angle"] = 0.0;

	json rigidBodyJson5;
	rigidBodyJson5["name"] = "Rigidbody";
	rigidBodyJson5["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson5["body_type"] = p2BodyType::STATIC;
	rigidBodyJson5["gravity_scale"] = 1;

	entityBody10["components"] = { transformJson10, circleShapeJson2, rigidBodyJson5, circleColliderJson2 };
	entities[9] = entityBody10;

	sceneJson["entities"] = entities;
	json contactDebugSystem = {

		{"systemClassName", "ContactDebug"}

	};
	json satTest =
	{

		{"systemClassName", "SatTest"}

	};
	json flipper =
	{

		{"systemClassName", "FlipperScript"}

	};
	json mouseController =
	{

		{"systemClassName", "MouseController"}

	};
	sceneJson["systems"] = json::array({ contactDebugSystem, flipper, mouseController, satTest });
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}

TEST(Presentation, TestGaz)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->gravity = p2Vec2(0.0f, 0.0f);
	config->devMode = false;

	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Contacts";

	const int entitiesNmb = 50;
	json entities[entitiesNmb];

	for (int i = 0; i < entitiesNmb; i++)
	{
		json shape =
		{
				{"name", "Rect Shape Component"},
				{"type", sfge::ComponentType::SHAPE2D},
				{"shape_type", sfge::ShapeType::CIRCLE},
				{"radius", 25}
		};
		json collider =
		{
				{"name", "Circle Collider"},
				{"type", sfge::ComponentType::COLLIDER2D},
				{"collider_type", sfge::ColliderType::CIRCLE},
				{"radius", 25},
		};

		json& entityJson = entities[i];

		json transformJson =
		{
			{"position", {rand() % 500 + 250, rand() % 500}},
			{"type", sfge::ComponentType::TRANSFORM2D}
		};

		json rigidbody =
		{
			{"name", "Rigidbody"},
			{"type", sfge::ComponentType::BODY2D},
			{"body_type", p2BodyType::DYNAMIC},
			{"velocity", {rand() % 500, rand() % 500}}
		};

		int randShapeIndex = rand() % 2;
		entityJson["components"] = { transformJson, shape, rigidbody, collider };
	}
	sceneJson["entities"] = entities;
	sceneJson["systems"] = json::array({
			{
				{"systemClassName", "ContactDebug"}
			},
			{
				{"systemClassName", "StayOnScreen"}
			},
			{
				{
					"script_path",
					//"scripts/mouse_raycast_system.py" 
					"nothing"
				}
			},
			{

				{"systemClassName", "AabbTest"}

			},
			{

				{"systemClassName", "QuadTreeTest"}

			}
		}
	);
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}

TEST(Presentation, TestExplo)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->devMode = false;
	config->gravity = p2Vec2();

	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Contacts";

	const int entitiesNmb = 50;
	json entities[entitiesNmb + 3 ];

	for (int i = 3; i < entitiesNmb; i++)
	{
		json shape =
		{
				{"name", "Rect Shape Component"},
				{"type", sfge::ComponentType::SHAPE2D},
				{"shape_type", sfge::ShapeType::CIRCLE},
				{"radius", 25}
		};
		json collider =
		{
				{"name", "Circle Collider"},
				{"type", sfge::ComponentType::COLLIDER2D},
				{"collider_type", sfge::ColliderType::CIRCLE},
				{"radius", 25},
				{"bouncing", 0.5}
		};

		json& entityJson = entities[i];

		json transformJson =
		{
			{"position", {rand() % 600 + 300, rand() % 600}},
			{"type", sfge::ComponentType::TRANSFORM2D}
		};

		json rigidbody =
		{
			{"name", "Rigidbody"},
			{"type", sfge::ComponentType::BODY2D},
			{"body_type", p2BodyType::DYNAMIC}
		};

		entityJson["components"] = { transformJson, shape, rigidbody, collider };
	}
	json entityBody5;
	entityBody5["name"] = "Bombe";

	json circleShapeJson;
	circleShapeJson["name"] = "Circle Shape Component";
	circleShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	circleShapeJson["shape_type"] = sfge::ShapeType::CIRCLE;
	circleShapeJson["radius"] = 50;

	json bombSpriteJson;
	bombSpriteJson["name"] = "Bomb Sprite Component";
	bombSpriteJson["type"] = sfge::ComponentType::SPRITE2D;
	bombSpriteJson["path"] = "data/sprites/bomb.png";

	json circleColliderJson;
	circleColliderJson["name"] = "Circle Collider";
	circleColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	circleColliderJson["collider_type"] = sfge::ColliderType::CIRCLE;
	circleColliderJson["radius"] = 50;
	circleColliderJson["bouncing"] = 1;

	json transformJson5;
	transformJson5["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson5["position"] = { 650, 300 };
	transformJson5["scale"] = { 5.0, 5.0 };
	transformJson5["angle"] = 0.0;

	json rigidBodyJson3;
	rigidBodyJson3["name"] = "Rigidbody";
	rigidBodyJson3["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson3["body_type"] = p2BodyType::STATIC;
	rigidBodyJson3["gravity_scale"] = 1;
	rigidBodyJson3["velocity"] = { 0, 0 };

	entityBody5["components"] = { transformJson5, bombSpriteJson, rigidBodyJson3, circleColliderJson };
	entities[0] = entityBody5;

	json entityBody2;
	entityBody2["name"] = "Ground";

	json transformJson2;
	transformJson2["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson2["position"] = { 500, 700 };
	transformJson2["scale"] = { 1.0, 1.0 };
	transformJson2["angle"] = 0.0;

	json rectShapeJson;
	rectShapeJson["name"] = "Rect Shape Component";
	rectShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson["size"] = { 2000, 200 };

	json rigidBodyJson2;
	rigidBodyJson2["name"] = "Rigidbody";
	rigidBodyJson2["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson2["body_type"] = p2BodyType::STATIC;
	rigidBodyJson2["mass"] = 1000;

	json rectColliderJson;
	rectColliderJson["name"] = "Rect Collider";
	rectColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson["size"] = { 2000, 200 };
	rectColliderJson["bouncing"] = 1.0;

	entityBody2["components"] = { transformJson2, rectShapeJson, rigidBodyJson2, rectColliderJson };

	//entities[1] = entityBody2;

	sceneJson["entities"] = entities;
	sceneJson["systems"] = json::array({
			{
				{"systemClassName", "ContactDebug"}
			},
			{
				{"systemClassName", "Explosion"}
			},
			{
				{"systemClassName", "StayOnScreen"}
			},
		}
	);
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}

#pragma endregion Presentation

