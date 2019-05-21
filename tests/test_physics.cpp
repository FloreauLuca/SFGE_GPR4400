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
	circleColliderJson["sensor"] = false;

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
	rectColliderJson["sensor"] = false;

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
			//"nothing"
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

TEST(Physics, TestShapeContact)
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
			{"sensor", true}
		},
		{
			{"name", "Circle Collider"},
			{"type", sfge::ComponentType::COLLIDER2D},
			{"collider_type", sfge::ColliderType::CIRCLE},
			{"radius", 100},
			{"sensor", true}
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
			{"body_type", p2BodyType::DYNAMIC},
			{"velocity", {rand() % 400, rand() % 400}}
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

			}
		}
	);
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

	const int entitiesNmb = 50;
	json entities[entitiesNmb];

	for (int i = 0; i < entitiesNmb; i++)
	{
		int sizeX = (rand() % 100) + 10;
		int sizeY = (rand() % 100) + 10;
		int radius = (rand() % 75) + 10;
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
				{"sensor", true}
			},
			{
				{"name", "Circle Collider"},
				{"type", sfge::ComponentType::COLLIDER2D},
				{"collider_type", sfge::ColliderType::CIRCLE},
				{"radius", radius},
				{"sensor", true}
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
			{"velocity", {rand() % 400, rand() % 400}}
		};

		int randShapeIndex = rand() % 2;
		entityJson["components"] = { transformJson, shapes[randShapeIndex], rigidbody, colliders[randShapeIndex] };
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

				{"systemClassName", "SatTest nothing"}

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
	transformJson1["position"] = { 700, 300 };
	transformJson1["scale"] = { 1.0, 1.0 };
	transformJson1["angle"] = 0;

	json rectShapeJson;
	rectShapeJson["name"] = "Circle Shape Component";
	rectShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson["size"] = { 200, 100 };

	json rigidBodyJson1;
	rigidBodyJson1["name"] = "Rigidbody";
	rigidBodyJson1["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson1["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson1["mass"] = 1;

	json rectColliderJson;
	rectColliderJson["name"] = "Circle Collider";
	rectColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson["size"] = { 200, 100 };
	rectColliderJson["bouncing"] = 0.5;
	rectColliderJson["sensor"] = false;

	entityBody1["components"] = { transformJson1, rectShapeJson, rigidBodyJson1, rectColliderJson };

	json entityBody2;
	entityBody2["name"] = "Ground";

	json transformJson2;
	transformJson2["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson2["position"] = { 700, 800 };
	transformJson2["scale"] = { 1.0, 1.0 };
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
	circleColliderJson["sensor"] = false;

	entityBody2["components"] = { transformJson2, circleShapeJson, rigidBodyJson2, circleColliderJson };

	sceneJson["entities"] = { entityBody1, entityBody2 };
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
	sceneJson["systems"] = json::array({ contactDebugSystem, raycastDebugJson, gizmoCollider, aabbTest, quadTest });
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

	const int entitiesNmb = 250;
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
				{"sensor", true}
			},
			{
				{"name", "Circle Collider"},
				{"type", sfge::ComponentType::COLLIDER2D},
				{"collider_type", sfge::ColliderType::CIRCLE},
				{"radius", radius},
				{"sensor", true}
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
			{"body_type", p2BodyType::DYNAMIC},
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


TEST(Physics, TestSATDetect)
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
	circleShapeJson["radius"] = 50;

	json rigidBodyJson1;
	rigidBodyJson1["name"] = "Rigidbody";
	rigidBodyJson1["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson1["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson1["gravity_scale"] = 0.5;

	json circleColliderJson;
	circleColliderJson["name"] = "Circle Collider";
	circleColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	circleColliderJson["collider_type"] = sfge::ColliderType::CIRCLE;
	circleColliderJson["radius"] = 50;
	circleColliderJson["bouncing"] = 0.5;
	circleColliderJson["sensor"] = true;

	entityBody1["components"] = { transformJson1, circleShapeJson, rigidBodyJson1, circleColliderJson };

	json entityBody4;
	entityBody4["name"] = "Body4";

	json transformJson4;
	transformJson4["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson4["position"] = { 500, 400 };
	transformJson4["scale"] = { 1.0, 1.0 };
	transformJson4["angle"] = 0.0;

	json rigidBodyJson4;
	rigidBodyJson4["name"] = "Rigidbody";
	rigidBodyJson4["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson4["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson4["gravity_scale"] = 1;

	entityBody4["components"] = { transformJson4, circleShapeJson, rigidBodyJson4, circleColliderJson };


	json entityBody3;
	entityBody3["name"] = "Body3";

	json transformJson3;
	transformJson3["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson3["position"] = { 200, 200 };
	transformJson3["scale"] = { 1.0, 1.0 };
	transformJson3["angle"] = 45;

	json rectShapeJson2;
	rectShapeJson2["name"] = "Circle Shape Component";
	rectShapeJson2["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson2["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson2["size"] = { 50, 50 };

	json rigidBodyJson3;
	rigidBodyJson3["name"] = "Rigidbody";
	rigidBodyJson3["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson3["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson3["gravity_scale"] = 0.5;

	json rectColliderJson2;
	rectColliderJson2["name"] = "Circle Collider";
	rectColliderJson2["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson2["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson2["size"] = { 50, 50 };
	rectColliderJson2["bouncing"] = 0.5;
	rectColliderJson2["sensor"] = false;

	entityBody3["components"] = { transformJson3, rectShapeJson2, rigidBodyJson3, rectColliderJson2 };


	json entityBody2;
	entityBody2["name"] = "Ground";

	json transformJson2;
	transformJson2["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson2["position"] = { 400, 600 };
	transformJson2["scale"] = { 1.0, 1.0 };
	transformJson2["angle"] = 0.0;

	json rectShapeJson;
	rectShapeJson["name"] = "Rect Shape Component";
	rectShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson["size"] = { 500, 200 };

	json rigidBodyJson2;
	rigidBodyJson2["name"] = "Rigidbody";
	rigidBodyJson2["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson2["body_type"] = p2BodyType::STATIC;

	json rectColliderJson;
	rectColliderJson["name"] = "Rect Collider";
	rectColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson["size"] = { 500, 200 };
	rectColliderJson["sensor"] = false;

	entityBody2["components"] = { transformJson2, rectShapeJson, rigidBodyJson2, rectColliderJson };

	sceneJson["entities"] = { entityBody3, entityBody2 };// , entityBody1};
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
	sceneJson["systems"] = json::array({ contactDebugSystem, raycastDebugJson, gizmoCollider, aabbTest, quadTest, satTest});
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}


