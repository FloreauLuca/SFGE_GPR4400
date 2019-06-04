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

#include <engine/system.h>
#include <engine/engine.h>

#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <extensions/python_extensions.h>
#include <extensions/flipper_script.h>
#include <extensions/pong_script.h>
#include <extensions/explosion.h>
#include <extensions/planet_system.h>
#include "extensions/aabb_test.h"
#include "extensions/quad_tree_test.h"
#include "extensions/sat_test.h"
#include "extensions/quad_tree_test.h"
#include "extensions/contact_debug.h"
#include "extensions/stay_on_screen.h"
#include "extensions/mouse_controller.h"

#include <tools/tools_pch.h>

namespace sfge::ext
{

static std::vector<std::function<void(py::module&)>> m_OtherPythonExtensions;

void ExtendPython(py::module& m)
{
	py::class_<PlanetSystem, System> planetSystem(m, "PlanetSystem");
	planetSystem
		.def(py::init<Engine&>());
	py::class_<AabbTest, System> aabbTest(m, "AabbTest");
	aabbTest
		.def(py::init<Engine&>());
	py::class_<QuadTreeTest, System> quadTreeTest(m, "QuadTreeTest");
	quadTreeTest
		.def(py::init<Engine&>());
	py::class_<SatTest, System> satTest(m, "SatTest");
	satTest
		.def(py::init<Engine&>());
	py::class_<ContactDebug, System> contactDebug(m, "ContactDebug");
	contactDebug
		.def(py::init<Engine&>());
	py::class_<StayOnScreen, System> stayOnScreen(m, "StayOnScreen");
	stayOnScreen
		.def(py::init<Engine&>());
	py::class_<MouseController, System> mouseController(m, "MouseController");
	mouseController
		.def(py::init<Engine&>());
	py::class_<Explosion, System> explosion(m, "Explosion");
	explosion
		.def(py::init<Engine&>());
	py::class_<FlipperScript, System> flipperScript(m, "FlipperScript");
	flipperScript
		.def(py::init<Engine&>());
	py::class_<PongScript, System> pongScript(m, "PongScript");
	pongScript
		.def(py::init<Engine&>());

	tools::ExtendPythonTools(m);
}

}
