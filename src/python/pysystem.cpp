//
// Created by efarhan on 21.08.18.
//

#include <sstream>


#include <physics/collider2d.h>
#include <python/pysystem.h>
#include <python/python_engine.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <utility/python_utility.h>

#include <utility/log.h>

#include <imgui.h>

namespace sfge
{

void PySystem::OnEngineInit()
{
	try
	{
		//py::gil_scoped_release release;
		PYBIND11_OVERLOAD_NAME(
			void,
			System,
			"init",
			OnEngineInit,

			);
	}
	catch (std::runtime_error& e)
	{
		std::stringstream oss;
		oss << "Python error on PySystem Init\n" << e.what();
		Log::GetInstance()->Error(oss.str());
	}
}

void PySystem::OnUpdate(float dt)
{

	try
	{
		//py::gil_scoped_release release;
		PYBIND11_OVERLOAD_NAME(
			void,
			System,
			"update",
			OnUpdate,
			dt
			);
	}
	catch (std::runtime_error& e)
	{
		std::stringstream oss;
		oss << "Python error on PySystem Update\n" << e.what();
		Log::GetInstance()->Error(oss.str());
	}
}

void PySystem::OnFixedUpdate()
{
	try
	{
		//py::gil_scoped_release release;
		PYBIND11_OVERLOAD_NAME(
			void,
			System,
			"fixed_update",
			OnFixedUpdate,
			);
	}
	catch (std::runtime_error& e)
	{
		std::stringstream oss;
		oss << "Python error on PySystem FixedUpdate\n" << e.what();
		Log::GetInstance()->Error(oss.str());
	}
}

void PySystem::OnDraw()
{
	try
	{

		//py::gil_scoped_release release;
		PYBIND11_OVERLOAD_NAME(
			void,
			System,
			"on_draw",
			OnDraw,
			);
	}
	catch (std::runtime_error& e)
	{
		std::stringstream oss;
		oss << "Python error on PySystem Draw\n" << e.what();
		Log::GetInstance()->Error(oss.str());
	}
}

void PySystem::OnEditorDraw()
{
	const auto pySysObj = py::cast(this);
	py::dict pySysAttrDict = pySysObj.attr("__dict__");
	for(auto& elem : pySysAttrDict)
	{
		std::string key = py::str(elem.first);
		std::string value = py::str(elem.second);
		ImGui::LabelText(key.c_str(), value.c_str());
	}
}

void PySystem::OnContact(ColliderData* c1, ColliderData* c2, bool enter)
{
	try
	{

		//py::gil_scoped_release release;
		PYBIND11_OVERLOAD_NAME(
			void,
			System,
			"on_contact",
			OnContact,
			c1,
			c2,
			enter
			);
	}
	catch (std::runtime_error& e)
	{
		std::stringstream oss;
		oss << "Python error on PySystem Contact\n" << e.what();
		Log::GetInstance()->Error(oss.str());
	}
}

std::string PySystem::GetPySystemName()
{
	std::string pySystemName;
	auto pySystem = py::cast(this, py::return_value_policy::automatic_reference);
	pySystemName = py::cast<std::string>(pySystem.attr("__class__").attr("__name__"));
	return pySystemName;
}

void PySystemManager::OnEngineInit()
{
	System::OnEngineInit();
	m_PythonEngine = m_Engine.GetPythonEngine();
}

InstanceId PySystemManager::LoadPySystem(ModuleId moduleId)
{
	std::string className = m_PythonEngine->GetClassNameFrom(moduleId);
	try
	{
		auto globals = py::globals();
		auto moduleName = m_PythonEngine->GetModuleNameFrom(moduleId);
		auto moduleObj = py::module(m_PythonEngine->GetModuleObjFrom(moduleId));

		const auto pyInstanceId = m_IncrementalInstanceId;
		//Load PySystem
		m_PythonInstances[pyInstanceId] =
			moduleObj.attr(className.c_str())(m_Engine);
		m_PySystemNames[pyInstanceId] = className;
		const auto pySystem = GetPySystemFromInstanceId(pyInstanceId);
		if (pySystem != nullptr)
		{
			m_PySystems.push_back(pySystem);

			/* TODO editor info on system
			 *
			 *auto pyInfo = editor::PyComponentInfo();
			pyInfo.name = className;
			pyInfo.path = m_PythonModulePaths[moduleId];
			pyInfo.pyComponent = pySystem;
			m_PyComponentsInfo.push_back(pyInfo);
			*/
		}
		else
		{
			std::ostringstream oss;
			oss << "[Python Error] Could not load the PyComponent* out of the instance";
			Log::GetInstance()->Error(oss.str());
			return INVALID_INSTANCE;
		}

		m_IncrementalInstanceId++;
		return pyInstanceId;
	}
	catch (std::runtime_error& e)
	{
		std::stringstream oss;
		oss << "[PYTHON ERROR] trying to instantiate class: " << className << "\n" << e.what() << "\n" << py::str(py::globals());
		Log::GetInstance()->Error(oss.str());
	}

	return INVALID_INSTANCE;
}


InstanceId PySystemManager::LoadCppExtensionSystem(std::string systemClassName)
{
	const auto pyInstanceId = m_IncrementalInstanceId;
	try
	{
		py::module sfge = py::module::import("SFGE");
		m_PythonInstances[pyInstanceId] = sfge.attr(systemClassName.c_str())(m_Engine);
		m_PySystemNames[pyInstanceId] = systemClassName;
		const auto pySystem = GetPySystemFromInstanceId(pyInstanceId);
		if (pySystem != nullptr)
		{
			m_PySystems.push_back(pySystem);
		}
		m_IncrementalInstanceId++;
		return pyInstanceId;
	}
	catch(std::runtime_error& e)
	{
		std::stringstream oss;
		oss << "[PYTHON ERROR] trying to instantiate System from C++: " << systemClassName << "\n" << e.what() << "\n";
		Log::GetInstance()->Error(oss.str());
	}
	return INVALID_INSTANCE;
}


PySystem* PySystemManager::GetPySystemFromInstanceId(InstanceId instanceId)
{
	if (instanceId > m_IncrementalInstanceId)
	{
		std::ostringstream oss;
		oss << "[Python Error] Could not find instance Id: " << instanceId << " in the pythonInstanceMap";
		Log::GetInstance()->Error(oss.str());

		return nullptr;
	}
	return m_PythonInstances[instanceId].cast<PySystem*>();
}
void PySystemManager::Destroy()
{
	System::Destroy();
	m_PySystems.clear();
	m_PythonInstances.clear();
}

PySystem *PySystemManager::GetPySystemFromClassName(std::string className)
{
	for(auto i = 0u; i< m_PySystemNames.size();i++)
	{
		if(m_PySystemNames[i] == className)
		{
			return m_PySystems[i];
		}
	}
	return nullptr;
}

std::vector<PySystem*>& PySystemManager::GetPySystems()
{
	return m_PySystems;
}
}
