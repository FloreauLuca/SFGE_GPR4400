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
#include <sstream>
#include <GL/glew.h>
#include <SFML/OpenGL.hpp>
#include <graphics/graphics3d.h>
#include "utility/file_utility.h"
#include "utility/log.h"
#include <imgui.h>
#include <glm/vec2.hpp>

#include <Remotery.h>

namespace sfge
{

void Shader::Init(std::string vertexShaderPath, std::string fragmentShaderPath)
{
	const unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);//declare vertex shader
	const auto vertexShaderProgram = LoadFile(vertexShaderPath);
	const char* vertexShaderChar = vertexShaderProgram.c_str();

	glShaderSource(vertexShader, 1, &vertexShaderChar, NULL);//define vertex shader data
	glCompileShader(vertexShader);//compile
	//Check success status of shader compilation 
	int  success;
	char infoLog[512];
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
		std::ostringstream oss;
		oss << "[Error] Vertex Shader Compilation for "<< vertexShaderPath <<"\n" << infoLog;
		Log::GetInstance()->Error(oss.str());
		return;
	}

	const unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	auto fragmentShaderProgram = LoadFile(fragmentShaderPath);
	const char* fragmentShaderChar = fragmentShaderProgram.c_str();
	glShaderSource(fragmentShader, 1, &fragmentShaderChar, NULL);
	glCompileShader(fragmentShader);
	//Check success status of shader compilation 
	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
		std::ostringstream oss;
		oss << "[Error] Fragment Shader Compilation for " << fragmentShaderPath << "\n" << infoLog;
		Log::GetInstance()->Error(oss.str());
		return;
	}

	shaderProgram = glCreateProgram();
	glAttachShader(shaderProgram, vertexShader);
	glAttachShader(shaderProgram, fragmentShader);
	glLinkProgram(shaderProgram);
	//Check if shader program was linked correctly
	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
	if (!success) {
		glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
		std::cerr << "[Error] Shader" << vertexShaderPath << " and "<<fragmentShaderPath<<" link error\n" << infoLog << std::endl;
		return;
	}

	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
}

void Shader::Bind()
{
	if(shaderProgram != 0)
		glUseProgram(shaderProgram);
}

int Shader::GetProgram()
{
	return shaderProgram;
}

void Shader::SetBool(const std::string& attributeName, bool value) const
{
	glUniform1i(glGetUniformLocation(shaderProgram, attributeName.c_str()), (int)value);
}

void Shader::SetInt(const std::string& attributeName, int value) const
{
	glUniform1i(glGetUniformLocation(shaderProgram, attributeName.c_str()), value);
}

void Shader::SetFloat(const std::string& attributeName, float value) const
{
	glUniform1f(glGetUniformLocation(shaderProgram, attributeName.c_str()), value);
}

// ------------------------------------------------------------------------
void  Shader::SetVec2(const std::string &name, const glm::vec2 &value) const
{
	glUniform2fv(glGetUniformLocation(shaderProgram, name.c_str()), 1, &value[0]);
}
void  Shader::SetVec2(const std::string &name, float x, float y) const
{
	glUniform2f(glGetUniformLocation(shaderProgram, name.c_str()), x, y);
}
// ------------------------------------------------------------------------
void  Shader::SetVec3(const std::string &name, const glm::vec3 &value) const
{
	glUniform3fv(glGetUniformLocation(shaderProgram, name.c_str()), 1, &value[0]);
}
void  Shader::SetVec3(const std::string &name, float x, float y, float z) const
{
	glUniform3f(glGetUniformLocation(shaderProgram, name.c_str()), x, y, z);
}
// ------------------------------------------------------------------------
void  Shader::SetVec4(const std::string &name, const glm::vec4 &value) const
{
	glUniform4fv(glGetUniformLocation(shaderProgram, name.c_str()), 1, &value[0]);
}
void  Shader::SetVec4(const std::string &name, float x, float y, float z, float w)
{
	glUniform4f(glGetUniformLocation(shaderProgram, name.c_str()), x, y, z, w);
}
// ------------------------------------------------------------------------
void  Shader::SetMat2(const std::string &name, const glm::mat2 &mat) const
{
	glUniformMatrix2fv(glGetUniformLocation(shaderProgram, name.c_str()), 1, GL_FALSE, &mat[0][0]);
}
// ------------------------------------------------------------------------
void  Shader::SetMat3(const std::string &name, const glm::mat3 &mat) const
{
	glUniformMatrix3fv(glGetUniformLocation(shaderProgram, name.c_str()), 1, GL_FALSE, &mat[0][0]);
}
// ------------------------------------------------------------------------
void  Shader::SetMat4(const std::string &name, const glm::mat4 &mat) const
{
	glUniformMatrix4fv(glGetUniformLocation(shaderProgram, name.c_str()), 1, GL_FALSE, &mat[0][0]);
}

void Graphics3dManager::AddDrawingProgam(DrawingProgram* drawingProgram)
{
	m_DrawingPrograms.push_back(drawingProgram);
}

void Graphics3dManager::OnEngineInit()
{
	for (auto* drawingProgram : m_DrawingPrograms)
	{
		drawingProgram->OnEngineInit();
	}
}

void Graphics3dManager::OnDraw()
{
	rmt_ScopedCPUSample(Graphics3dDraw,0);
	for(auto* drawingProgram : m_DrawingPrograms)
	{
		drawingProgram->OnDraw();
	}
}


void Graphics3dManager::Destroy()
{
	for(int i = 0; i < m_DrawingPrograms.size();i++)
	{
		m_DrawingPrograms[i]->Destroy();
	}
	m_DrawingPrograms.clear();
}

std::vector<DrawingProgram*>& Graphics3dManager::GetDrawingPrograms()
{
	return m_DrawingPrograms;
}

void DrawingProgram::OnEditorDraw()
{
	ImGui::Separator();
	ImGui::Text("Shaders Data");
	ImGui::Separator();
	for (int i = 0; i < shaders.size(); i++)
	{
		GLint count;

		GLint size; // size of the variable
		GLenum type; // type of the variable (float, vec3 or mat4, etc)

		const GLsizei bufSize = 16; // maximum name length
		GLchar name[bufSize]; // variable name in GLSL
		GLsizei length; // name length

		auto* shader = shaders[i];
		ImGui::Text("Shader %d", i);
		glGetProgramiv(shader->GetProgram(), GL_ACTIVE_ATTRIBUTES, &count);

		for (int j = 0; j < count; j++)
		{
			glGetActiveAttrib(shader->GetProgram(), (GLuint)j, bufSize, &length, &size, &type, name);
			int loc = glGetAttribLocation(shader->GetProgram(), name);
			ImGui::Text("Attribute #%d Type: %u Name: %s", loc, type, name);
		}

		glGetProgramiv(shader->GetProgram(), GL_ACTIVE_UNIFORMS, &count);

		for (int j = 0; j < count; j++)
		{
			glGetActiveUniform(shader->GetProgram(), (GLuint)j, bufSize, &length, &size, &type, name);
			int loc = glGetUniformLocation(shader->GetProgram(), name);
			ImGui::Text("Uniform #%d Type: %u Name: %s", loc, type, name);
		}
	}
}

const std::string& DrawingProgram::GetProgramName()
{
	return programName;
}
}
