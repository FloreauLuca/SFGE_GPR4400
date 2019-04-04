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

#include <iostream>
#include <cmath>
#include <engine/engine.h>
#include <engine/system.h>
#include <graphics/graphics3d.h>
#include "utility/log.h"

#define EBO_DOUBLE_TRIANGLE

class HelloTriangleDrawingProgram : public sfge::DrawingProgram
{
public:
	using sfge::DrawingProgram::DrawingProgram;
	void OnEngineInit() override;
	void OnDraw() override;
	void Destroy() override;
private:

#ifndef EBO_DOUBLE_TRIANGLE
	float vertices[9] = {
		-0.5f, -0.5f, 0.0f,
		 0.5f, -0.5f, 0.0f,
		 0.0f,  0.5f, 0.0f
	};
	float colors[9] = {
		1.0f,0.0f,1.0f,
		0.0f,1.0f,0.0f,
		0.0f,0.0f,1.0f
	};
#else
	float vertices[12] = {
		 0.5f,  0.5f, 0.0f,  // top right
		 0.5f, -0.5f, 0.0f,  // bottom right
		-0.5f, -0.5f, 0.0f,  // bottom left
		-0.5f,  0.5f, 0.0f   // top left 
	};
	float colors[12] = {
		1.0f, 0.0f, 1.0f,
		0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 1.0f,
		0.5f, 0.0f, 0.5f
	};
	unsigned int indices[6] = {
		// note that we start from 0!
		0, 1, 3,   // first triangle
		1, 2, 3    // second triangle
	};

	unsigned int EBO = 0; // Element Buffer Object
#endif

	unsigned int VBO[2] = {}; //Vertex Buffer Object
	unsigned int VAO = 0; //Vertex Array Object
	sfge::Shader shaderProgram;

};

void HelloTriangleDrawingProgram::OnEngineInit()
{
	programName = "HelloTriangle";
	shaders.push_back(&shaderProgram);

	glGenBuffers(2, &VBO[0]); //declaration of vbo, one for position, one for color, no size yet
#ifdef EBO_DOUBLE_TRIANGLE
	glGenBuffers(1, &EBO);
#endif
	
	shaderProgram.Init("data/shaders/01_hello_triangle/triangle.vert", 
		"data/shaders/01_hello_triangle/triangle.frag");

	glGenVertexArrays(1, &VAO);//declaration of vao
	// 1. bind Vertex Array Object
	glBindVertexArray(VAO);
	// 2. copy our vertices array in a buffer for OpenGL to use
	glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);//buffer data 9*4 bytes
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	// 2. copy our colors array in a buffer for OpenGL to use
	glBindBuffer(GL_ARRAY_BUFFER, VBO[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(colors), colors, GL_STATIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(1);
#ifdef EBO_DOUBLE_TRIANGLE
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
#endif
	glBindVertexArray(0);
}

void HelloTriangleDrawingProgram::OnDraw()
{
	shaderProgram.Bind();
	const float timeValue = m_Engine.GetTimeSinceInit();
	const float colorValue = (sin(timeValue) + 1.0f) / 2.0f;
	if(shaderProgram.GetProgram() == 0)
	{
		sfge::Log::GetInstance()->Error("[Error] No shader program");
		return;
	}
	glUseProgram(shaderProgram.GetProgram());
	shaderProgram.SetFloat("colorCoeff", colorValue);

	glBindVertexArray(VAO);
#ifndef EBO_DOUBLE_TRIANGLE
	glDrawArrays(GL_TRIANGLES, 0, 3);
#else
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
#endif
	glBindVertexArray(0);
}

void HelloTriangleDrawingProgram::Destroy()
{
	glDeleteVertexArrays(1, &VAO);
#ifndef EBO_DOUBLE_TRIANGLE
	glDeleteBuffers(1, &VBO[0]);
#else
	glDeleteBuffers(2, &VBO[0]);
	glDeleteBuffers(2, &EBO);
#endif
	glDeleteProgram(shaderProgram.GetProgram());
}

int main(int argc, char **argv)
{
	sfge::Engine engine;


	HelloTriangleDrawingProgram helloTriangleProgram(engine);

	auto* graphics3dManager = engine.GetGraphics3dManager();
	graphics3dManager->AddDrawingProgam(&helloTriangleProgram);

	{
		auto config = std::make_unique<sfge::Configuration>();
		
		config->screenResolution.x = 1024;
		config->screenResolution.y = 1024;
		config->bgColor = { 255,255,255,255 };
		config->windowName = "Hello Triangle";

		engine.Init(std::move(config));
	}

	engine.Start();

	return EXIT_SUCCESS;
}
