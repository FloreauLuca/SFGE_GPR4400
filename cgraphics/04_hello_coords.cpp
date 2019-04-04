#include <engine/engine.h>
#include <graphics/graphics3d.h>
#include <glm/glm.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "SFML/Graphics/Texture.hpp"

#define CUBE_SAMPLE

class HelloCoordsDrawingProgram : public sfge::DrawingProgram
{
public:
	using sfge::DrawingProgram::DrawingProgram;
	void OnEngineInit() override;
	void OnDraw() override;
	void Destroy() override;
private:
	sfge::Shader shaderProgram;

	unsigned VAO;
	sf::Texture sfTextureWall;
	unsigned textureWall;

#ifdef CUBE_SAMPLE
	unsigned VBO;
	float vertices[36*5] = {
	-0.5f, -0.5f, -0.5f,  0.0f, 0.0f,
	 0.5f, -0.5f, -0.5f,  1.0f, 0.0f,
	 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
	 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
	-0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
	-0.5f, -0.5f, -0.5f,  0.0f, 0.0f,

	-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
	 0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
	 0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
	 0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
	-0.5f,  0.5f,  0.5f,  0.0f, 1.0f,
	-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,

	-0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
	-0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
	-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
	-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
	-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
	-0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

	 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
	 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
	 0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
	 0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
	 0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
	 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

	-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
	 0.5f, -0.5f, -0.5f,  1.0f, 1.0f,
	 0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
	 0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
	-0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
	-0.5f, -0.5f, -0.5f,  0.0f, 1.0f,

	-0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
	 0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
	 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
	 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
	-0.5f,  0.5f,  0.5f,  0.0f, 0.0f,
	-0.5f,  0.5f, -0.5f,  0.0f, 1.0f
	};

	glm::vec3 cubePositions[10] =
	{
		glm::vec3(0.0f,  0.0f,  0.0f),
		glm::vec3(2.0f,  5.0f, -15.0f),
		glm::vec3(-1.5f, -2.2f, -2.5f),
		glm::vec3(-3.8f, -2.0f, -12.3f),
		glm::vec3(2.4f, -0.4f, -3.5f),
		glm::vec3(-1.7f,  3.0f, -7.5f),
		glm::vec3(1.3f, -2.0f, -2.5f),
		glm::vec3(1.5f,  2.0f, -2.5f),
		glm::vec3(1.5f,  0.2f, -1.5f),
		glm::vec3(-1.3f,  1.0f, -1.5f)
	};
#else
	unsigned VBO[2];
	unsigned EBO;
	float vertices[12] = {
			0.5f,  0.5f, 0.0f,  // top right
			0.5f, -0.5f, 0.0f,  // bottom right
			-0.5f, -0.5f, 0.0f,  // bottom left
			-0.5f,  0.5f, 0.0f   // top left
	};
	float texCoords[8] = {
			1.0f, 1.0f,	  // top right
			1.0f, 0.0f,   // bottom right
			0.0f, 0.0f,   // bottom left
			0.0f, 1.0f,   // bottom left
	};
	unsigned int indices[6] = {
		// note that we start from 0!
		0, 1, 3,   // first triangle
		1, 2, 3    // second triangle
	};
#endif
};

void HelloCoordsDrawingProgram::OnEngineInit()
{
	programName = "HelloCoords";

	shaders.push_back(&shaderProgram);
	shaderProgram.Init(
		"data/shaders/04_hello_coords/coords.vert",
		"data/shaders/04_hello_coords/coords.frag");
	sfTextureWall.loadFromFile("data/sprites/wall.jpg");
	textureWall = sfTextureWall.getNativeHandle();

#ifdef CUBE_SAMPLE
	glGenBuffers(1, &VBO);//like new VBO
	glGenVertexArrays(1, &VAO); //like: new VAO()
	// 1. bind Vertex Array Object
	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	// texture coord attribute
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	
#else
	glGenBuffers(2, &VBO[0]);
	glGenBuffers(1, &EBO);

	
	glGenVertexArrays(1, &VAO); //like: new VAO()
	// 1. bind Vertex Array Object
	glBindVertexArray(VAO);//Now use our VAO
	//bind vertices data
	glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	//bind texture coords data
	glBindBuffer(GL_ARRAY_BUFFER, VBO[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(texCoords), texCoords, GL_STATIC_DRAW);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(1);
	//bind vertices index
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
#endif
	//unbind Vertex Array
	glBindVertexArray(0);
	
}

void HelloCoordsDrawingProgram::OnDraw()
{

	glEnable(GL_DEPTH_TEST);
	auto* config = m_Engine.GetConfig();

	glm::mat4 view = glm::mat4(1.0f);
	// note that we're translating the scene in the reverse direction of where we want to move
	view = glm::translate(view, glm::vec3(0.0f, 0.0f, -3.0f));

	glm::mat4 projection;
	projection = glm::perspective(glm::radians(45.0f), (float)config->screenResolution.x / config->screenResolution.y, 0.1f, 100.0f);
	//projection = glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f);
	shaderProgram.Bind();
	glBindTexture(GL_TEXTURE_2D, textureWall);
	glBindVertexArray(VAO);
	int viewLoc = glGetUniformLocation(shaderProgram.GetProgram(), "view");//camera transform matrix
	glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

	int projectionLoc = glGetUniformLocation(shaderProgram.GetProgram(), "projection");//projection matrix
	glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));
#ifdef CUBE_SAMPLE
	for (int i = 0; i < 10; i++)
	{
		glm::mat4 model = glm::mat4(1.0f); //model transform matrix
		model = glm::translate(model, cubePositions[i]);
		model = glm::rotate(model, glm::radians(m_Engine.GetTimeSinceInit()*45.0f), glm::vec3(1.0f, 0.0f, 0.0f));
		model = glm::rotate(model, glm::radians(m_Engine.GetTimeSinceInit()*45.0f), glm::vec3(0.0f, 1.0f, 0.0f));

		int modelLoc = glGetUniformLocation(shaderProgram.GetProgram(), "model");
		glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

		glDrawArrays(GL_TRIANGLES, 0, 36);
	}
#else
	glm::mat4 model = glm::mat4(1.0f); //model transform matrix
	model = glm::rotate(model, glm::radians(m_Engine.GetTimeSinceInit()*45.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	int modelLoc = glGetUniformLocation(shaderProgram.GetProgram(), "model");
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
#endif
	glBindVertexArray(0);
}

void HelloCoordsDrawingProgram::Destroy()
{
	glDeleteVertexArrays(1, &VAO);
#ifdef CUBE_SAMPLE
	glDeleteBuffers(1, &VBO);
#else
	glDeleteBuffers(2, &VBO[0]);
	glDeleteBuffers(2, &EBO);
#endif
}

int main(int argc, char** argv)
{
	sfge::Engine engine;

	HelloCoordsDrawingProgram helloCoords(engine);
	auto* graphics3dManager = engine.GetGraphics3dManager();
	graphics3dManager->AddDrawingProgam(&helloCoords);
	{
		auto config = std::make_unique<sfge::Configuration>();
		config->screenResolution.x = 1024;
		config->screenResolution.y = 1024;
		config->bgColor = sf::Color::Black;
		config->windowName = "Hello Coords";

		engine.Init(std::move(config));
	}


	engine.Start();
	return EXIT_SUCCESS;
}
