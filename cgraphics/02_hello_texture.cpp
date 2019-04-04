#include <engine/engine.h>
#include <graphics/graphics3d.h>

#include <SFML/Graphics/Texture.hpp>
#include "utility/file_utility.h"
#include "utility/log.h"


#define OTHER_TEXTURE
#define USE_STB_IMAGE

#ifdef USE_STB_IMAGE
#include "stb_image/stb_image.h"
#endif

class HelloTextureDrawingProgram : public sfge::DrawingProgram
{
public:
	using sfge::DrawingProgram::DrawingProgram;
	void OnEngineInit() override;
	void OnDraw() override;
	void Destroy() override;
#ifdef USE_STB_IMAGE
	int LoadTexture(const std::string& filename);
#endif
private:

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
		0.0f, 1.0f,   // top left
	};
	unsigned int indices[6] = {
		// note that we start from 0!
		0, 1, 3,   // first triangle
		1, 2, 3    // second triangle
	};

	sfge::Shader shaderProgram;
	unsigned int VBO[2] = {}; //Vertex Buffer Object
	unsigned int VAO = 0; //Vertex Array Object
	unsigned int EBO = 0; // Element Buffer Object
#ifndef USE_STB_IMAGE
	sf::Texture sfTextureWall{};
#endif
	GLuint textureWall;
#ifdef OTHER_TEXTURE
#ifndef USE_STB_IMAGE
	sf::Texture sfTextureOtherPlay{};
#endif
	GLuint textureOtherPlay;
#endif
};

void HelloTextureDrawingProgram::OnEngineInit()
{

	programName = "HelloTexture";

	shaders.push_back(&shaderProgram);

	glGenBuffers(2, &VBO[0]);
	glGenBuffers(1, &EBO);

	shaderProgram.Init(
		"data/shaders/02_hello_texture/texture.vert",
#ifdef OTHER_TEXTURE
		"data/shaders/02_hello_texture/texture_other.frag"
#else
		"data/shaders/02_hello_texture/texture.frag"
#endif
	);
#ifndef USE_STB_IMAGE
	sfTextureWall.loadFromFile("data/sprites/wall.jpg");
	textureWall = sfTextureWall.getNativeHandle();
#ifdef OTHER_TEXTURE
	sfTextureOtherPlay.loadFromFile("data/sprites/other_play.png");
	textureOtherPlay = sfTextureOtherPlay.getNativeHandle();
#endif
#else
	textureWall = LoadTexture("data/sprites/wall.jpg");
#ifdef OTHER_TEXTURE
	textureOtherPlay = LoadTexture("data/sprites/other_play.png");
#endif
#endif
	glGenVertexArrays(1, &VAO);
	// 1. bind Vertex Array Object
	glBindVertexArray(VAO);
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
}

void HelloTextureDrawingProgram::OnDraw()
{
	shaderProgram.Bind();
#ifdef OTHER_TEXTURE
	glUniform1i(glGetUniformLocation(shaderProgram.GetProgram(), "texture1"), 0);
	glUniform1i(glGetUniformLocation(shaderProgram.GetProgram(), "texture2"), 1);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureWall);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, textureOtherPlay);
#else
	glBindTexture(GL_TEXTURE_2D, textureWall);
#endif
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
	
}

void HelloTextureDrawingProgram::Destroy()
{
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(2, &VBO[0]);
	glDeleteBuffers(2, &EBO);

}

#ifdef USE_STB_IMAGE
int HelloTextureDrawingProgram::LoadTexture(const std::string& filename)
{
	std::string extension = sfge::GetFilenameExtension(filename);
	int width, height, nrChannels;
	
	unsigned char *data = stbi_load(filename.c_str(), &width, &height, &nrChannels, 0);
	if (data == nullptr)
	{
		std::ostringstream oss;
		oss << "[Error] Texture: cannot load " << filename << "\n";
		sfge::Log::GetInstance()->Error(oss.str());
		return 0;
	}
	unsigned int texture;
	glGenTextures(1, &texture);

	glBindTexture(GL_TEXTURE_2D, texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	if (extension == ".jpg")
	{
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
	}
	else if(extension == ".png")
	{
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
	}
	glGenerateMipmap(GL_TEXTURE_2D);

	stbi_image_free(data);
	return texture;
}
#endif

int main(int argc, char** argv)
{
	sfge::Engine engine;
	auto* graphics3dManager = engine.GetGraphics3dManager();
	HelloTextureDrawingProgram helloTexture(engine);
	graphics3dManager->AddDrawingProgam(&helloTexture);
	{
		auto config = std::make_unique<sfge::Configuration>();
		config->screenResolution.x = 1024;
		config->screenResolution.y = 1024;
		config->bgColor = sf::Color::Black;
		config->windowName = "Hello Texture";

		engine.Init(std::move(config));
	}
	engine.Start();
	return EXIT_SUCCESS;
}
