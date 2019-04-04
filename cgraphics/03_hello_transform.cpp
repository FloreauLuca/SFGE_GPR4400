#include <engine/engine.h>

#include "graphics/graphics3d.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "SFML/Graphics/Texture.hpp"
#include "imgui.h"

//#define ROTATE_OVER_TIME

class HelloTransformDrawingProgram : public sfge::DrawingProgram
{
public:
	using sfge::DrawingProgram::DrawingProgram;
    void OnEngineInit() override;
    void OnDraw() override;
    void Destroy() override;
	void OnEditorDraw() override;
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
    unsigned VBO[2];
    unsigned EBO;
    unsigned VAO;
	sf::Texture sfTextureWall;
    unsigned textureWall;

	float angle = 45.0f;
	float x = 0.0f, y = 0.0f, z = 0.0f;
	float scale = 0.5f;
};

void HelloTransformDrawingProgram::OnEngineInit()
{
    programName = "HelloTransform";

    shaders.push_back(&shaderProgram);

    glGenBuffers(2, &VBO[0]);
    glGenBuffers(1, &EBO);

    shaderProgram.Init(
            "data/shaders/03_hello_transform/transform.vert",
            "data/shaders/03_hello_transform/transform.frag"
    );
	sfTextureWall.loadFromFile("data/sprites/wall.jpg");
	textureWall = sfTextureWall.getNativeHandle();
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
    //unbind Vertex Array
    glBindVertexArray(0);
}

void HelloTransformDrawingProgram::OnDraw()
{
    glm::mat4 trans = glm::mat4(1.0f);
#ifndef ROTATE_OVER_TIME
	trans = glm::translate(trans, glm::vec3(x, y, z));
    trans = glm::rotate(trans, glm::radians(angle), glm::vec3(0.0, 0.0, 1.0));
    trans = glm::scale(trans, glm::vec3(scale, scale, scale));
#else
    trans = glm::translate(trans, glm::vec3(0.5f, -0.5f, 0.0f));
    trans = glm::rotate(trans, m_Engine.GetTimeSinceInit(), glm::vec3(0.0f, 0.0f, 1.0f));
#endif

    shaderProgram.Bind();

    //unsigned int transformLoc = glGetUniformLocation(shaderProgram.GetProgram(), "transform");
    //glUniformMatrix4fv(transformLoc, 1, GL_FALSE, glm::value_ptr(trans));

	shaderProgram.SetMat4("transform", trans);

    glBindTexture(GL_TEXTURE_2D, textureWall);
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

}

void HelloTransformDrawingProgram::Destroy()
{
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(2, &VBO[0]);
	glDeleteBuffers(2, &EBO);
}

void HelloTransformDrawingProgram::OnEditorDraw()
{
	sfge::DrawingProgram::OnEditorDraw();
	ImGui::Separator();
	ImGui::SliderFloat("X", &x, -2.0f, 2.0f, "x = %.3f");
	ImGui::SliderFloat("Y", &y, -2.0f, 2.0f, "y = %.3f");
	ImGui::SliderFloat("Z", &z, -2.0f, 2.0f, "z = %.3f");
	ImGui::SliderFloat("Scale", &scale, 0.0f, 10.0f, "scale = %.3f");

	ImGui::SliderFloat("Angle", &angle, -180.0f, 180.0f, "angle = %.3f");
}

int main(int argc, char** argv)
{
	sfge::Engine engine;

	HelloTransformDrawingProgram helloTransform(engine);

	auto* graphics3dManager = engine.GetGraphics3dManager();
	graphics3dManager->AddDrawingProgam(&helloTransform);
	{
		auto config = std::make_unique<sfge::Configuration>();
		config->screenResolution.x = 1024;
		config->screenResolution.y = 1024;
		//config.bgColor = sf::Color::Black;
		config->windowName = "Hello Transform";
		engine.Init(std::move(config));
	}

    engine.Start();

    return EXIT_SUCCESS;
}
