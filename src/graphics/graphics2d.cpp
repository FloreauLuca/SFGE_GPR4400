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
#include <graphics/graphics2d.h>
#include <engine/engine.h>
#include <utility/log.h>
#include <engine/config.h>

//Dependencies includes
#include <SFML/Graphics/RenderWindow.hpp>
#include <imgui-SFML.h>
#include <imgui.h>


namespace sfge
{

void Graphics2dManager::OnEngineInit()
{
	if (const auto configPtr = m_Engine.GetConfig())
	{
		m_Windowless = configPtr->windowLess;
		if (!m_Windowless)
		{
			sf::ContextSettings settings;
			settings.depthBits = 24;
			settings.stencilBits = 8;
			settings.antialiasingLevel = 4;
			settings.majorVersion = 4;
			settings.minorVersion = 6;
			m_Window = std::make_unique<sf::RenderWindow>(
				sf::VideoMode(configPtr->screenResolution.x, configPtr->screenResolution.y),
				configPtr->windowName, sf::Style::Default, settings);
			if (configPtr->maxFramerate)
			{
				m_Window->setFramerateLimit(configPtr->maxFramerate);
				CheckVersion();
			}

		}
	}
	else
	{
		Log::GetInstance()->Error("[Error] Config is null from Graphics Manager");
		
	}
	m_TextureManager.OnEngineInit();
	m_ShapeManager.OnEngineInit();
	m_SpriteManager.OnEngineInit();

}

void Graphics2dManager::OnUpdate(float dt)
{
	if (!m_Windowless)
	{
		rmt_ScopedCPUSample(Graphics2dUpdate,0)
		m_Window->clear();

		m_SpriteManager.OnUpdate(dt);
		m_ShapeManager.OnUpdate(dt);

		
	}
}

void Graphics2dManager::OnDraw()
{
	rmt_ScopedCPUSample(Graphics2dDraw,0);
	if(!m_Windowless)
	{
		m_SpriteManager.DrawSprites(*m_Window);
		m_ShapeManager.DrawShapes(*m_Window);
	}
}

void Graphics2dManager::Display()
{

	rmt_ScopedCPUSample(Graphics2dDisplay,0)
	if (!m_Windowless)
	{
		m_Window->display();
	}
}

void Graphics2dManager::DrawLine(Vec2f from, Vec2f to, sf::Color color)
{
	sf::Vertex vertices[2] =
	{
	    sf::Vertex(from, color),
	    sf::Vertex(to, color)
	};

	m_Window->draw(vertices, 2, sf::Lines);
}

sf::RenderWindow* Graphics2dManager::GetWindow()
{
	return m_Window.get();
}

SpriteManager* Graphics2dManager::GetSpriteManager()
{
	return &m_SpriteManager;
}

TextureManager* Graphics2dManager::GetTextureManager()
{
	return &m_TextureManager;
}

ShapeManager* Graphics2dManager::GetShapeManager()
{
	return &m_ShapeManager;
}

void Graphics2dManager::CheckVersion() const
{
	sf::ContextSettings settings = m_Window->getSettings();
	std::stringstream log_message;
	log_message << "OpenGL version: " << settings.majorVersion << "." << settings.minorVersion << std::endl;

	Log::GetInstance()->Msg(log_message.str());
}


void Graphics2dManager::Destroy()
{
	OnBeforeSceneLoad();
	OnAfterSceneLoad();

	m_Window = nullptr;
}

void Graphics2dManager::OnBeforeSceneLoad()
{
	m_TextureManager.OnBeforeSceneLoad();
	m_SpriteManager.OnBeforeSceneLoad();
}

void Graphics2dManager::OnAfterSceneLoad()
{

	m_TextureManager.OnAfterSceneLoad();
	m_SpriteManager.OnAfterSceneLoad();
}

void Graphics2dManager::DrawVector(Vec2f drawingVector, Vec2f originPos, sf::Color color)
{

    const Vec2f destination = originPos+drawingVector*debugVectorPixelResolution;
    //Draw length line
    DrawLine(originPos, destination, color);
    const Vec2f dir = drawingVector.Normalized();
    const float length = (drawingVector*debugVectorPixelResolution).GetMagnitude();
    DrawLine(destination, destination+dir.Rotate(135.0f)*length/5,color);
    DrawLine(destination, destination+dir.Rotate(-135.0f)*length/5,color);

}

}
