#pragma once

#include "GL.h"

class ShadowMap
{
public:
	ShadowMap();
	~ShadowMap();

	bool init(GLuint bufferWidth, GLuint bufferHeight);

	void bindForRendering();
	void unbindForRendering();
	void clearDepth();

	GLuint getShadowTexture()
	{
		return m_texture;
	}

	GLuint getWidth()const
	{
		return m_bufferWidth;
	}
	GLuint getHeight()const
	{
		return m_bufferHeight;
	}
private:
	GLuint m_fbo;
	GLuint m_texture;
	GLuint m_bufferWidth;
	GLuint m_bufferHeight;
};