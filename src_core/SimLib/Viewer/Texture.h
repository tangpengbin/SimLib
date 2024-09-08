#pragma once

#include "GL.h"
#include <string>

class Texture
{
public:
	Texture();
	~Texture();
	bool load(const std::string &filename);

	bool has() const { return m_image_data != nullptr; }
	int width() const { return m_image_width; }
	int height() const { return m_image_height; }
	const unsigned char * data() const { return m_image_data; }
private:
	void free();

	int m_image_width;
	int m_image_height;
	unsigned char* m_image_data;
};

class TextureGPU
{
public:
	TextureGPU(const Texture &cpuTexture, bool convertFromSRGBToLinear);
	~TextureGPU();

	void bind() const;
	void unbind() const;

	GLuint getId() const
	{
		return m_textureId;
	}

private:
	GLuint m_textureId;
	GLsizei m_width;
	GLsizei m_height;
};
class TextureManaged
{
public:
	TextureManaged();

	bool load(const std::string &filename, bool convertFromSRGBToLinear);

	void initGL();
	void shutGL();

	bool hasGPUTexture() const { return m_gpuTexture.operator bool(); }
	const TextureGPU& getGPUTexture() const { return *m_gpuTexture; }

	int width() const { return m_cpuTexture.width(); }
	int height() const { return m_cpuTexture.height(); }

private:
	bool m_glInitialized;
	Texture m_cpuTexture;
	std::unique_ptr<TextureGPU> m_gpuTexture;
	bool m_convertFromSRGBToLinear;
};
