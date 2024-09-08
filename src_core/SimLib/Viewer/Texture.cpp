#include "Texture.h"

//see https://github.com/ocornut/imgui/wiki/Image-Loading-and-Displaying-Examples

#undef STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#include <cassert>

// Simple helper function to load an image into a OpenGL texture with common settings
bool LoadTextureFromFile(const char* filename, GLuint* out_texture, int* out_width, int* out_height)
{
	// Load from file
	int image_width = 0;
	int image_height = 0;
	unsigned char* image_data = stbi_load(filename, &image_width, &image_height, NULL, 4);
	if (image_data == NULL)
		return false;

	// Create a OpenGL texture identifier
	GLuint image_texture;
	glGenTextures(1, &image_texture);
	glBindTexture(GL_TEXTURE_2D, image_texture);

	// Setup filtering parameters for display
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Upload pixels into texture
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);


	stbi_image_free(image_data);

	*out_texture = image_texture;
	*out_width = image_width;
	*out_height = image_height;

	return true;
}

Texture::Texture()
{
	m_image_data = nullptr;
}
Texture::~Texture()
{
}
bool Texture::load(const std::string &filename)
{
	int image_width = 0;
	int image_height = 0;
	unsigned char* image_data = stbi_load(filename.c_str(), &image_width, &image_height, NULL, 4);
	if (image_data == NULL)
		return false;

	free();
	m_image_data = image_data;
	m_image_width = image_width;
	m_image_height = image_height;
	return true;
}
void Texture::free()
{
	if (m_image_data != nullptr)
	{
		stbi_image_free(m_image_data);
		m_image_data = nullptr;
	}
}
TextureGPU::TextureGPU(const Texture &cpuTexture, bool convertFromSRGBToLinear)
{
	assert(glGetError() == GL_NO_ERROR);
	m_width = cpuTexture.width();
	m_height = cpuTexture.height();

	// Create a OpenGL texture identifier
	glGenTextures(1, &m_textureId);
	glBindTexture(GL_TEXTURE_2D, m_textureId);

	//// Setup filtering parameters for display
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);


	// Upload pixels into texture
	//glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	if(convertFromSRGBToLinear) glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8_ALPHA8, m_width, m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, cpuTexture.data());
	else glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_width, m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, cpuTexture.data());

	//generate mipmap structure for filtered display
	glGenerateMipmap(GL_TEXTURE_2D);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glBindTexture(GL_TEXTURE_2D, 0);
	assert(glGetError() == GL_NO_ERROR);
}
TextureGPU::~TextureGPU()
{
	glDeleteTextures(1, &m_textureId);
}
void TextureGPU::bind() const
{
	glBindTexture(GL_TEXTURE_2D, m_textureId);
}
void TextureGPU::unbind() const
{
	glBindTexture(GL_TEXTURE_2D, 0);
}
TextureManaged::TextureManaged()
{
	m_glInitialized = false;
	m_convertFromSRGBToLinear = false;
}
bool TextureManaged::load(const std::string &filename, bool convertFromSRGBToLinear)
{
	m_convertFromSRGBToLinear = convertFromSRGBToLinear;
	bool succeeded = m_cpuTexture.load(filename);
	if (!succeeded) return false;

	if (m_glInitialized)
	{
		m_gpuTexture.reset(new TextureGPU(m_cpuTexture, m_convertFromSRGBToLinear));
	}
	return true;
}
void TextureManaged::initGL()
{
	m_glInitialized = true;
	if (m_cpuTexture.has())
	{
		m_gpuTexture.reset(new TextureGPU(m_cpuTexture, m_convertFromSRGBToLinear));
	}
}
void TextureManaged::shutGL()
{
	m_glInitialized = false;
}
