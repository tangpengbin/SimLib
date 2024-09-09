#pragma once

#include <Eigen/Core>
#include <igl/opengl/gl.h>

#include <string>
#include <utility>
#include <memory>
#include <map>

class ShaderPipelineDesc
{
	struct Part
	{
		Part(GLenum type, std::string path)
			:shaderType(type), filePath(path)
		{

		}

		GLenum shaderType;
		std::string filePath;
	};
public:
	static ShaderPipelineDesc VertexFragment(const std::string& vertex_file_path, const std::string& fragment_file_path);
	static ShaderPipelineDesc VertexGeometryFragment(const std::string& vertex_file_path, const std::string& geometry_shader_file_path, const std::string& fragment_file_path);


	int getPartCount() const
	{
		return (int)m_parts.size();
	}

	GLenum getShaderType(int idx) const
	{
		return m_parts[idx].shaderType;
	}

	const std::string& getFilePath(int idx) const
	{
		return m_parts[idx].filePath;
	}

private:

	std::vector<Part> m_parts;
};
inline bool operator<(const ShaderPipelineDesc& desc1, const ShaderPipelineDesc& desc2)
{
	if (desc1.getPartCount() < desc2.getPartCount()) return true;
	if (desc1.getPartCount() > desc2.getPartCount()) return false;
	for (int i = 0; i < desc1.getPartCount(); i++)
	{
		if (desc1.getShaderType(i) < desc2.getShaderType(i)) return true;
		if (desc1.getShaderType(i) > desc2.getShaderType(i)) return false;
		if (desc1.getFilePath(i) < desc2.getFilePath(i)) return true;
		if (desc1.getFilePath(i) > desc2.getFilePath(i)) return false;
	}
	return false; // they are equal
}

bool LoadShaders(const ShaderPipelineDesc& desc, const std::map<std::string, std::string> &includeDefinitions, GLuint& programId);
bool LoadShaders(const ShaderPipelineDesc& desc, GLuint& programId);

class ProgramGL_resource
{
public:

	ProgramGL_resource(GLuint programId)
		:m_program(programId)
	{

	}

	~ProgramGL_resource()
	{
		if (m_program != -1) glDeleteProgram(m_program);
	}

	void update(GLuint id)
	{
		if (m_program != -1) glDeleteProgram(m_program);
		m_program = id;
	}

	GLuint m_program;
};

class ProgramGL
{
public:
	ProgramGL()
	{

	}
	/** takes ownership of the resource */
	ProgramGL(GLuint programId)
		:m_resource(new ProgramGL_resource(programId))
	{

	}
	ProgramGL(const std::shared_ptr<ProgramGL_resource>& resource)
		:m_resource(resource)
	{

	}

	ProgramGL(const std::string& vertex_file_path, const std::string& fragment_file_path);


	static ProgramGL loadFromFiles(const ShaderPipelineDesc& desc,
		const std::map<std::string, std::string> &includeDefinitions = std::map<std::string, std::string>());

	void use();
	void unbind();

	void setUniform1i(const char* name, int i)
	{
		glUniform1i(glGetUniformLocation(m_resource->m_program, name), i);
	}

	void setUniform1f(const char* name, float f)
	{
		glUniform1f(glGetUniformLocation(m_resource->m_program, name), f);
	}
	void setUniform(const char* name, const Eigen::Vector2f &vec)
	{
		glUniform2f(glGetUniformLocation(m_resource->m_program, name), vec[0], vec[1]);
	}
	void setUniform(const char* name, const Eigen::Vector3f &vec)
	{
		glUniform3f(glGetUniformLocation(m_resource->m_program, name), vec[0], vec[1], vec[2]);
	}
	void setUniform(const char* name, const Eigen::Vector4f &vec)
	{
		glUniform4f(glGetUniformLocation(m_resource->m_program, name), vec[0], vec[1], vec[2], vec[3]);
	}
	void setUniform(const char* name, const Eigen::Matrix3f &mat)
	{
		glUniformMatrix3fv(glGetUniformLocation(m_resource->m_program, name), 1, GL_FALSE, mat.data());
	}
	void setUniform(const char* name, const Eigen::Matrix4f &mat)
	{
		glUniformMatrix4fv(glGetUniformLocation(m_resource->m_program, name), 1, GL_FALSE, mat.data());
	}

	bool hasUniform(const char* name)
	{
		return glGetUniformLocation(m_resource->m_program, name) != -1;
	}

	GLuint getId()
	{
		return m_resource->m_program;
	}

	std::shared_ptr<ProgramGL_resource> getResource()
	{
		return m_resource;
	}
private:
	std::shared_ptr<ProgramGL_resource> m_resource;
};
