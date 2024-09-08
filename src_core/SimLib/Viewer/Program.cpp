#include "Program.h"

#include "../Core/File.h"

#include "../Core/Error.h"

#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <ios>
#include <algorithm>

#include <stdlib.h>
#include <string.h>

bool compileShader(GLenum shaderType, const std::string &filePath, const std::string &shaderSource, GLuint &shaderId)
{
	shaderId = glCreateShader(shaderType);
	printf("Compiling shader : %s\n", filePath.c_str());
	char const * VertexSourcePointer = shaderSource.c_str();
	glShaderSource(shaderId, 1, &VertexSourcePointer, NULL);
	glCompileShader(shaderId);

	// Check Vertex Shader
	GLint Result = GL_FALSE;
	int InfoLogLength;
	glGetShaderiv(shaderId, GL_COMPILE_STATUS, &Result);
	glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &InfoLogLength);
	if (InfoLogLength > 0)
	{
		std::vector<char> VertexShaderErrorMessage(InfoLogLength + 1);
		glGetShaderInfoLog(shaderId, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
		printf("%s\n", &VertexShaderErrorMessage[0]);
	}
	return Result == GL_TRUE;
}

GLuint linkProgram(const std::vector<GLuint> &shaderIDs)
{
	// Link the program
	printf("Linking program\n");
	GLuint ProgramID = glCreateProgram();
	for (GLuint shaderID : shaderIDs)
	{
		glAttachShader(ProgramID, shaderID);
	}
	glLinkProgram(ProgramID);

	// Check the program
	GLint Result = GL_FALSE;
	int InfoLogLength;
	glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
	glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
	if (InfoLogLength > 0)
	{
		std::vector<char> ProgramErrorMessage(InfoLogLength + 1);
		glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
		printf("%s\n", &ProgramErrorMessage[0]);
	}
	for (GLuint shaderID : shaderIDs)
	{
		glDetachShader(ProgramID, shaderID);
	}
	return ProgramID;
}

bool replaceIncludes(const std::string& text, const std::map<std::string, std::string>& includeDefinitions, std::string &result, std::string &errorMessage)
{
	std::istringstream f(text);
	std::ostringstream out;
	std::string line;
	int lineIdx = 1;
	while (std::getline(f, line, '\n'))
	{
		if (line.rfind("#include ", 0) == 0) // rfind(., 0) checks whether string start s with the string and returns 0 if it found it otherwise std::string::npos = -1 is returned
		{
			char delimiter = '"';
			size_t n = std::count(line.begin(), line.end(), delimiter);
			if (n != 2)
			{
				errorMessage = "Line " + std::to_string(lineIdx) + ": wrong number of \" characters found (must be 2)";
				return false;
			}
			unsigned first = line.find(delimiter);
			unsigned last = line.find_last_of(delimiter);
			std::string includeName = line.substr(first + 1, last - (first + 1));
			auto iter = includeDefinitions.find(includeName);
			if (iter == includeDefinitions.end())
			{
				errorMessage = "Line " + std::to_string(lineIdx) + ": include '" + includeName + "' found, but not defined in includeDefinitions (make sure spelled correctly)";
				return false;
			}
			out << iter->second << '\n';
		}
		else
		{
			out << line << '\n';
		}
		lineIdx += 1;
	}

	result = out.str();
	return true;
}

bool LoadShaders(const ShaderPipelineDesc& desc, const std::map<std::string, std::string>& includeDefinitions, GLuint& programId)
{
	std::string error;

	std::vector<std::string> shaderTexts(desc.getPartCount());
	for (int i = 0; i < desc.getPartCount(); i++)
	{
		const std::string& filepath = desc.getFilePath(i);
		bool success = get_file_contents(filepath, shaderTexts[i], error);
		if (!success)
		{
			std::cout << "Could not load shader " << filepath << " " << error << std::endl;
			return false;
		}
		std::string result;
		success = replaceIncludes(shaderTexts[i], includeDefinitions, result, error);
		if (!success)
		{
			std::cout << "Could not load shader " << filepath << ", Replace includes resulted in: " << error << std::endl;
			return false;
		}
		shaderTexts[i] = result;
	}

	std::vector<GLuint> shaderIds;
	for (int i = 0; i < desc.getPartCount(); i++)
	{
		GLuint shaderId;
		bool success = compileShader(desc.getShaderType(i), desc.getFilePath(i), shaderTexts[i], shaderId);
		if (!success) return false;
		shaderIds.push_back(shaderId);
	}

	programId = linkProgram(shaderIds);

	for (int i = 0; i < desc.getPartCount(); i++)
	{
		glDeleteShader(shaderIds[i]);
	}

	return true;
}
bool LoadShaders(const ShaderPipelineDesc& desc, GLuint &programId)
{
	std::map<std::string, std::string> includeDefs;
	return LoadShaders(desc, includeDefs, programId);
}

ShaderPipelineDesc ShaderPipelineDesc::VertexFragment(const std::string& vertex_file_path, const std::string& fragment_file_path)
{
	ShaderPipelineDesc result;
	result.m_parts.emplace_back(GL_VERTEX_SHADER, vertex_file_path);
	result.m_parts.emplace_back(GL_FRAGMENT_SHADER, fragment_file_path);
	return result;
}
ShaderPipelineDesc ShaderPipelineDesc::VertexGeometryFragment(const std::string& vertex_file_path, const std::string& geometry_shader_file_path, const std::string& fragment_file_path)
{
	ShaderPipelineDesc result;
	result.m_parts.emplace_back(GL_VERTEX_SHADER, vertex_file_path);
	result.m_parts.emplace_back(GL_GEOMETRY_SHADER, geometry_shader_file_path);
	result.m_parts.emplace_back(GL_FRAGMENT_SHADER, fragment_file_path);
	return result;
}

ProgramGL::ProgramGL(const std::string& vertex_file_path, const std::string& fragment_file_path)
{
	GLuint programId;
	bool success = LoadShaders(ShaderPipelineDesc::VertexFragment(vertex_file_path, fragment_file_path), programId);
	if (!success)
	{
		throw_error("could not load shaders in constructor");
	}
	m_resource.reset(new ProgramGL_resource(programId));
}
ProgramGL ProgramGL::loadFromFiles(const ShaderPipelineDesc& desc, const std::map<std::string, std::string>& includeDefinitions)
{
	GLuint programId;
	bool success = LoadShaders(desc, includeDefinitions, programId);
	if (!success) return ProgramGL();
	return ProgramGL(programId);
}


void ProgramGL::use()
{
	assert(glGetError() == GL_NO_ERROR);
	glUseProgram(m_resource->m_program);
	GLuint error = glGetError();
	assert(error == GL_NO_ERROR);
}
void ProgramGL::unbind()
{
	assert(glGetError() == GL_NO_ERROR);
	glUseProgram(0);
	GLuint error = glGetError();
	assert(error == GL_NO_ERROR);
}