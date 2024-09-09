#include "ShaderCache.h"

#include <iostream>
#include "../Core/File.h"

ShaderCache::ShaderCache()
{

}
void ShaderCache::reload()
{
	for (auto& kvp : m_replacementFilepath)
	{
		std::string contents;
		std::string filepath = kvp.second[0];
		std::string errorMessage;
		bool success = get_file_contents(filepath, contents, errorMessage);
		if (!success)
		{
			std::cout << "ShaderCache could not reload include file path " << errorMessage << ':' << filepath << std::endl;
		}
		else
		{
			std::array<std::string, 2> a = { filepath, contents };
			m_replacementFilepath.find(kvp.first)->second = a; // this shouldnt change structure of map.... (while iterating over it)
			m_replacementTextsCombined[kvp.first] = contents;
		}
	}
	for (auto& kvp : m_programs)
	{
		const ShaderPipelineDesc& a = kvp.first;
		kvp.second.getResource();

		GLuint programId;
		bool success = LoadShaders(a, m_replacementTextsCombined, programId);
		if (!success)
		{
			std::cout << " could not reload shader " << std::endl;
		}
		else
		{
			kvp.second.getResource()->update(programId);
		}
	}
}
ProgramGL ShaderCache::getProgram(const ShaderPipelineDesc& args)
{
	auto iter = m_programs.find(args);
	if (iter != m_programs.end()) return iter->second;

	ProgramGL program = ProgramGL::loadFromFiles(args, m_replacementTextsCombined);

	m_programs.emplace(args, program);

	return program;
}
ProgramGL ShaderCache::getProgram(const std::string &vs, const std::string &fs)
{
	ShaderPipelineDesc args = ShaderPipelineDesc::VertexFragment(vs, fs);

	return getProgram(args);
}
ProgramGL ShaderCache::getProgram(const std::string &vs, const std::string &gs, const std::string &fs)
{
	ShaderPipelineDesc args = ShaderPipelineDesc::VertexGeometryFragment(vs, gs, fs);

	return getProgram(args);
}
void ShaderCache::addIncludeText(const std::string& includeName, const std::string& text)
{
	if (!m_programs.empty())
	{
		std::cout << " WARNING: ShaderCache: include texts changed after shaders were already loaded " << std::endl;
	}
	m_replacementTexts.emplace(includeName, text);
	m_replacementTextsCombined.emplace(includeName, text);
}
void ShaderCache::addIncludeFile(const std::string& includeName, const std::string& filepath)
{
	if (!m_programs.empty())
	{
		std::cout << " WARNING: ShaderCache: include paths changed after shaders were already loaded " << std::endl;
	}
	std::string contents;
	std::string errorMessage;
	bool success = get_file_contents(filepath, contents, errorMessage);
	if (!success)
	{
		std::cout << "ShaderCache: could not laod include file " << errorMessage << ':' << filepath << std::endl;
		return;
	}
	std::array<std::string, 2> a = { filepath, contents };
	m_replacementFilepath.emplace(includeName, a);
	m_replacementTextsCombined.emplace(includeName, contents);
}