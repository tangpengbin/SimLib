#pragma once

#include "Program.h"
#include <map>
#include <memory>
#include <array>

class ShaderCache
{
public:

	ShaderCache();

	static ShaderCache& instance()
	{
		static ShaderCache m_instance;
		return m_instance;
	}

	void reload();

	ProgramGL getProgram(const ShaderPipelineDesc& desc);
	ProgramGL getProgram(const std::string& vs, const std::string& fs);
	ProgramGL getProgram(const std::string &vs, const std::string &gs, const std::string &fs);

	void addIncludeText(const std::string& includeName, const std::string& text);
	void addIncludeFile(const std::string& includeName, const std::string& filepath);

private:
	std::map < ShaderPipelineDesc, ProgramGL > m_programs;

	std::map < std::string, std::string > m_replacementTexts; // these are not automatically reloaded
	std::map < std::string, std::array<std::string, 2> > m_replacementFilepath; // these are also automatically reloaded

	std::map< std::string, std::string> m_replacementTextsCombined;
};
