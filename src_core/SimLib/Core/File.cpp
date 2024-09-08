#include "File.h"

#include <fstream>

bool file_exists(const std::string &filepath)
{
	if (FILE *fileHandle = fopen(filepath.c_str(), "r"))
	{
		fclose(fileHandle);
		return true;
	}
	return false;
}

bool get_file_contents(const std::string&  filename, std::string &contents, std::string &errorMessage)
{
	std::ifstream in(filename, std::ios::in | std::ios::binary);
	if (in)
	{
		in.seekg(0, std::ios::end);
		contents.resize(in.tellg());
		in.seekg(0, std::ios::beg);
		in.read(&contents[0], contents.size());
		in.close();
		return true;
	}
	errorMessage = std::string("could not open file for reading: ") + filename;
	return false;
}
std::string get_file_contents(const char* filename)
{
	std::ifstream in(filename, std::ios::in | std::ios::binary);
	if (in)
	{
		std::string contents;
		in.seekg(0, std::ios::end);
		contents.resize(in.tellg());
		in.seekg(0, std::ios::beg);
		in.read(&contents[0], contents.size());
		in.close();
		return(contents);
	}
	throw(errno);
}
std::string get_folder(const std::string &filepath)
{
	size_t found;
	found = filepath.find_last_of("/\\");
	if (std::string::npos == found) return ".";
	else return std::string(filepath, 0, found);
}
