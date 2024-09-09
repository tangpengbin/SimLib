#pragma once

#include <string>

bool file_exists(const std::string &filepath);
bool get_file_contents(const std::string& filename, std::string &contents, std::string &errorMessage);
std::string get_file_contents(const char* filename);
std::string get_folder(const std::string &filepath);
