#include "Error.h"

#include <iostream>
#include <stdexcept>

void throw_error(const std::string &error)
{
	std::cout << " error " << error << std::endl;
	throw std::logic_error(error);
}
