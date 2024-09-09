#include "StringSubsequence.h"

#include <cassert>

bool StringSubsequence::contains(char c) const
{
	for (int i = 0; i < length(); i++)
	{
		if (this->operator[](i) == c) return true;
	}
	return false;
}
StringSubsequence StringSubsequence::trim() const
{
	int start = m_start;
	int end = m_end;
	while (isspace((*m_s)[start]) && start < end)
	{
		start += 1;
	}
	while (isspace((*m_s)[end - 1]) && start < end)
	{
		end -= 1;
	}
	return StringSubsequence(m_s, start, end);
}
std::vector<StringSubsequence> StringSubsequence::split(char c) const
{
	assert(m_end >= m_start);
	std::vector<StringSubsequence> result;
	int start = m_start;
	int end = m_start;
	while (true)
	{
		while ((*m_s)[end] != c && end < m_end)
		{
			end += 1;
		}
		result.emplace_back(m_s, start, end);
		if (end == m_end)
		{
			break;
		}
		start = end + 1; // +1 is save cause end < m_end
		end = start;
	}
	return result;
}
std::string StringSubsequence::toString() const
{
	return std::string(*m_s, (std::size_t)m_start, (std::size_t)length());
}
int StringSubsequence::toInt() const
{
	return std::stoi(toString());
}
