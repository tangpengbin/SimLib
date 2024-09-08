#pragma once

#include <string>
#include <vector>

class StringSubsequence
{
public:
	StringSubsequence()
	{
		m_s = nullptr;
	}
	StringSubsequence(const std::string *s, int start, int end)
	{
		m_s = s;
		m_start = start;
		m_end = end;
	}
	void set(const std::string *s, int start, int end)
	{
		m_s = s;
		m_start = start;
		m_end = end;
	}

	char operator[](int i) const
	{
		return (*m_s)[m_start + i];
	}

	int length() const
	{
		return m_end - m_start;
	}

	int startIndex() const
	{
		return m_start;
	}
	int endIndex() const
	{
		return m_end;
	}

	bool contains(char c) const;
	StringSubsequence trim() const;
	std::vector<StringSubsequence> split(char c) const;
	std::string toString() const;
	int toInt() const;
private:
	const std::string *m_s;
	int m_start;
	int m_end;
};
