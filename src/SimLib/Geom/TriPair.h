#pragma once

#include <array>
#include <vector>

namespace SimOpt
{

struct TriPair
{
	std::array<int, 2> tris;
	std::array<int, 4> vtcs; // this is guaranteed in the order of the bending energy definition
};

struct TriKeiborhood
{
	std::vector<int> vtcs; // these are in the order we need for discrete shell strain stuff
};

class TriKeiborhoods
{
public:
	TriKeiborhoods();
	TriKeiborhoods(
		std::vector<TriKeiborhood> keiborhoods
	);

	int size() const
	{
		return (int)getAll().size();
	}

	const TriKeiborhood& operator[](int i) const
	{
		return getAll()[i];
	}

	const std::vector<TriKeiborhood >& getAll() const
	{
		return m_triKeiborhoodsAll;
	}
	const std::vector<std::array<int, 4> >& get4() const
	{
		return m_triKeiborhoods4;
	}
	const std::vector<std::array<int, 5> >& get5() const
	{
		return m_triKeiborhoods5;
	}
	const std::vector<std::array<int, 6> >& get6() const
	{
		return m_triKeiborhoods6;
	}

private:
	std::vector<TriKeiborhood > m_triKeiborhoodsAll;
	std::vector<std::array<int, 4> > m_triKeiborhoods4;
	std::vector<std::array<int, 5> > m_triKeiborhoods5;
	std::vector<std::array<int, 6> > m_triKeiborhoods6;
};

TriKeiborhoods computeTriKeiborhoods(
	const std::vector<std::array<int, 3> > &tris,
	const std::vector<TriPair> &triPairs
);

}
