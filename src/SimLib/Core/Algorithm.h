#pragma once

#include <algorithm>
#include <vector>

namespace SimOpt {

template <class Type, class UnaryPredicate>
void unique(std::vector<Type> &values,
	UnaryPredicate pred)
{
	auto result = std::unique(values.begin(), values.end(), pred);
	values.resize(result - values.begin());
}

template <class Type, class UnaryPredicate>
void remove_if(std::vector<Type> &values,
	UnaryPredicate pred)
{
	auto first = values.begin();
	auto last = values.end();
	auto result = first;
	while (first != last)
	{
		if (!pred(*first))
		{
			*result = *first;
			++result;
		}
		++first;
	}
	values.resize(result - values.begin());
}

template<typename T, int N>
std::array<T, N> make_array(std::initializer_list<T> l)
{
	std::array<T, N> arr = l;
	return arr;
}

}
