#pragma once
#ifndef LLT_H
#define LLT_H

#include "LLT_Eigen.h"
namespace SimOpt
{

template<typename _MatrixType, int _Options = Eigen::Upper>
using LLT = LLT_Eigen< _MatrixType, _Options>;

}

#endif