#pragma once
#ifndef HITRECORD_INCLUDED
#define HITRECORD_INCLUDED
#include <limits>
#include <Eigen/Dense>
#include <float.h>
namespace SimOpt
{
	struct HitRecord
	{
		Eigen::Vector3d position, normal;
		double t; // Along the ray
		void set_zero()
		{
			position.setZero();
			normal.setZero();
			//t = 1000000;
			t = DBL_MAX;
		}
	};
}
#endif //HITRECORD_INCLUDED