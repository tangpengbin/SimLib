#pragma once
#ifndef CRAY_INCLUDED
#define CRAY_INCLUDED

#include <Eigen/Dense>

namespace SimOpt
{
	class collisionElement;
	class CRay
	{
	private:
		Eigen::Vector3d start, end;
		Eigen::Vector3d nDir, invDir;
		Eigen::Vector3i signs;
		double segmax;

		void PrepareData();
	public:
		CRay();
		CRay(const Eigen::Vector3d& start, const Eigen::Vector3d& end);

		bool Intersects(const Eigen::Vector3d bounds[2]) const;
		bool Intersects(const collisionElement& triangle) const;
		bool Intersects(const collisionElement& triangle, Eigen::Vector3d* out, double* t) const;
	};
}
#endif /* RAY_INCLUDED */