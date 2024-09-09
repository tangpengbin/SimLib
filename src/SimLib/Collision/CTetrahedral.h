#pragma once
#ifndef CTETRAHEDRAL_H
#define CTETRAHEDRAL_H

#include <iostream>
#include <list>
#include <vector>

#include "collisionElement.h"
//#include "AABB.h"
namespace SimOpt
{
	// CTriangle primitive class
	class CTetrahedral : public collisionElement {
	public:
		CTetrahedral() {}
		// make from vert array
		//CTriangle(const float* verts,const float* norms,const int* indices);
		CTetrahedral(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3, const Eigen::Vector3d& v4,
			const Eigen::VectorXi& verticesIndices,
			int elementIndex);
		
		// distance squared to a point from the tri
		double DistSqrd(const Eigen::Vector3d& point) const override;
		
	};
}
#endif //CTRIANGLE_INCLUDED