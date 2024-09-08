#pragma once
#ifndef CTRIANGLE_INCLUDED
#define CTRIANGLE_INCLUDED

#include <iostream>
#include <list>
#include <vector>

#include "collisionElement.h"
//#include "AABB.h"
namespace SimOpt
{
	// CTriangle primitive class
	class CTriangle : public collisionElement {
		Eigen::MatrixXd m_vnorms; // vertex normals, for lighting
		Eigen::MatrixXd antiPlane;
		Eigen::Vector3d m_trinorm; // CTriangle normal
		double m_tri_d; // CTriangle's d plane component
	public:
		CTriangle() : m_tri_d(0) {}
		// make from vert array
		//CTriangle(const float* verts,const float* norms,const int* indices);
		CTriangle(const Eigen::Vector3d& n1, const Eigen::Vector3d& n2, const Eigen::Vector3d& n3, 
			const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3,
			const Eigen::VectorXi& verticesIndices,
			int elementIndex);
		CTriangle(const Eigen::Vector3d& face_norm, 
			const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3,
			const Eigen::VectorXi& verticesIndices,
			int elementIndex);
		
		// distance squared to a point from the tri
		double DistSqrd(const Eigen::Vector3d& point) const override;
		
		Eigen::Vector3d getTriangleNormal()const { return m_trinorm; }
		Eigen::MatrixXd& getM_vnorms() { return m_vnorms; };

		//static void RenderBatch(const std::vector<const CTriangle*>& triangles);
	};
}
#endif //CTRIANGLE_INCLUDED