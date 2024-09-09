#pragma once
#ifndef COLLISION_ELEMENT_H
#define COLLISION_ELEMENT_H

#include <Eigen/Dense>
#include "AABB.h"
namespace SimOpt {

	class collisionElement
	{

	public:
		collisionElement() { midPoint.setZero(); };
		~collisionElement() {};
	public:
		//col is a vertex
		Eigen::MatrixXd m_verts; // vertices 
		Eigen::Vector3d midPoint;
		Eigen::VectorXi m_verticesIndices;//the indices for all its vertices
		int m_elementIndex;//the index for this collision element

		virtual AABB computeAABB()  const;
		void computeMidPoint();

		Eigen::Vector3d MidPoint() const { return midPoint; }
		Eigen::MatrixXd& getM_verts() { return m_verts; };
		const Eigen::MatrixXd& getM_verts() const { return m_verts; };
		const Eigen::Vector3d& operator[](unsigned int i) const { return m_verts.col(i); }
		const Eigen::VectorXi& getVerticesIndices() const { return m_verticesIndices; }
		
		virtual double DistSqrd(const Eigen::Vector3d& point) const = 0;
	};

}
#endif