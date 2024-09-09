#pragma once
#ifndef SPHERE_INCLUDED
#define SPHERE_INCLUDED


#include <iostream>
#include <list>
#include <vector>

#include <Eigen/Dense>
#include "CTriangle.h"
#include "AABB.h"

namespace SimOpt
{
	// Sphere primitive class
	class Sphere {
		Eigen::Vector3d m_origin, m_lastOrigin;
		double m_radsqrd;
	public:
		// spere from vector and scalar
		Sphere() : m_origin(), m_lastOrigin(), m_radsqrd(0.f) {}
		Sphere(const  Eigen::Vector3d& orgn, double rad) : m_origin(orgn), m_lastOrigin(orgn), m_radsqrd(rad* rad) {}

		bool Intersects(const  collisionElement& tri)
		{
			return (tri.DistSqrd(m_origin) <= m_radsqrd);
		}

		bool Intersects(const AABB& aabb)
		{
			return (aabb.DistSqrd(m_origin) <= m_radsqrd);
		}

		void SetOrigin(double x, double y, double z)
		{
			m_lastOrigin = m_origin;
			m_origin = Eigen::Vector3d(x, y, z);

		}
		void SetRadiusSquare(double radSq) { m_radsqrd = radSq; }
		const Eigen::Vector3d& getOrigin() const { return m_origin; }
		const Eigen::Vector3d& getLastOrigin() const { return m_lastOrigin; }
		const double& getRadius() const { return m_radsqrd; }

	};
}

#endif //SPHERE_INCLUDED