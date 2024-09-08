// This header file is free for any kind of use commercial or non by any party.

// Author: Conor Stokes
// Purpose: Simple geometry primitive operations
#pragma once
#ifndef _aabb_H
#define _aabb_H

#include <iostream>
#include <list>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

// AABB primitive class
namespace SimOpt
{
	class collisionElement;

	class AABB
	{
		Eigen::Vector3d m_bounds[2]; // box bounds - cal_min followed by cal_max 
	public:
		// default null constructor
		AABB() {}
		// box from cal_min and cal_max points
		AABB(const Eigen::Vector3d& boxmin, const Eigen::Vector3d& boxmax) { m_bounds[0] = boxmin; m_bounds[1] = boxmax; }
		// calc aabb from trilist
		AABB(const std::vector<collisionElement*>& triangles);

		void Build(const std::vector<collisionElement*>& triangles);
		// returns a sub box of the octant specified, with the partition as the new corner. 
		double DistSqrd(const Eigen::Vector3d& point) const;
		// calculate the floating point error metric 
		double ErrorMetric() const;
		// intersection scalar (used for weighting in building aabb)
		unsigned int LongestAxis() const;
		// mid-point
		Eigen::Vector3d MidPoint() const { return (m_bounds[0] + m_bounds[1]) * .5f; }
		// copy 
		void Cpy(const AABB& box) { m_bounds[0] = box.m_bounds[0]; m_bounds[1] = box.m_bounds[1]; }
		// make this box encompass the current box as well as this one - returns * this
		void Encompass(const AABB& encbox);
		// render this box
		void Render() const;
		const Eigen::Vector3d* getM_bounds() const { return m_bounds; };
	};
}
#endif