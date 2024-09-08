#include "CRay.h"
#include "AABB.h"
#include "CTriangle.h"

namespace SimOpt
{
	const double kEpsilon = 1e-8;
	void CRay::PrepareData()
	{
		const Eigen::Vector3d dir = (end - start);
		segmax = dir.norm();
		nDir = dir;
		nDir.normalize();
		invDir.x() = 1.0 / nDir.x();
		invDir.y() = 1.0 / nDir.y();
		invDir.z() = 1.0 / nDir.z();

		signs.x() = int(invDir.x() < 0);
		signs.y() = int(invDir.y() < 0);
		signs.z() = int(invDir.z() < 0);
	}

	CRay::CRay()
	{

	}

	CRay::CRay(const Eigen::Vector3d& s, const Eigen::Vector3d& e) : start(s), end(e)
	{
		PrepareData();
	}

	bool CRay::Intersects(const Eigen::Vector3d bounds[2]) const
	{
	//http://people.csail.mit.edu/amy/papers/box-jgt.pdf
		return true;
		double txmin = (bounds	[		signs.x()].x() - start.x()) * invDir.x();
		double txmax = (bounds	[1 -	signs.x()].x() - start.x()) * invDir.x();

		double tymin = (bounds	[		signs.y()].y() - start.y()) * invDir.y();
		double tymax = (bounds	[1 -	signs.y()].y() - start.y()) * invDir.y();

		if ((txmin > tymax) || (tymin > txmax))
			return false;

		if (tymin > txmin)
			txmin = tymin;

		if (tymax < txmax)
			txmax = tymax;


		double tzmin = (bounds[signs.z()].z() - start.z()) * invDir.z();
		double tzmax = (bounds[1 - signs.z()].z() - start.z()) * invDir.z();


		if ((txmin > tzmax) || (tzmin > txmax))
			return false;

		if (tzmin > txmin)
			txmin = tzmin;
		if (tzmax < txmax)
			txmax = tzmax;

		return txmin < segmax&& txmax >= 0.0;
	}

	bool CRay::Intersects(const collisionElement& triangle) const
	{
		/*const Eigen::Vector3d AB = triangle[1] - triangle[0];
		const Eigen::Vector3d AC = triangle[2] - triangle[0];

		const Eigen::Vector3d distanceAR = start - triangle[0];

		const Eigen::Vector3d ACD = AC.cross(nDir);

		const double denom = AB.dot(ACD);
		if (denom == double(0.0)) // Degenerated triangle :(
			return false;
		const double denomScale = double(1.0) / denom;

		const double beta = (distanceAR.dot(ACD)) * denomScale;

		if (beta <= double(0.0) || beta >= double(1.0))
			return false;

		const Eigen::Vector3d ABD = AB.cross(distanceAR);

		const double gamma = (nDir.dot(ABD)) * denomScale;

		if (gamma <= double(0.0) || beta + gamma >= double(1.0))
			return false;

		const double t = -(AC.dot(ABD)) * denomScale;

		const double alpha = double(1.0) - beta - gamma;

		if (t > segmax || t<double(0.0))
			return false;

		return true;*/

		//if(dynamic_cast<const CTriangle*>(&triangle) == nullptr);
		//	return false;

		Eigen::Vector3d out;
		double t;
		return Intersects(triangle, &out, &t);
	}

	bool CRay::Intersects(const collisionElement& triangle, Eigen::Vector3d* out, double* tOut) const
	{
		const CTriangle* triangle_casted = dynamic_cast<const CTriangle*>(&triangle);
		if (triangle_casted == nullptr)
			return false;
		auto triangleVertices = triangle.getM_verts();

		const Eigen::Vector3d AB = triangleVertices.col(0) - triangleVertices.col(1);
		const Eigen::Vector3d AC = triangleVertices.col(0) - triangleVertices.col(2);

		const Eigen::Vector3d distanceAR = triangleVertices.col(0) - start;

		const Eigen::Vector3d ACD = AC.cross(nDir);

		const double denom = AB.dot(ACD);
		if (denom == 0.0) // Degenerated triangle :(
			return false;
		const double denomScale = 1.0 / denom;

		const double beta = (distanceAR.dot(ACD)) * denomScale;

		if (beta <= 0.0 || beta >= 1.0)
			return false;

		const Eigen::Vector3d ABD = AB.cross(distanceAR);

		const double gamma = (nDir.dot(ABD)) * denomScale;

		if (gamma <= 0.0 || beta + gamma >= 1.0)
			return false;

		const double t = -(AC.dot(ABD)) * denomScale;

		if (t > segmax || t < 0.0)
			return false;

		const double alpha = 1.0 - beta - gamma;

		*tOut = t;
		*out = start + nDir * t;

		return true;

		/*
		//https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution
		// compute plane's normal
		Eigen::Vector3d v0v1 = triangleVertices.col(1) - triangleVertices.col(0);
		Eigen::Vector3d v0v2 = triangleVertices.col(2) - triangleVertices.col(0);
		// no need to normalize
		Eigen::Vector3d N = v0v1.cross(v0v2);  //N 
		double area2 = N.norm();

		// Step 1: finding P

		// check if ray and plane are parallel.
		double NdotRayDirection = N.dot(nDir);
		if (fabs(NdotRayDirection) < kEpsilon)  //almost 0 
			return false;  //they are parallel so they don't intersect ! 

		// compute d parameter using equation 2
		double d = -N.dot(triangleVertices.col(0));

		// compute t (equation 3)
		double t = -(N.dot(start) + d) / NdotRayDirection;
		
		*tOut = t;
		// check if the triangle is in behind the ray
		if (t < 0) return false;  //the triangle is behind 

		// compute the intersection point using equation 1
		Eigen::Vector3d P = start + (*tOut) * nDir;

		// Step 2: inside-outside test
		Eigen::Vector3d C;  //vector perpendicular to triangle's plane 

		// edge 0
		Eigen::Vector3d edge0 = triangleVertices.col(1) - triangleVertices.col(0);
		Eigen::Vector3d vp0 = P - triangleVertices.col(0);
		C = edge0.cross(vp0);
		if (N.dot(C) < -kEpsilon) return false;  //P is on the right side 

		// edge 1
		Eigen::Vector3d edge1 = triangleVertices.col(2) - triangleVertices.col(1);
		Eigen::Vector3d vp1 = P - triangleVertices.col(1);
		C = edge1.cross(vp1);
		if (N.dot(C) < -kEpsilon)  return false;  //P is on the right side 

		// edge 2
		Eigen::Vector3d edge2 = triangleVertices.col(0) - triangleVertices.col(2);
		Eigen::Vector3d vp2 = P - triangleVertices.col(2);
		C = edge2.cross(vp2);
		if (N.dot(C) < -kEpsilon) return false;  //P is on the right side; 

		return true;  //this ray hits the triangle */
	}

}