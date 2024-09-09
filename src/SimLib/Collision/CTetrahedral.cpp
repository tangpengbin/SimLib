#include "CTetrahedral.h"
#include <assert.h>
#include <ipc/distance/point_triangle.hpp>
#include <ipc/distance/point_triangle.hpp>

namespace SimOpt
{

	CTetrahedral::CTetrahedral(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3, const Eigen::Vector3d& v4,
		const Eigen::VectorXi& verticesIndices,
		int elementIndex)
	{
		m_verts = Eigen::MatrixXd(3, 4);
		m_verts.col(0) = v1;
		m_verts.col(1) = v2;
		m_verts.col(2) = v3;
		m_verts.col(3) = v4;
		computeMidPoint();

		m_verticesIndices = verticesIndices;
		m_elementIndex = elementIndex;
	}


	double CTetrahedral::DistSqrd(const Eigen::Vector3d& point) const
	{
		auto PointInTetrahedron = [](const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3, const Eigen::Vector3d& v4, const Eigen::Vector3d& p)
		{
			//https://stackoverflow.com/questions/25179693/how-to-check-whether-the-point-is-in-the-tetrahedron-or-not
			auto SameSide = [](const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3, const Eigen::Vector3d& v4, const Eigen::Vector3d& p)
			{
				Eigen::Vector3d normal = (v2 - v1).cross(v3 - v1);
				double dotV4 = normal.dot(v4 - v1);
				double dotP = normal.dot(p - v1);
				//signbit
				return signbit(dotV4) == signbit(dotP);
			};
			return SameSide(v1, v2, v3, v4, p) &&
				SameSide(v2, v3, v4, v1, p) &&
				SameSide(v3, v4, v1, v2, p) &&
				SameSide(v4, v1, v2, v3, p);
		};

		if (!PointInTetrahedron(m_verts.col(0), m_verts.col(1), m_verts.col(2), m_verts.col(3), point))
		{
			//or we directly return infinity is the testing vertex is not in the tet
			return std::numeric_limits<double>::infinity();


			//not in tet, we test triangles
			Eigen::Vector4d distSqrd;
			distSqrd[0] = ipc::point_triangle_distance(point, m_verts.col(0), m_verts.col(1), m_verts.col(2));
			distSqrd[1] = ipc::point_triangle_distance(point, m_verts.col(0), m_verts.col(1), m_verts.col(3));
			distSqrd[2] = ipc::point_triangle_distance(point, m_verts.col(0), m_verts.col(2), m_verts.col(3));
			distSqrd[3] = ipc::point_triangle_distance(point, m_verts.col(1), m_verts.col(2), m_verts.col(3));
			return distSqrd.minCoeff();
		}

		return 0.0;
	}

}