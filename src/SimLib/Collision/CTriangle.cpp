#include "CTriangle.h"
#include <assert.h>
#include <ipc/distance/point_triangle.hpp>

namespace SimOpt
{
	CTriangle::CTriangle(const  Eigen::Vector3d& n1, const Eigen::Vector3d& n2, const Eigen::Vector3d& n3, 
		const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3,
		const Eigen::VectorXi& verticesIndices,
		int elementIndex)
	{
		m_verts = Eigen::MatrixXd(3, 3);
		m_verts.col(0) = v1;
		m_verts.col(1) = v2;
		m_verts.col(2) = v3;

		m_vnorms = Eigen::MatrixXd(3, 3);
		m_vnorms.col(0) = n1;
		m_vnorms.col(1) = n2;
		m_vnorms.col(2) = n3;

		m_trinorm = (n1 + n2 + n3);
		m_trinorm.normalize();

		m_tri_d = (double)(n1.x() * v1.x() + n1.y() * v1.y() + n1.z() * v1.z());
		this->computeMidPoint();

		antiPlane.col(0) = (v2 - v1).cross(m_trinorm);
		antiPlane.col(1) = (v3 - v2).cross(m_trinorm);
		antiPlane.col(2) = (v1 - v3).cross(m_trinorm);

		antiPlane.col(0).normalize();
		antiPlane.col(1).normalize();
		antiPlane.col(2).normalize();

		m_verticesIndices = verticesIndices;
		m_elementIndex = elementIndex;
	}

	CTriangle::CTriangle(const Eigen::Vector3d& face_norm,
		const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3,
		const Eigen::VectorXi& verticesIndices,
		int elementIndex)
	{
		m_verts = Eigen::MatrixXd(3, 3);
		m_verts.col(0) = v1;
		m_verts.col(1) = v2;
		m_verts.col(2) = v3;

		m_vnorms = Eigen::MatrixXd(3, 3);
		m_vnorms.col(0) = face_norm;
		m_vnorms.col(1) = face_norm;
		m_vnorms.col(2) = face_norm;

		m_trinorm = face_norm;
		m_trinorm.normalize();

		m_tri_d = (double)(face_norm.x() * v1.x() + face_norm.y() * v1.y() + face_norm.z() * v1.z());
		this->computeMidPoint();

		antiPlane = Eigen::MatrixXd(3, 3);
		antiPlane.col(0) = (v2 - v1).cross(m_trinorm);
		antiPlane.col(1) = (v3 - v2).cross(m_trinorm);
		antiPlane.col(2) = (v1 - v3).cross(m_trinorm);

		antiPlane.col(0).normalize();
		antiPlane.col(1).normalize();
		antiPlane.col(2).normalize();


		m_verticesIndices = verticesIndices;
		m_elementIndex = elementIndex;
	}

	// distance squared to a point from the tri
	// note SMOD is just a specialised case of a modulation 
	double CTriangle::DistSqrd(const  Eigen::Vector3d& point) const
	{
		double distSqrd = ipc::point_triangle_distance(point, m_verts.col(0), m_verts.col(1), m_verts.col(2));
		return distSqrd;
	}

}