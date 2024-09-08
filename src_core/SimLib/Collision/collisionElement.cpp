#include "collisionElement.h"

namespace SimOpt
{
	// get the aabb for this CTriangle (used for making group aabb)
	AABB  collisionElement::computeAABB()  const
	{
		// new aabb with cal_min as minimum of 3 verts and maximum cal_max of 3 verts
		Eigen::Vector3d max = m_verts.rowwise().maxCoeff();
		Eigen::Vector3d min = m_verts.rowwise().minCoeff();

		return  AABB(min, max);
	}

	void collisionElement::computeMidPoint()
	{
		midPoint.setZero();
		for (int i = 0; i < m_verts.cols(); i++)
		{
			midPoint += m_verts.col(i);
		}

		midPoint /= double(m_verts.size());
	}


}