#include "../Geom/Normals.h"

#include <Eigen/Geometry>

template <typename Scalar>
void perVertexNormalsS(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& vertices,
	const Eigen::MatrixXi& triangles,
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& normals,
	WeightType::Type weightType)
{
	typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

	normals.setZero(vertices.rows(), vertices.cols());
	std::vector<Scalar> weightSum(vertices.cols(), Scalar(0.0) );

	for (int tri = 0; tri < triangles.cols(); tri++)
	{
		std::array<int, 3> vIdcs = { triangles(0, tri), triangles(1, tri), triangles(2, tri) };
		std::array<Vector3, 3> v;
		for (int i = 0; i < 3; i++)
		{
			if (vIdcs[i] < 0 || vIdcs[i] >= vertices.cols())
			{
				throw std::logic_error("err");
			}
			v[i] = vertices.col(vIdcs[i]);
		}
		Vector3 normal = (v[1] - v[0]).cross(v[2] - v[0]);
		Scalar twiceArea = normal.norm();
		normal /= twiceArea;
		for (int i = 0; i < 3; i++)
		{
			Vector3 e1 = v[(i + 1) % 3] - v[i];
			Vector3 e2 = v[(i + 2) % 3] - v[i];
			Scalar angleBetween = std::acos(std::min(Scalar(1.0), std::max(Scalar(-1.0), e1.normalized().dot(e2.normalized()))));
			Scalar weight;
			switch (weightType)
			{
			case WeightType::CONSTANT:
				weight = 1.0;
				break;
			case WeightType::AREA:
				weight = twiceArea;
				break;
			case WeightType::ANGLE:
				weight = angleBetween;
				break;
			case WeightType::ANGLE_AREA:
				weight = twiceArea * angleBetween;
				break;
			default: assert(false);
			}
			//double weight = 1.0;
			Vector3 wn = weight * normal;
			normals.col(vIdcs[i]) += wn;
			weightSum[vIdcs[i]] += weight;
		}
	}

	for (int i = 0; i < vertices.cols(); i++)
	{
		if (weightSum[i] != Scalar(0.0))
		{
			normals.col(i) /= weightSum[i];
		}
	}
}

void perVertexNormals(const Eigen::MatrixXf& vertices,
	const Eigen::MatrixXi& triangles,
	Eigen::MatrixXf& normals,
	WeightType::Type weightType)
{
	perVertexNormalsS<float>(vertices, triangles, normals, weightType);
}

void perVertexNormals(const Eigen::MatrixXd& vertices,
	const Eigen::MatrixXi& triangles,
	Eigen::MatrixXd& normals,
	WeightType::Type weightType)
{
	perVertexNormalsS<double>(vertices, triangles, normals, weightType);
}
