#pragma once

#include <Eigen/Core>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>

namespace WeightType
{
enum Type
{
	CONSTANT, AREA, ANGLE, ANGLE_AREA
};
}

void perVertexNormals(const Eigen::MatrixXf& vertices,
	const Eigen::MatrixXi& triangles,
	Eigen::MatrixXf& normals,
	WeightType::Type weightType = WeightType::ANGLE_AREA);
void perVertexNormals(const Eigen::MatrixXd& vertices,
	const Eigen::MatrixXi& triangles,
	Eigen::MatrixXd& normals,
	WeightType::Type weightType = WeightType::ANGLE_AREA);
