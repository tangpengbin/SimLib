#pragma once

#include <Eigen/Core>

#include "Program.h"

struct SimpleShadingSettings
{
	SimpleShadingSettings();

	Eigen::Matrix4f viewProjection;
	Eigen::Vector3f cameraPosition;
	Eigen::Vector3f ambientLight;
	Eigen::Vector3f directionalLight;
	Eigen::Vector3f directionalLightDirection;
};


void setupShader(const SimpleShadingSettings &settings, const Eigen::Matrix4f &model, ProgramGL& program);
