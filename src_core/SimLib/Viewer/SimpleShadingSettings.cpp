#include "SimpleShadingSettings.h"

SimpleShadingSettings::SimpleShadingSettings()
{

	ambientLight = Eigen::Vector3f(0.27f, 0.27f, 0.27f);
	directionalLight = Eigen::Vector3f(0.76f, 0.76f, 0.76f);
	//directionalLightDirection = Eigen::Vector3f(-1.0f, -1.0f, -1.0f);
	directionalLightDirection = Eigen::Vector3f(-1.0f, .0f, -1.0f);
}
void setupShader(const SimpleShadingSettings& settings, const Eigen::Matrix4f& model, ProgramGL& program)
{
	Eigen::Matrix3f modelInverseTranspose = model.inverse().transpose().topLeftCorner<3, 3>().cast<float>();

	assert(glGetError() == GL_NO_ERROR);
	program.use();
	program.setUniform("viewProjection", settings.viewProjection);
	program.setUniform("Model", Eigen::Matrix4f(model.cast<float>()));
	program.setUniform("ModelInverseTranspose", modelInverseTranspose);
	program.setUniform("cameraPositionWorld", settings.cameraPosition);
	program.setUniform("ambientLight", settings.ambientLight);
	Eigen::Vector3f lightDirection = settings.directionalLightDirection.normalized();
	program.setUniform("lightDirection", lightDirection);
	program.setUniform("lightColor", settings.directionalLight);
	assert(glGetError() == GL_NO_ERROR);
}
