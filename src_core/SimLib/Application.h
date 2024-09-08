#pragma once
#ifndef APPLICATION_H
#define APPLICATION_H
//#define EIGEN_USE_MKL_ALL

#include <Eigen/Core>
#include "Viewer/Camera.h"
#include "Viewer/PrimitiveDrawer.h"
#include "Viewer/ShadowMap.h"
#include <GLFW/glfw3.h>
#include "Viewer/SimpleShadingSettings.h"
#include "Viewer/ObjMesh.h"

class Application
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Application();
	Application(int argc, char* argv[]);
	virtual ~Application();

	virtual void launch(int argc, char* argv[]);

	void setupShader(ProgramGL& program, const Eigen::Matrix4f& modelMatrix);
	void setupShaderWithShadowMap(ProgramGL& program, const Eigen::Matrix4f& modelMatrix);

	PrimitiveDrawer *getPrimitiveDrawer() { return &m_primitiveDrawer; }
	meshDrawer* getMeshDrawer() { return &m_meshDrawer; }
protected:
	virtual void draw_buffer_RGBA(Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& R,
		Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& G,
		Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& B,
		Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& A);
	virtual void keyPressed(int key, int mod);
	virtual void keyReleased(int key, int mod);
	virtual void mouseMove(double xPos, double yPos);
	virtual void mouseButtonPressed(int button, int mods, double xPos, double yPos);
	virtual void mouseButtonReleased(int button, int mods, double xPos, double yPos);
	virtual void scrollWheel(double yOffset);

	virtual bool init(int argc, char* argv[]);
	void saveScreenRecorder();
	void initScreenRecorder();
	virtual void renderShadowMap() {}
	virtual void render() {}
	virtual void ImGuiSetting();
	virtual void runSimulation() {}
	virtual void beforeExit() {}

	PrimitiveDrawer m_primitiveDrawer;
	meshDrawer m_meshDrawer;

	void getWindowSize(int& width, int& height)
	{
		width = 0;
		height = 0;
		if (m_window)
			glfwGetFramebufferSize(m_window, &width, &height);

	}
	ShaderCache& getShaderCache()
	{
		return *m_shaderCache;
	}
	SimOpt::Camera* getCamera()
	{
		return &m_camera;
	}

	int numFrames;
	float m_currentAngle;
	bool m_enableCameraRotatePan;
	bool m_enableCameraRotatePanFix;
	Eigen::Vector3f m_rotateAxis;
	float m_rotatePanFixAngle;
	float m_rotatePanSpeed;//degree
	float m_rotateTrackBallRadius;
	bool m_useOrthogonalDepressionRotationAxis;
	float m_rotatePanDepressionAngle;

	bool m_saveGif;
	uint32_t GIFDelay;

private:

	void initGL();
	void shutGL();
	virtual void draw_buffer(Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& R,
		Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& G,
		Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& B,
		Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& A);

	std::shared_ptr<ShaderCache> m_shaderCache;

	GLFWwindow* m_window;
	Eigen::Vector4f m_clearColor = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f); // Eigen::Vector4f(116.0 / 255.0, 135 / 255.0, 165 / 255.0, 1.0f);
	SimpleShadingSettings m_shadingSettings;
	ShadowMap m_shadowMap;
	SimOpt::Camera m_camera;
	bool enableSRGBConversion = true;

	bool m_enableShadowMap = true;
	bool m_renderShadowMapCamera = false;
	bool m_renderShadowMapFrustum = false;
	SimOpt::Camera m_shadowMapCamera;
	float m_shadowMapCameraDistanceDummy;
	std::unique_ptr<ObjMesh> m_cameraMesh;
	ProgramGL m_program;
	bool m_renderingShadowMap = false;
	SimpleShadingSettings m_shadowMapShadingSettings;
	float m_shadowSamplingScale = 0.01f;
	float m_shadowBias = 1e-5f;


};

#endif