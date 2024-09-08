#pragma once

#include <Eigen/Geometry>

namespace SimOpt
{

class Camera
{
	enum Mode {NONE, PANNING, ROTATING};
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW


	// Trackball angle (quaternion)
	enum RotationType
	{
		ROTATION_TYPE_TRACKBALL = 0,
		ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP,
		ROTATION_TYPE_NO_ROTATION,
		ROTATION_TYPE_ONLY_Y,
		NUM_ROTATION_TYPES
	};

	Camera();
	void reset();

	void setViewMatrix(const Eigen::Matrix4f& viewMatrix)
	{
		m_view = viewMatrix;
	}
	const Eigen::Matrix4f& getViewMatrix() const
	{
		return m_view;
	}

	void setProjectionMatrix(const Eigen::Matrix4f& projectionMatrix)
	{
		m_proj = projectionMatrix;
	}
	const Eigen::Matrix4f& getProjectionMatrix() const
	{
		return m_proj;
	}

	const Eigen::Vector3f& getTrackballCenter() const
	{
		return m_trackball_center;
	}

	void setRotationType(const RotationType & value);
	RotationType getRotationType() const { return m_rotationType; }

	Eigen::Vector3f computeCenter() const
	{
		return m_trackball_center + m_trackball_radius * (m_trackball_angles.inverse()*Eigen::Vector3f(0.0f, 0.0f, 1.0f));
	}

	void setTrackballCenter(const Eigen::Vector3f &c)
	{
		m_trackball_center = c;
	}
	void setTrackballRadius(float radius)
	{
		m_trackball_radius = radius;
	}
	double getTrackballRadius() const
	{
		return m_trackball_radius;
	}

	void computeMatrices(float width, float height, bool orthographic, Eigen::Matrix4f &viewMat, Eigen::Matrix4f &projMat);
	void updateMatrices();
	void zoom(float multiplier)
	{
		m_trackball_radius *= multiplier;
	}
	void alignVertexRows(const Eigen::MatrixXf & V);
	void snapView();
	void computeRay(int x, int y, Eigen::Vector3d &raySource, Eigen::Vector3d &rayDir) const;
	Eigen::Vector2d worldToScreenSpace(const Eigen::Vector3d& position);

	Eigen::Vector4f getViewport() const
	{
		return Eigen::Vector4f(0.f, 0.f, m_viewportWidth, m_viewportHeight);
	}

	void setViewport(double w, double h)
	{
		m_viewportWidth = w;
		m_viewportHeight = h;
	}
	void setOrthographic(bool b)
	{
		m_orthographic = b;
		updateMatrices();
	}
	bool getOrthographic() const
	{
		return m_orthographic;
	}

	const Eigen::Quaternionf& getAngles() const
	{
		return m_trackball_angles;
	}
	void setAngles(const Eigen::Quaternionf& angles)
	{
		m_trackball_angles = angles;
	}

	float getDNear() const;
	void setDNear(float v);
	float getDFar() const;
	void setDFar(float v);

	void mouseDown(int mouse_x, int mouse_y, bool translation);
	bool mouseMove(int mouse_x, int mouse_y);
	void mouseRelease();

protected:
	enum class MouseMode { None, Rotation, Zoom, Translation } mouse_mode;

	RotationType m_rotationType;

	// Save the OpenGL transformation matrices used for the previous rendering pass
	Eigen::Matrix4f m_view;
	Eigen::Matrix4f m_proj;

	// Camera parameters
	bool m_orthographic;

	Eigen::Quaternionf m_trackball_angles;
	float m_trackball_radius;
	Eigen::Vector3f m_trackball_center;

	float m_camera_view_angle;
	float m_camera_dnear;
	float m_camera_dfar;
	
	double m_ref_x;
	double m_ref_y;

	Mode m_mode;

	double m_viewportWidth;
	double m_viewportHeight;

	Eigen::Vector3f m_downTrackballCenter;

	// Temporary data stored when the mouse button is pressed
	Eigen::Quaternionf m_down_rotation;
	int m_mouse_down_x;
	int m_mouse_down_y;
	float m_mouse_down_z;
	bool m_down;
	bool m_mouse_never_moved;
};

}
