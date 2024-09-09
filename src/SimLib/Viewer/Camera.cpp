#include "Camera.h"

#include <igl/snap_to_fixed_up.h>
#include <igl/frustum.h>
#include <igl/ortho.h>
#include <igl/PI.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/trackball.h>
#include <igl/project.h>
#include <igl/two_axis_valuator_fixed_up.h>

#include <iostream>

namespace SimOpt
{

Camera::Camera()
{
	m_viewportWidth = 0;
	m_viewportHeight = 0;

	// Temporary variables initialization
	m_mouse_never_moved = true;

	// Default trackball
	m_trackball_angles = Eigen::Quaternionf::Identity();
	setRotationType(ROTATION_TYPE_TRACKBALL);
	//set_rotation_type(ViewerCore::ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP);
	m_trackball_radius = 1.0f;
	m_trackball_center << 0, 0, 0;

	// Camera parameters
	m_orthographic = false;
	m_camera_view_angle = 45.0;
	m_camera_dnear = 1e-3;
	m_camera_dfar = 1e4;
}
void Camera::reset()
{
	m_trackball_angles = Eigen::Quaternionf::Identity();
	setTrackballCenter(Eigen::Vector3f::Zero());
	m_trackball_radius = 1.0f;
}
void Camera::setRotationType(
	const Camera::RotationType & value)
{
	using namespace Eigen;
	using namespace std;
	const RotationType old_rotation_type = value;
	m_rotationType = value;
	if (m_rotationType == ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP &&
		old_rotation_type != ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP)
	{
		igl::snap_to_fixed_up(Quaternionf(m_trackball_angles), m_trackball_angles);
	}
}

void Camera::computeMatrices(float width, float height, bool ortho, Eigen::Matrix4f &viewMat, Eigen::Matrix4f &projMat)
{
	projMat = Eigen::Matrix4f::Identity();

	// Set projection
	if (ortho)
	{
		float length = m_trackball_radius;
		float h = tan(m_camera_view_angle / 360.0 * igl::PI) * (length);
		igl::ortho(-h * width / height, h*width / height, -h, h, m_camera_dnear, m_camera_dfar, projMat);
	}
	else
	{
		float fH = tan(m_camera_view_angle / 360.0 * igl::PI) * m_camera_dnear;
		float fW = fH * (double)width / (double)height;
		igl::frustum(-fW, fW, -fH, fH, m_camera_dnear, m_camera_dfar, projMat);
	}
	// end projection

	// Set view
	viewMat = Eigen::Matrix4f::Identity();
	viewMat.topLeftCorner(3, 3) = m_trackball_angles.toRotationMatrix();

	// Why not just use Eigen::Transform<double,3,Projective> for model...?
	//view.topLeftCorner(3, 3) *= camera_zoom;
	Eigen::Vector3f camera_center = m_trackball_center + m_trackball_radius * (m_trackball_angles.inverse()*Eigen::Vector3f(0.0f, 0.0f, 1.0f));
	viewMat.col(3).head(3) = viewMat.topLeftCorner(3, 3)*-camera_center;
}
void Camera::updateMatrices()
{
	computeMatrices(m_viewportWidth, m_viewportHeight, m_orthographic, m_view, m_proj);
}
void Camera::alignVertexRows(const Eigen::MatrixXf & V)
{
	if (V.rows() == 0 || V.cols() == 0)
	{
		return;
	}

	Eigen::Vector3f min_point = V.rowwise().minCoeff();
	Eigen::Vector3f max_point = V.rowwise().maxCoeff();
	Eigen::Vector3f centroid = 0.5f*(min_point + max_point);
	double size = (max_point - min_point).array().abs().maxCoeff();
	m_trackball_radius = 3 * size;
	m_trackball_center = centroid.cast<float>();
}
void Camera::snapView()
{
	Eigen::Quaternionf snapq = m_trackball_angles;
	igl::snap_to_canonical_view_quat<float, float>(snapq, 1.0, m_trackball_angles);
}

static Eigen::Vector3d unproject(
	const Eigen::Vector4d &viewPort,
	double nearDepth,
	double farDepth,
	const Eigen::Matrix4d &view,
	const Eigen::Matrix4d &proj,
	int x, int y, double z = 0.0
)
{
	//libigl has an unprojection routine, but did not work correctly for us
	// i think the reason is just , that qt and opengl are not using the same window coordinate system! qt uses(0,0, top left, while opengl uses (0,0) bottom left
	// assumption: windowCoordinates are not in viewport coordinates (i.e. (0,0) can be outside of viewport, this changes transformation between ndc and window coordinates))
	Eigen::Vector3d windowCoordinates;
	windowCoordinates << (x + 0.5), (y + 0.5), z;
	Eigen::Vector3d normalizedDeviceCoordinates;
	normalizedDeviceCoordinates[0] = (windowCoordinates[0] - viewPort[2] * 0.5) / (0.5 * viewPort[2]);
	normalizedDeviceCoordinates[1] = (viewPort[3] * 0.5 - windowCoordinates[1]) / (0.5 * viewPort[3]);
	normalizedDeviceCoordinates[2] = (windowCoordinates[2] - 0.5*(farDepth + nearDepth)) / (0.5*(farDepth - nearDepth)); // warning here glDepthRange could change this

	Eigen::Vector4d clipCoordinates;
	clipCoordinates << normalizedDeviceCoordinates, 1.0;
	Eigen::Vector4d viewCoordinates = proj.partialPivLu().solve(clipCoordinates);
	Eigen::Vector4d worldCoordinates = view.partialPivLu().solve(viewCoordinates);
	Eigen::Vector3d p = worldCoordinates.hnormalized();
	return p;
}
void Camera::computeRay(int x, int y, Eigen::Vector3d &raySource, Eigen::Vector3d &rayDir) const
{
	raySource = unproject(
		getViewport().cast<double>(),
		m_camera_dnear,
		m_camera_dfar,
		getViewMatrix().cast<double>(),
		getProjectionMatrix().cast<double>(),
		x, y, 0.0);
	Eigen::Vector3d p2 = unproject(
		getViewport().cast<double>(),
		m_camera_dnear,
		m_camera_dfar,
		getViewMatrix().cast<double>(),
		getProjectionMatrix().cast<double>(),
		x, y, 1.0);
	//raySource = igl::unproject<double>(
	//	Eigen::Vector3d(x, y, 0.0),
	//	getViewMatrix().cast<double>(),
	//	getProjectionMatrix().cast<double>(),
	//	getViewport().cast<double>());
	//Eigen::Vector3d p2 = igl::unproject<double>(
	//	Eigen::Vector3d(x, y, 1.0),
	//	getViewMatrix().cast<double>(),
	//	getProjectionMatrix().cast<double>(),
	//	getViewport().cast<double>());

	rayDir = (p2 - raySource).normalized();
}
Eigen::Vector2d Camera::worldToScreenSpace(const Eigen::Vector3d& position)
{
	Eigen::Vector4d viewport(0, 0, m_viewportWidth, m_viewportHeight);
	Eigen::Vector3d p = igl::project<double>(position, getViewMatrix().cast<double>(), getProjectionMatrix().cast<double>(), viewport);
	return Eigen::Vector2d(p[0], m_viewportHeight - p[1]);
}
float Camera::getDNear() const
{
	return m_camera_dnear;
}
void Camera::setDNear(float v)
{
	m_camera_dnear = v;
}
float Camera::getDFar() const
{
	return m_camera_dfar;
}
void Camera::setDFar(float v)
{
	m_camera_dfar = v;
}
void Camera::mouseDown(int mouse_x, int mouse_y, bool translation)
{
	if (translation)
	{
		mouse_mode = MouseMode::Translation;

	}
	else
	{
		if (getRotationType() == SimOpt::Camera::ROTATION_TYPE_NO_ROTATION)
		{
			//mouse_mode = MouseMode::None;
			return;
		}
		else
		{
			mouse_mode = MouseMode::Rotation;
		}
	}

	// Remember mouse location at m_down even if used by callback/plugin
	m_mouse_down_x = mouse_x;
	m_mouse_down_y = mouse_y;

	m_downTrackballCenter = getTrackballCenter();

	Eigen::Vector3f coord =
		igl::project(
			m_downTrackballCenter,
			getViewMatrix(),
			getProjectionMatrix(),
			getViewport());
	m_mouse_down_z = coord[2];
	m_down_rotation = this->m_trackball_angles;


}
bool Camera::mouseMove(int mouse_x, int mouse_y)
{
	if (m_mouse_never_moved)
	{
		m_mouse_down_x = mouse_x;
		m_mouse_down_y = mouse_y;
		m_mouse_never_moved = false;
	}

	switch (mouse_mode)
	{
	case MouseMode::Rotation:
	{
		switch (getRotationType())
		{
		default:
			assert(false && "Unknown rotation type");
		case SimOpt::Camera::ROTATION_TYPE_NO_ROTATION:
			break;
		case SimOpt::Camera::ROTATION_TYPE_TRACKBALL:
			//std::cout << " mouse move " << mouse_x << ", " << mouse_y << ", track_ball " << m_trackball_angles << std::endl;
			igl::trackball(
				m_viewportWidth,
				m_viewportHeight,
				2.0f,
				m_down_rotation,
				m_mouse_down_x,
				m_mouse_down_y,
				mouse_x,
				mouse_y,
				m_trackball_angles);
			break;
		case SimOpt::Camera::ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP:
			igl::two_axis_valuator_fixed_up(
				m_viewportWidth,
				m_viewportHeight,
				2.0,
				m_down_rotation,
				m_mouse_down_x,
				m_mouse_down_y,
				mouse_x, mouse_y,
				m_trackball_angles);
			break;
		case SimOpt::Camera::ROTATION_TYPE_ONLY_Y:
			igl::trackball( // abuse this function :/ , just fake middle mouse movement in the middle of the screen
				m_viewportWidth,
				m_viewportHeight,
				2.0f,
				m_down_rotation,
				m_mouse_down_x,
				m_viewportHeight / 2.0,
				mouse_x,
				m_viewportHeight / 2.0,
				m_trackball_angles);
			break;
		}

		return true;
	}

	case MouseMode::Translation:
	{
		//translation
		Eigen::Vector3f pos1 = igl::unproject(Eigen::Vector3f(mouse_x, m_viewportHeight - mouse_y, m_mouse_down_z), getViewMatrix(), getProjectionMatrix(), getViewport());
		Eigen::Vector3f pos0 = igl::unproject(Eigen::Vector3f(m_mouse_down_x, m_viewportHeight - m_mouse_down_y, m_mouse_down_z), getViewMatrix(), getProjectionMatrix(), getViewport());

		Eigen::Vector3f diff = pos0 - pos1;
		m_trackball_center = m_downTrackballCenter + Eigen::Vector3f(diff[0], diff[1], diff[2]);

		return true;
	}
	case MouseMode::Zoom:
	{
		//float delta = 0.001f * (mouse_x - m_mouse_down_x + mouse_y - m_mouse_down_y);
		//m_core->camera_zoom *= 1 + delta;
		throw std::logic_error("not implemented");
		m_mouse_down_x = mouse_x;
		m_mouse_down_y = mouse_y;

		return true;
	}

	default:
		return false;
	}
}
void Camera::mouseRelease()
{

	mouse_mode = MouseMode::None;
}

}
