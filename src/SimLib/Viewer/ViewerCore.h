#pragma once

// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "../Viewer/OpenGL_state.h"
//#include <igl/opengl/glfw/TextRenderer.h>
#include "../Viewer/ViewerData.h"

#include <igl/igl_inline.h>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "../Viewer/Camera.h"

namespace SimOpt
{

// Basic class of the 3D mesh viewer
// TODO: write documentation

class ViewerCore
{
public:
	ViewerCore();

	// Initialization
	void init();

	// Shutdown
	void shut();

	// Serialization code
	//void InitSerialization();


	const Eigen::Matrix4f& getViewMatrix() const
	{
		return camera.getViewMatrix();
	}
	const Eigen::Matrix4f& getProjectionMatrix() const
	{
		return camera.getProjectionMatrix();
	}
	Eigen::Vector3f computeCameraCenter() const
	{
		return camera.computeCenter();
	}
	const Eigen::Vector3f& getCameraTrackballCenter() const
	{
		return camera.getTrackballCenter();
	}

	// ------------------- Camera control functions

	// Adjust the view to see the entire model
	void align_camera_center(
		const Eigen::MatrixXf& V);

	void setCameraTrackballRadius(double radius);
	void setCameraTrackballCenter(const Eigen::Vector3f &center);
	//columns are vertices
	void setCameraByMesh(const Eigen::MatrixXd &mesh, double minTrackBallRadius);

	void zoom(float multiplier);

	// ------------------- Drawing functions

	// Clear the frame buffers
	void clear_framebuffers();

	void init_draw();
	void draw(igl::viewer::ViewerData& data, igl::viewer::OpenGL_state& opengl,
		const Eigen::Matrix4f &modelMatrix);


	// ------------------- Properties

	// Text rendering helper
	//igl::viewer::TextRenderer textrenderer;

	// Shape material
	float shininess;

	// Colors
	Eigen::Vector4f background_color;
	Eigen::Vector4f line_color;

	// Lighting
	Eigen::Vector3f light_position;
	float lighting_factor;

	// Model viewing parameters

	// Model viewing paramters (uv coordinates)
	float model_zoom_uv;
	Eigen::Vector3f model_translation_uv;

	// Visualization options
	bool show_overlay;
	bool show_overlay_depth;
	bool show_texture;
	bool show_faces;
	bool show_lines;
	bool show_vertid;
	bool show_faceid;
	bool invert_normals;
	bool depth_test;

	// Point size / line width
	float point_size;
	float line_width;

	// Animation
	bool is_animating;
	double animation_max_fps;

	// Viewport size

	Camera camera;

	void setViewport(const Eigen::Vector4f &view)
	{
		assert(view[0] == 0.0);
		assert(view[1] == 0.0);
		camera.setViewport(view(2), view(3));
	}

	Eigen::Vector4f getViewport() const
	{
		return camera.getViewport();
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#ifdef ENABLE_SERIALIZATION
#include <igl/serialize.h>
namespace igl
{
namespace serialization
{

inline void serialization(bool s, igl::viewer::ViewerCore& obj, std::vector<char>& buffer)
{
	SERIALIZE_MEMBER(shininess);

	SERIALIZE_MEMBER(background_color);
	SERIALIZE_MEMBER(line_color);

	SERIALIZE_MEMBER(light_position);
	SERIALIZE_MEMBER(lighting_factor);

	SERIALIZE_MEMBER(trackball_angle);
	SERIALIZE_MEMBER(rotation_type);

	SERIALIZE_MEMBER(model_zoom);
	SERIALIZE_MEMBER(model_translation);

	SERIALIZE_MEMBER(model_zoom_uv);
	SERIALIZE_MEMBER(model_translation_uv);

	SERIALIZE_MEMBER(camera_zoom);
	SERIALIZE_MEMBER(orthographic);
	SERIALIZE_MEMBER(camera_view_angle);
	SERIALIZE_MEMBER(camera_dnear);
	SERIALIZE_MEMBER(camera_dfar);
	SERIALIZE_MEMBER(camera_eye);
	SERIALIZE_MEMBER(camera_center);
	SERIALIZE_MEMBER(camera_up);

	SERIALIZE_MEMBER(show_faces);
	SERIALIZE_MEMBER(show_lines);
	SERIALIZE_MEMBER(invert_normals);
	SERIALIZE_MEMBER(show_overlay);
	SERIALIZE_MEMBER(show_overlay_depth);
	SERIALIZE_MEMBER(show_vertid);
	SERIALIZE_MEMBER(show_faceid);
	SERIALIZE_MEMBER(show_texture);
	SERIALIZE_MEMBER(depth_test);

	SERIALIZE_MEMBER(point_size);
	SERIALIZE_MEMBER(line_width);
	SERIALIZE_MEMBER(is_animating);
	SERIALIZE_MEMBER(animation_max_fps);

	SERIALIZE_MEMBER(object_scale);

	SERIALIZE_MEMBER(viewport);
	SERIALIZE_MEMBER(view);
	SERIALIZE_MEMBER(model);
	SERIALIZE_MEMBER(proj);
}

template<>
inline void serialize(const igl::viewer::ViewerCore& obj, std::vector<char>& buffer)
{
	serialization(true, const_cast<igl::viewer::ViewerCore&>(obj), buffer);
}

template<>
inline void deserialize(igl::viewer::ViewerCore& obj, const std::vector<char>& buffer)
{
	serialization(false, obj, const_cast<std::vector<char>&>(buffer));
}
}
}
#endif
