// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "ViewerCore.h"

#include <igl/quat_to_mat.h>
#include <igl/look_at.h>
#include <Eigen/Geometry>
#include <iostream>

namespace SimOpt
{

ViewerCore::ViewerCore()
{
	// Default shininess
	shininess = 35.0f;

	// Default colors
	background_color << 0.95f, 0.95f, 0.95f, 1.0f;
	line_color << 0.0f, 0.0f, 0.0f, 1.0f;

	// Default lights settings
	light_position << -1.0, 1.0f, 1.0f;
	lighting_factor = 1.0f; //on

	// Default visualization options
	show_faces = true;
	show_lines = true;
	invert_normals = false;
	show_overlay = true;
	show_overlay_depth = true;
	show_vertid = false;
	show_faceid = false;
	show_texture = false;
	depth_test = true;

	// Default point size / line width
	point_size = 30;
	line_width = 0.5f;
	is_animating = false;
	animation_max_fps = 30.;

	setViewport(Eigen::Vector4f::Zero());
}

void ViewerCore::init()
{
	//textrenderer.Init();
}

void ViewerCore::shut()
{
	//textrenderer.Shut();
}

void ViewerCore::align_camera_center(
	const Eigen::MatrixXf& V)
{
	camera.alignVertexRows(V);
}
void ViewerCore::setCameraTrackballCenter(const Eigen::Vector3f &center)
{
	camera.setTrackballCenter(center);
}
void ViewerCore::setCameraTrackballRadius(double radius)
{
	camera.setTrackballRadius(radius);
}
void ViewerCore::setCameraByMesh(const Eigen::MatrixXd &mesh, double minTrackBallRadius)
{
	//simple avg approximation now, no linear programming for now
	Eigen::VectorXd mp = mesh.rowwise().sum() / mesh.cols();
	double maxDist = 0.0;
	for (int j = 0; j < mesh.cols(); j++)
	{
		maxDist = std::max(maxDist, (mp - mesh.col(j)).norm());
	}
	double radius = std::max(minTrackBallRadius, 3.0 * maxDist);

	if (mp.size() == 2)
	{
		mp.conservativeResize(3);
		mp[2] = 0.0;
	}

	this->setCameraTrackballCenter(mp.cast<float>());
	this->setCameraTrackballRadius(radius);
}
void ViewerCore::zoom(float multiplier)
{
	camera.zoom(multiplier);
}

void ViewerCore::clear_framebuffers()
{
	glClearColor(background_color[0],
		background_color[1],
		background_color[2],
		1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void ViewerCore::init_draw()
{
	clear_framebuffers();

	if (depth_test)
		glEnable(GL_DEPTH_TEST);
	else
		glDisable(GL_DEPTH_TEST);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	Eigen::Vector4f viewport = getViewport();
	glViewport(viewport(0), viewport(1), viewport(2), viewport(3));

}
void ViewerCore::draw(igl::viewer::ViewerData& data, igl::viewer::OpenGL_state& opengl,
	const Eigen::Matrix4f &modelMatrix)
{
	/* Bind and potentially refresh mesh/line/point data */
	if (data.dirty)
	{
		opengl.set_data(data, invert_normals);
		data.dirty = igl::viewer::ViewerData::DIRTY_NONE;
	}
	opengl.bind_mesh();

	camera.updateMatrices();

	// Send transformations to the GPU
	GLint modeli = opengl.shader_mesh.uniform("model");
	GLint viewi = opengl.shader_mesh.uniform("view");
	GLint proji = opengl.shader_mesh.uniform("proj");
	glUniformMatrix4fv(modeli, 1, GL_FALSE, modelMatrix.data());
	glUniformMatrix4fv(viewi, 1, GL_FALSE, camera.getViewMatrix().data());
	glUniformMatrix4fv(proji, 1, GL_FALSE, camera.getProjectionMatrix().data());

	// Light parameters
	GLint specular_exponenti = opengl.shader_mesh.uniform("specular_exponent");
	GLint light_position_worldi = opengl.shader_mesh.uniform("light_position_world");
	GLint lighting_factori = opengl.shader_mesh.uniform("lighting_factor");
	GLint fixed_colori = opengl.shader_mesh.uniform("fixed_color");
	GLint texture_factori = opengl.shader_mesh.uniform("texture_factor");

	glUniform1f(specular_exponenti, shininess);
	Eigen::Vector3f rev_light = -1.*light_position;
	glUniform3fv(light_position_worldi, 1, rev_light.data());
	glUniform1f(lighting_factori, lighting_factor); // enables lighting
	glUniform4f(fixed_colori, 0.0, 0.0, 0.0, 0.0);

	if (data.V.rows() > 0)
	{
		// Render fill
		if (show_faces)
		{
			// Texture
			glUniform1f(texture_factori, show_texture ? 1.0f : 0.0f);
			opengl.draw_mesh(true);
			glUniform1f(texture_factori, 0.0f);
		}

		// Render wireframe
		if (show_lines)
		{
			glLineWidth(line_width);
			glUniform4f(fixed_colori, line_color[0], line_color[1],
				line_color[2], 1.0f);
			opengl.draw_mesh(false);
			glUniform4f(fixed_colori, 0.0f, 0.0f, 0.0f, 0.0f);
		}

		//if (show_vertid)
		//{
		//	textrenderer.BeginDraw(view*modelMatrix, proj, viewport, object_scale);
		//	for (int i = 0; i < data.V.rows(); ++i)
		//		textrenderer.DrawText(data.V.row(i), data.V_normals.row(i), std::to_string(i));
		//	textrenderer.EndDraw();
		//}

		//if (show_faceid)
		//{
		//	textrenderer.BeginDraw(view*modelMatrix, proj, viewport, object_scale);

		//	for (int i = 0; i < data.F.rows(); ++i)
		//	{
		//		Eigen::RowVector3d p = Eigen::RowVector3d::Zero();
		//		for (int j = 0; j < data.F.cols(); ++j)
		//			p += data.V.row(data.F(i, j));
		//		p /= data.F.cols();

		//		textrenderer.DrawText(p, data.F_normals.row(i), std::to_string(i));
		//	}
		//	textrenderer.EndDraw();
		//}
	}

	if (show_overlay)
	{
		if (show_overlay_depth)
			glEnable(GL_DEPTH_TEST);
		else
			glDisable(GL_DEPTH_TEST);

		if (data.lines.rows() > 0)
		{
			opengl.bind_overlay_lines();
			modeli = opengl.shader_overlay_lines.uniform("model");
			viewi = opengl.shader_overlay_lines.uniform("view");
			proji = opengl.shader_overlay_lines.uniform("proj");

			glUniformMatrix4fv(modeli, 1, GL_FALSE, modelMatrix.data());
			glUniformMatrix4fv(viewi, 1, GL_FALSE, camera.getViewMatrix().data());
			glUniformMatrix4fv(proji, 1, GL_FALSE, camera.getProjectionMatrix().data());
			// This must be enabled, otherwise glLineWidth has no effect
			glEnable(GL_LINE_SMOOTH);
			glLineWidth(line_width);

			opengl.draw_overlay_lines();
		}

		if (data.points.rows() > 0)
		{
			opengl.bind_overlay_points();
			modeli = opengl.shader_overlay_points.uniform("model");
			viewi = opengl.shader_overlay_points.uniform("view");
			proji = opengl.shader_overlay_points.uniform("proj");

			glUniformMatrix4fv(modeli, 1, GL_FALSE, modelMatrix.data());
			glUniformMatrix4fv(viewi, 1, GL_FALSE, camera.getViewMatrix().data());
			glUniformMatrix4fv(proji, 1, GL_FALSE, camera.getProjectionMatrix().data());
			glPointSize(point_size);

			opengl.draw_overlay_points();
		}

		//if (data.labels_positions.rows() > 0)
		//{
		//	textrenderer.BeginDraw(view*modelMatrix, proj, viewport, object_scale);
		//	for (int i = 0; i < data.labels_positions.rows(); ++i)
		//		textrenderer.DrawText(data.labels_positions.row(i), Eigen::Vector3d(0.0, 0.0, 0.0),
		//			data.labels_strings[i]);
		//	textrenderer.EndDraw();
		//}

		glEnable(GL_DEPTH_TEST);
	}

}


}
