#pragma once

// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

// Coverts mesh data inside a igl::viewer::ViewerData class in an OpenGL compatible format
// The class includes a shader and the opengl calls to plot the data

#include "../Viewer/OpenGL_shader.h"
#include "../Viewer/ViewerData.h"

namespace igl
{
namespace viewer
{

class OpenGL_state
{
public:
  typedef unsigned int GLuint;

  GLuint vao_mesh;
  GLuint vao_overlay_lines;
  GLuint vao_overlay_points;
  OpenGL_shader shader_mesh;
  OpenGL_shader shader_overlay_lines;
  OpenGL_shader shader_overlay_points;

  GLuint vbo_V; // Vertices of the current mesh (#V x 3)
  GLuint vbo_V_uv; // UV coordinates for the current mesh (#V x 2)
  GLuint vbo_V_normals; // Vertices of the current mesh (#V x 3)
  GLuint vbo_V_ambient; // Ambient material  (#V x 3)
  GLuint vbo_V_diffuse; // Diffuse material  (#V x 3)
  GLuint vbo_V_specular; // Specular material  (#V x 3)

  GLuint vbo_F; // Faces of the mesh (#F x 3)
  GLuint vbo_tex; // Texture

  GLuint vbo_lines_F;         // Indices of the line overlay
  GLuint vbo_lines_V;         // Vertices of the line overlay
  GLuint vbo_lines_V_colors;  // Color values of the line overlay
  GLuint vbo_points_F;        // Indices of the point overlay
  GLuint vbo_points_V;        // Vertices of the point overlay
  GLuint vbo_points_V_colors; // Color values of the point overlay

  // Temporary copy of the content of each VBO
  Eigen::MatrixXf V_vbo;
  Eigen::MatrixXf V_normals_vbo;
  Eigen::MatrixXf V_ambient_vbo;
  Eigen::MatrixXf V_diffuse_vbo;
  Eigen::MatrixXf V_specular_vbo;
  Eigen::MatrixXf V_uv_vbo;
  Eigen::MatrixXf lines_V_vbo;
  Eigen::MatrixXf lines_V_colors_vbo;
  Eigen::MatrixXf points_V_vbo;
  Eigen::MatrixXf points_V_colors_vbo;

  int tex_u;
  int tex_v;
  Eigen::Matrix<char,Eigen::Dynamic,1> tex;

  Eigen::Matrix<unsigned, Eigen::Dynamic, Eigen::Dynamic> F_vbo;
  Eigen::Matrix<unsigned, Eigen::Dynamic, Eigen::Dynamic> lines_F_vbo;
  Eigen::Matrix<unsigned, Eigen::Dynamic, Eigen::Dynamic> points_F_vbo;

  // Marks dirty buffers that need to be uploaded to OpenGL
  uint32_t dirty;

  // Initialize shaders and buffers
  void init();

  // Release all resources
  void free();

  // Create a new set of OpenGL buffer objects
  void init_buffers();

  // Update contents from a 'Data' instance
  void set_data(const igl::viewer::ViewerData &data, bool invert_normals);

  // Bind the underlying OpenGL buffer objects for subsequent mesh draw calls
  void bind_mesh();

  /// Draw the currently buffered mesh (either solid or wireframe)
  void draw_mesh(bool solid);

  // Bind the underlying OpenGL buffer objects for subsequent line overlay draw calls
  void bind_overlay_lines();

  /// Draw the currently buffered line overlay
  void draw_overlay_lines();

  // Bind the underlying OpenGL buffer objects for subsequent point overlay draw calls
  void bind_overlay_points();

  /// Draw the currently buffered point overlay
  void draw_overlay_points();

  // Release the OpenGL buffer objects
  void free_buffers();

};

}
}

