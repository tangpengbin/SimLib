#pragma once

// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace igl
{
namespace viewer
{

// TODO: write documentation

class ViewerData
{
public:
  ViewerData();

  enum DirtyFlags
  {
    DIRTY_NONE           = 0x0000,
    DIRTY_POSITION       = 0x0001,
    DIRTY_UV             = 0x0002,
    DIRTY_NORMAL         = 0x0004,
    DIRTY_AMBIENT        = 0x0008,
    DIRTY_DIFFUSE        = 0x0010,
    DIRTY_SPECULAR       = 0x0020,
    DIRTY_TEXTURE        = 0x0040,
    DIRTY_FACE           = 0x0080,
    DIRTY_MESH           = 0x00FF,
    DIRTY_OVERLAY_LINES  = 0x0100,
    DIRTY_OVERLAY_POINTS = 0x0200,
    DIRTY_ALL            = 0x03FF
  };

  // Empy all fields
  void clear();

  // Change the visualization mode, invalidating the cache if necessary
  void set_face_based(bool newvalue);

  // Helpers that can draw the most common meshes
  void set_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
  void set_vertices(const Eigen::MatrixXd& V);
  void set_normals(const Eigen::MatrixXd& N);

  // Set the color of the mesh
  //
  // Inputs:
  //   C  #V|#F|1 by 3 list of colors
  void set_colors(const Eigen::MatrixXd &C);
  // Set per-vertex UV coordinates
  //
  // Inputs:
  //   UV  #V by 2 list of UV coordinates (indexed by F)
  void set_uv(const Eigen::MatrixXd& UV);
  // Set per-corner UV coordinates
  //
  // Inputs:
  //   UV_V  #UV by 2 list of UV coordinates
  //   UV_F  #F by 3 list of UV indices into UV_V
  void set_uv(const Eigen::MatrixXd& UV_V, const Eigen::MatrixXi& UV_F);
  // Set the texture associated with the mesh.
  //
  // Inputs:
  //   R  width by height image matrix of red channel
  //   G  width by height image matrix of green channel
  //   B  width by height image matrix of blue channel
  //
  void set_texture(
    const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& R,
    const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& G,
    const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& B);

  // Set the texture associated with the mesh.
  //
  // Inputs:
  //   R  width by height image matrix of red channel
  //   G  width by height image matrix of green channel
  //   B  width by height image matrix of blue channel
  //   A  width by height image matrix of alpha channel
  //
  void set_texture(
    const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& R,
    const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& G,
    const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& B,
    const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& A);

  // Sets points given a list of point vertices. In constrast to `set_points`
  // this will (purposefully) clober existing points.
  //
  // Inputs:
  //   P  #P by 3 list of vertex positions
  //   C  #P|1 by 3 color(s)
  void set_points(
    const Eigen::MatrixXd& P,
    const Eigen::MatrixXd& C);
  void add_points(const Eigen::MatrixXd& P,  const Eigen::MatrixXd& C);
  // Sets edges given a list of edge vertices and edge indices. In constrast
  // to `add_edges` this will (purposefully) clober existing edges.
  //
  // Inputs:
  //   P  #P by 3 list of vertex positions
  //   E  #E by 2 list of edge indices into P
  //   C  #E|1 by 3 color(s)
  void set_edges (const Eigen::MatrixXd& P, const Eigen::MatrixXi& E, const Eigen::MatrixXd& C);
  void add_edges (const Eigen::MatrixXd& P1, const Eigen::MatrixXd& P2, const Eigen::MatrixXd& C);
  void add_label (const Eigen::VectorXd& P,  const std::string& str);

  // Computes the normals of the mesh
  void compute_normals();

  // Assigns uniform colors to all faces/vertices
  void uniform_colors(
    const Eigen::Vector3d& diffuse,
    const Eigen::Vector3d& ambient,
    const Eigen::Vector3d& specular);

  // Assigns uniform colors to all faces/vertices
  void uniform_colors(
    const Eigen::Vector4d& ambient,
    const Eigen::Vector4d& diffuse,
    const Eigen::Vector4d& specular);

  // Generates a default grid texture
  void grid_texture();

  Eigen::MatrixXd V; // Vertices of the current mesh (#V x 3)
  Eigen::MatrixXi F; // Faces of the mesh (#F x 3)

  // Per face attributes
  Eigen::MatrixXd F_normals; // One normal per face

  Eigen::MatrixXd F_material_ambient; // Per face ambient color
  Eigen::MatrixXd F_material_diffuse; // Per face diffuse color
  Eigen::MatrixXd F_material_specular; // Per face specular color

  // Per vertex attributes
  Eigen::MatrixXd V_normals; // One normal per vertex

  Eigen::MatrixXd V_material_ambient; // Per vertex ambient color
  Eigen::MatrixXd V_material_diffuse; // Per vertex diffuse color
  Eigen::MatrixXd V_material_specular; // Per vertex specular color

  // UV parametrization
  Eigen::MatrixXd V_uv; // UV vertices
  Eigen::MatrixXi F_uv; // optional faces for UVs

  // Texture
  Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> texture_R;
  Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> texture_G;
  Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> texture_B;
  Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> texture_A;

  // Overlays

  // Lines plotted over the scene
  // (Every row contains 9 doubles in the following format S_x, S_y, S_z, T_x, T_y, T_z, C_r, C_g, C_b),
  // with S and T the coordinates of the two vertices of the line in global coordinates, and C the color in floating point rgb format
  Eigen::MatrixXd lines;

  // Points plotted over the scene
  // (Every row contains 6 doubles in the following format P_x, P_y, P_z, C_r, C_g, C_b),
  // with P the position in global coordinates of the center of the point, and C the color in floating point rgb format
  Eigen::MatrixXd points;

  // Text labels plotted over the scene
  // Textp contains, in the i-th row, the position in global coordinates where the i-th label should be anchored
  // Texts contains in the i-th position the text of the i-th label
  Eigen::MatrixXd           labels_positions;
  std::vector<std::string>  labels_strings;

  // Marks dirty buffers that need to be uploaded to OpenGL
  uint32_t dirty;

  // Enable per-face or per-vertex properties
  bool face_based;
  /*********************************/
};

}
}

#ifdef ENABLE_SERIALIZATION
#include <igl/serialize.h>
namespace igl {
	namespace serialization {

		inline void serialization(bool s, igl::viewer::ViewerData& obj, std::vector<char>& buffer)
		{
			SERIALIZE_MEMBER(V);
			SERIALIZE_MEMBER(F);

			SERIALIZE_MEMBER(F_normals);
			SERIALIZE_MEMBER(F_material_ambient);
			SERIALIZE_MEMBER(F_material_diffuse);
			SERIALIZE_MEMBER(F_material_specular);

			SERIALIZE_MEMBER(V_normals);
			SERIALIZE_MEMBER(V_material_ambient);
			SERIALIZE_MEMBER(V_material_diffuse);
			SERIALIZE_MEMBER(V_material_specular);

			SERIALIZE_MEMBER(V_uv);
			SERIALIZE_MEMBER(F_uv);

			SERIALIZE_MEMBER(texture_R);
			SERIALIZE_MEMBER(texture_G);
			SERIALIZE_MEMBER(texture_B);
      SERIALIZE_MEMBER(texture_A);

			SERIALIZE_MEMBER(lines);
			SERIALIZE_MEMBER(points);

			SERIALIZE_MEMBER(labels_positions);
			SERIALIZE_MEMBER(labels_strings);

			SERIALIZE_MEMBER(dirty);

			SERIALIZE_MEMBER(face_based);
		}

		template<>
		inline void serialize(const igl::viewer::ViewerData& obj, std::vector<char>& buffer)
		{
			serialization(true, const_cast<igl::viewer::ViewerData&>(obj), buffer);
		}

		template<>
		inline void deserialize(igl::viewer::ViewerData& obj, const std::vector<char>& buffer)
		{
			serialization(false, obj, const_cast<std::vector<char>&>(buffer));
			obj.dirty = igl::viewer::ViewerData::DIRTY_ALL;
		}
	}
}
#endif
