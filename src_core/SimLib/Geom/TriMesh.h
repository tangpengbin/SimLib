#pragma once
#ifndef SO_TRI_MESH_H
#define SO_TRI_MESH_H

#include "Mesh.h"
#include <tiny_obj_loader.h>

class TriMesh : public Mesh
{
public:
	TriMesh();
	TriMesh(const std::vector<Mesh*>& meshes);
	struct Hinge
	{
		Hinge()
		{
			for (int i = 0; i < 2; i++)
			{
				edge[i] = -1;
				flaps[i] = -1;
				tris[i] = -1;
			}
		}
		int edge[2];
		int flaps[2];
		int tris[2];
	};

public:
	bool loadFromFile(const char* szFileName) override;
	bool loadFromFile_obj(const char* szFileName);
	bool loadFromFile_objHardEdge();
	bool loadFromFile_tinyobj(const std::string& filepath, std::string& errorMessage);
	bool writeFile_tinyobj(const std::string& filename);
	void buildHingeStructure();
	void buildCoHingeStructure();
	void buildTriangleSurroundingTriangleStructure();
	void mirror(int mirror_dim);
	void flip(int k, int l);
	//void updateVerticesColors(const Eigen::MatrixXd& verticesColors) { m_verticesColors = verticesColors; }
	const std::vector<Hinge>& hinges() const { return m_hinges; }
	const std::vector<Eigen::Vector2i>& hardHinge_vIndices() const { return m_hardHinge_vIndices; }
	const auto& face_ordered_FaceV_surroundingV() const { return m_face_ordered_FaceV_surroundingV; }
	void const computeIsolatedMesh(Eigen::MatrixXd& isolatedVertices, Eigen::MatrixXi& isolatedFaces, Eigen::MatrixXd& isolatedVerticesColors);

	static void combineMeshes(const std::vector<Eigen::VectorXd>& vertices, const std::vector<std::vector<int>>& faces, Eigen::MatrixXd& output_vertices, Eigen::MatrixXi& output_faces);
	static void combineMeshes(const std::vector<Eigen::MatrixXd>& vertices, const std::vector<Eigen::MatrixXi>& faces, Eigen::MatrixXd& output_vertices, Eigen::MatrixXi& output_faces);

public:

	//use tiny obj for loading and writing
	std::vector<tinyobj::material_t> m_tinyMaterials;
	tinyobj::attrib_t m_attrib;
	std::vector<tinyobj::shape_t> m_shapes;
	string m_meshFileName;
private:
	std::vector<Hinge> m_hinges;
	std::vector<std::pair<Eigen::Vector3i, Eigen::Vector3i>> m_face_ordered_FaceV_surroundingV;
	std::vector<Eigen::Vector2i> m_hardHinge_vIndices;//harder hinges with two vertices

	//Eigen::MatrixXd m_verticesColors;

};

#endif
