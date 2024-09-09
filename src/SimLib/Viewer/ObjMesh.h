#pragma once

#include <string>
#include "GL.h"
#include "ShaderCache.h"
#include <tiny_obj_loader.h>
#include "Texture.h"

class ObjSubMesh
{
public:
	ObjSubMesh(Eigen::MatrixXf vertices, Eigen::MatrixXi tris, int materialIdx, bool hasTexCoords, Eigen::MatrixXf texCoords);
	ObjSubMesh(const ObjSubMesh &m) = delete;
	int getMaterialIdx() const
	{
		return m_materialIdx;
	}
	void draw()
	{
		m_mesh.draw();
	}
private:
	int m_materialIdx;
	GeneralMeshManaged m_mesh;
};

/** this class is for rendering textures, its common to cut some edges in uv mapping, thus sometimes we duplicate vertices for each face
s.t. we can assign different uv coordinates to a vertex depending on the face that is rendered */
class ObjMesh
{
public:
	ObjMesh(const std::string& filepath);
	bool load(const std::string& filepath, std::string& errorMessage);

	int getMeshCount()  const
	{
		return (int)m_meshes.size();
	}
	std::shared_ptr<TextureManaged> getMeshTexture(int meshIdx) const
	{
		return m_textures[m_meshes[meshIdx]->getMaterialIdx()];
	}
	const tinyobj::material_t& getMeshMaterial(int meshIdx) const
	{
		return m_materials[m_meshes[meshIdx]->getMaterialIdx()];
	}
	void drawMesh(int meshIdx)
	{
		m_meshes[meshIdx]->draw();
	}
private:

	std::vector< std::shared_ptr<ObjSubMesh> > m_meshes;
	std::vector< std::shared_ptr<TextureManaged> > m_textures;
	std::vector<tinyobj::material_t> m_materials;
};

void drawObjMesh(ObjMesh &objMesh, ProgramGL &shader);