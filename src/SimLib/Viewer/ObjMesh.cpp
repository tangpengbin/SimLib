#include "../Viewer/ObjMesh.h"

#include "../Geom/Normals.h"
#include "../Core/Error.h"

#include <tiny_obj_loader.h>
#include <iostream>
#include <iostream>


ObjSubMesh::ObjSubMesh(Eigen::MatrixXf vertices, Eigen::MatrixXi tris, int materialIdx, bool hasTexCoords, Eigen::MatrixXf texCoords)
	:m_materialIdx(materialIdx)
{
	Eigen::MatrixXf normals;
	perVertexNormals(vertices, tris, normals);
	m_mesh.setTriangles(std::move(tris));

	m_mesh.addVertexAttribute(std::move(vertices), "position");
	m_mesh.addVertexAttribute(std::move(normals), "normal");
	if (hasTexCoords)
	{
		m_mesh.addVertexAttribute(std::move(texCoords), "uv");
	}
	m_mesh.buildVAO();
}

ObjMesh::ObjMesh(const std::string& filepath)
{
	std::string errorMessage;
	bool success = load(filepath, errorMessage);
	if (!success)
	{
		throw_error(std::string("could not load ") + filepath + " , error " + errorMessage);
	}
	//some comments regarding blender:
	// ambient color comes in blender 2.82 from the "Metallic" property, see export_obj.py in the blender source
}
bool ObjMesh::load(const std::string& filepath, std::string& errorMessage)
{
	std::string directory = filepath.substr(0, filepath.find_last_of('/') + 1);

	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;

	std::string warning = "";
	errorMessage = "";
	bool ret = tinyobj::LoadObj(&attrib, &shapes, &m_materials, &warning, &errorMessage, filepath.c_str(), directory.c_str(), true);
	if (!ret) return false;

	if (warning != "")
	{
		std::cout << warning << std::endl; // output warning
	}

	for (int i = 0; i < m_materials.size(); i++)
	{
		std::shared_ptr<TextureManaged> texturediffuse;
		if (m_materials[i].diffuse_texname != "")
		{
			texturediffuse.reset(new TextureManaged);
			std::string path = directory + m_materials[i].diffuse_texname;
			bool success = texturediffuse->load(path, true);
			if (!success)
			{
				errorMessage = "could not load texture " + path;
				return false;
			}
			texturediffuse->initGL();
		}
		m_textures.push_back(texturediffuse);
	}

	for (int shapeIdx = 0; shapeIdx < shapes.size(); shapeIdx++)
	{
		tinyobj::shape_t &shape = shapes[shapeIdx];
		int materialId = shape.mesh.material_ids[0];
		for (int j = 0; j < shape.mesh.material_ids.size(); j++)
		{
			if (shape.mesh.material_ids[j] != materialId)
			{
				throw_error("different materials in one object group not supported");
			}
		}

		int triCount = shape.mesh.indices.size() / 3;

		typedef std::array<int, 3> VtxIdentifier; // need to put all necessary properties into "Global" vtx key, s.t. we automatically edge split when necessary

		std::map<VtxIdentifier, int> globalToLocalIdx;
		std::vector<int> localToGlobalVtxIdx;
		int nextLocalVtxIdx = 0;

		//compute "seperated" faces
		Eigen::MatrixXi tris(3, triCount);
		for (int j = 0; j < triCount; j++)
		{
			Eigen::Vector3i face;

			for (int k = 0; k < 3; k++)
			{
				int globalVtxIdx = shape.mesh.indices[3 * j +k].vertex_index;
				int textureCoordinateIdx = shape.mesh.indices[3 * j + k].texcoord_index;
				int normalIdx = shape.mesh.indices[3 * j + k].normal_index;

				VtxIdentifier identifier = { globalVtxIdx , textureCoordinateIdx, normalIdx };
				std::map<VtxIdentifier, int>::iterator iter = globalToLocalIdx.find(identifier);
				if (iter == globalToLocalIdx.end())
				{
					iter = globalToLocalIdx.emplace(identifier, nextLocalVtxIdx).first;
					nextLocalVtxIdx += 1;
					localToGlobalVtxIdx.push_back(globalVtxIdx);
				}
				int localVtxIdx = iter->second;
				face[k] = localVtxIdx;
			}

			tris.col(j) = face;
		}

		//compute corresponding local vtcs
		int objectVtxCount = nextLocalVtxIdx;
		Eigen::MatrixXf vertices(3, objectVtxCount);
		Eigen::MatrixXf texCoords(2, objectVtxCount);
		bool hasTexCoords = true;
		for (auto &kvp : globalToLocalIdx)
		{
			const VtxIdentifier &identifier = kvp.first;
			int localVtxIdx = kvp.second;
			int globalVtxIdx = identifier[0];
			int textureCoordinateIdx = identifier[1];
			int normalIdx = identifier[2];

			vertices.col(localVtxIdx) = Eigen::Vector3f(attrib.vertices[3 * globalVtxIdx + 0], attrib.vertices[3 * globalVtxIdx + 1], attrib.vertices[3 * globalVtxIdx + 2]);

			if (hasTexCoords)
			{
				if (textureCoordinateIdx == -1)
				{
					hasTexCoords = false;
				}
				else
				{
					Eigen::Vector2f texCoordsI(attrib.texcoords[2 * textureCoordinateIdx + 0], attrib.texcoords[2 * textureCoordinateIdx + 1]);
					Eigen::Vector2f texCoordsI_flipped = Eigen::Vector2f(texCoordsI[0], 1.0 - texCoordsI[1]);
					texCoords.col(localVtxIdx) = texCoordsI_flipped;
				}
			}
		}

		m_meshes.emplace_back(new ObjSubMesh(std::move(vertices), std::move(tris), materialId, hasTexCoords, std::move(texCoords)));

		int k = 3;
	}

	return true;
}

#define GL_TEXTURE_MAX_ANISOTROPY_EXT 0x84FE
#define GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT 0x84FF

void drawObjMesh(ObjMesh &objMesh, ProgramGL &shader, int meshIdx)
{
	assert(glGetError() == GL_NO_ERROR);
	const tinyobj::material_t &material = objMesh.getMeshMaterial(meshIdx);
	shader.setUniform("color_ambient", Eigen::Vector3f(material.ambient[0], material.ambient[1], material.ambient[2]));
	shader.setUniform("color_diffuse", Eigen::Vector3f(material.diffuse[0], material.diffuse[1], material.diffuse[2]));
	shader.setUniform("color_specular", Eigen::Vector3f(material.specular[0], material.specular[1], material.specular[2]));
	float roughness = std::sqrt(std::sqrt(2.0 /( material.shininess + 2.0)));
	shader.setUniform1f("roughness", roughness);
	shader.setUniform1f("color_alpha", 1.0);

	assert(glGetError() == GL_NO_ERROR);
	std::shared_ptr<TextureManaged> texture = objMesh.getMeshTexture(meshIdx);
	if (texture)
	{
		assert(glGetError() == GL_NO_ERROR);
		glActiveTexture(GL_TEXTURE0);
		assert(glGetError() == GL_NO_ERROR);
		texture->getGPUTexture().bind();
		assert(glGetError() == GL_NO_ERROR);
		shader.setUniform1i("texture_0", 0);
		assert(glGetError() == GL_NO_ERROR);

		float aniso;
		glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &aniso);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, aniso);
		assert(glGetError() == GL_NO_ERROR);
	}

	objMesh.drawMesh(meshIdx);
}
void drawObjMesh(ObjMesh &objMesh, ProgramGL &shader)
{
	for (int i = 0; i < objMesh.getMeshCount(); i++)
	{
		drawObjMesh(objMesh, shader, i);
	}
}