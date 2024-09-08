#include "TriMesh.h"
#include "../Core/SOUtils.h"

//#include <glm/glm.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>

#include "../Core/Error.h"

TriMesh::TriMesh()
{
	m_dirty = true;
	m_flipNorm = false;
}

TriMesh::TriMesh(const std::vector<Mesh*>& meshes) : Mesh(meshes)
{
	m_dirty = true;
	m_flipNorm = false;
}

bool TriMesh::loadFromFile(const char* szFileName)
{
	bool res = loadFromFile_obj(szFileName);
	m_dirty = true;
	m_flipNorm = false;
	return res;
}



bool TriMesh::loadFromFile_obj(const char* szFileName)
{
	m_meshFileName = szFileName;
	bool success = true;

	//check if obj file
	//...


	std::ifstream inFile;
	inFile.open(szFileName);
	if (!inFile)
	{
		printf("Cannot open file %s for reading\n", szFileName);
		return false;
	}
	std::vector<double> vx;
	std::vector<int> vi;
	////read vertices	
	while (!inFile.eof())
	{

		const int buf_size = 1e4;
		char *buf = new char[buf_size];
		inFile.getline(buf, buf_size);
		std::string s(buf);
		if (s.empty())
			continue;

		std::stringstream sstream(s);
		std::string stoken;
		sstream >> stoken;
		if (stoken.compare("v") == 0)
		{
			double x = 0.0;
			double y = 0.0;
			double z = 0.0;
			sstream >> x;
			sstream >> y;
			sstream >> z;

			vx.push_back(x);
			vx.push_back(y);
			vx.push_back(z);

		}
		else if (stoken.compare("f") == 0)
		{
			//read in face data
			std::vector<int> finds;
			//extract all vertices
			while (true)
			{
				//for each vertex, there is a substring vind/tind/nind. vind is mandatory, others are optional.
				//create second string stream to extract indices for first vertex
				std::string s2;
				sstream >> s2;
				if (s2.empty())
					break;

				std::stringstream ss2(s2);
				//only vertex index for now
				int ii = 0;
				ss2 >> ii;
				//obj indices start at 1
				vi.push_back(ii-1);
			}
		}
	}
	assert(vx.size() % 3 == 0);
	soutil::stdToEigenVec(vx, m_x);
	m_f = vi;
	int nfaces = m_f.size() / 3;
	m_faceColors = Eigen::VectorXd(nfaces*3);
	m_faceColors.setZero();
	//m_x = dVector(vx.size());
	//for (int i = 0; i< vx.size(); i++)
	//	m_x(i) = vx[i];
	//m_isFixed = isFixed;


	//inFile.clear();
	//inFile.seekg(0, std::ios::beg);
	//while (!inFile.eof())
	//{
	//	const int buf_size = 1e4;
	//	char *buf = new char[buf_size];
	//	inFile.getline(buf, buf_size);
	//	std::string s(buf);
	//	if (s.empty())
	//		continue;

	//	std::stringstream sstream(s);
	//	std::string stoken;
	//	sstream >> stoken;

	//	if ((stoken.compare("ec") == 0) || (stoken.compare("er") == 0))
	//	{
	//		std::vector<int> inds(2);
	//		sstream >> inds[0];
	//		sstream >> inds[1];
	//		if (stoken.compare("ec") == 0)
	//		{
	//			Cable* cable = new Cable;
	//			m_elements.push_back(cable);

	//			//m_elastics.push_back(EdgeElement());
	//			cable->init(inds, m_x);
	//			//m_elastics.back().setStiffness(m_kElastics);
	//		}
	//		else
	//		{
	//			Strut* strut = new Strut;
	//			m_elements.push_back(strut);
	//			strut->init(inds, m_x);
	//			//m_bars.push_back(EdgeElement());
	//			//m_bars.back().init(inds,m_x);
	//			//m_bars.back().setConstraint(true);
	//		}
	//	}
	//}
	return success;
}

bool TriMesh::loadFromFile_objHardEdge()
{//if we use obj file from origamiSimulator: https://origamisimulator.org/
	if (!m_meshFileName.empty())
	{
		std::ifstream inFile;
		inFile.open(m_meshFileName.c_str());
		if (!inFile)
		{
			printf("Cannot open file %s for reading\n", m_meshFileName.c_str());
			return false;
		}

		m_hardHinge_vIndices.clear();
		////read vertices	
		while (!inFile.eof())
		{

			const int buf_size = 1e4;
			char* buf = new char[buf_size];
			inFile.getline(buf, buf_size);
			std::string s(buf);
			if (s.empty())
				continue;

			std::stringstream sstream(s);
			std::string stoken;
			sstream >> stoken;
			if (stoken.compare("#e") == 0)
			{
				int vi = 0;
				int vj = 0;
				int type_1 = 0;
				int type_2 = 0;
				sstream >> vi;
				sstream >> vj;
				sstream >> type_1;
				sstream >> type_2;

				if (type_1 == 1)
				{
					m_hardHinge_vIndices.emplace_back(vi - 1, vj - 1);
				}

			}
		}
	}

}

bool TriMesh::loadFromFile_tinyobj(const std::string& filepath, std::string& errorMessage)
{
	m_meshFileName = filepath;
	std::string directory = filepath.substr(0, filepath.find_last_of('/') + 1);


	std::string warning = "";
	errorMessage = "";
	bool ret = tinyobj::LoadObj(&m_attrib, &m_shapes, &m_tinyMaterials, &warning, &errorMessage, filepath.c_str(), directory.c_str(), true);
	if (!ret) return false;

	if (warning != "")
	{
		std::cout << warning << std::endl; // output warning
	}

	/*for (int i = 0; i < m_tinyMaterials.size(); i++)
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
	}*/

	for (int shapeIdx = 0; shapeIdx < 1; shapeIdx++)//shapes.size() //now, we only support for single shape
	{
		tinyobj::shape_t& shape = m_shapes[shapeIdx];
		int materialId = shape.mesh.material_ids[0];
		for (int j = 0; j < shape.mesh.material_ids.size(); j++)
		{
			if (shape.mesh.material_ids[j] != materialId)
			{
				throw_error("different materials in one object group not supported");
			}
		}

		int triCount = shape.mesh.indices.size() / 3;

		int nextLocalVtxIdx = 0;

		//compute "seperated" faces
		Eigen::MatrixXi tris(3, triCount);
		for (int j = 0; j < triCount; j++)
		{
			Eigen::Vector3i face;

			for (int k = 0; k < 3; k++)
			{
				face[k] = shape.mesh.indices[3 * j + k].vertex_index;
			}

			tris.col(j) = face;
		}

		//here we only trans vertices and faces
		//Eigen::MatrixXd vertices_d = vertices.cast<double>();
		//m_x = Eigen::Map<Eigen::VectorXd>(vertices_d.data(), vertices_d.size());
		m_x = Eigen::Map<Eigen::VectorXf>(m_attrib.vertices.data(), m_attrib.vertices.size()).cast<double>();
		m_f = std::vector<int>(tris.data(), tris.data() + tris.size());
	}
	int nfaces = m_f.size() / 3;
	m_faceColors = Eigen::VectorXd(nfaces * 3);
	m_faceColors.setZero();

	return true;
}



static std::string GetFileBasename(const std::string& FileName)
{
	if (FileName.find_last_of(".") != std::string::npos)
		return FileName.substr(0, FileName.find_last_of("."));
	return "";
}

bool WriteMat(const std::string& filename, const std::vector<tinyobj::material_t>& materials) {
	FILE* fp = fopen(filename.c_str(), "w");
	if (!fp) {
		fprintf(stderr, "Failed to open file [ %s ] for write.\n", filename.c_str());
		return false;
	}

	for (size_t i = 0; i < materials.size(); i++) {

		tinyobj::material_t mat = materials[i];

		fprintf(fp, "newmtl %s\n", mat.name.c_str());
		fprintf(fp, "Ka %f %f %f\n", mat.ambient[0], mat.ambient[1], mat.ambient[2]);
		fprintf(fp, "Kd %f %f %f\n", mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
		fprintf(fp, "Ks %f %f %f\n", mat.specular[0], mat.specular[1], mat.specular[2]);
		fprintf(fp, "Kt %f %f %f\n", mat.transmittance[0], mat.specular[1], mat.specular[2]);
		fprintf(fp, "Ke %f %f %f\n", mat.emission[0], mat.emission[1], mat.emission[2]);
		fprintf(fp, "Ns %f\n", mat.shininess);
		fprintf(fp, "Ni %f\n", mat.ior);
		fprintf(fp, "illum %d\n", mat.illum);
		fprintf(fp, "map_Kd %s", mat.diffuse_texname.c_str());
		fprintf(fp, "\n");
		// @todo { texture }
	}

	fclose(fp);

	return true;
}

bool WriteObj(const std::string& filename, const tinyobj::attrib_t& attributes, const std::vector<tinyobj::shape_t>& shapes, const std::vector<tinyobj::material_t>& materials, bool coordTransform) {
	FILE* fp = fopen(filename.c_str(), "w");
	if (!fp) {
		fprintf(stderr, "Failed to open file [ %s ] for write.\n", filename.c_str());
		return false;
	}

	std::string basename = GetFileBasename(filename);
	std::string material_filename = basename + ".mtl";

	int prev_material_id = -1;

	fprintf(fp, "mtllib %s\n\n", material_filename.c_str());

	// facevarying vtx
	for (size_t k = 0; k < attributes.vertices.size(); k += 3) {
		if (coordTransform) {
			fprintf(fp, "v %f %f %f\n",
				attributes.vertices[k + 0],
				attributes.vertices[k + 2],
				-attributes.vertices[k + 1]);
		}
		else {
			fprintf(fp, "v %f %f %f\n",
				attributes.vertices[k + 0],
				attributes.vertices[k + 1],
				attributes.vertices[k + 2]);
		}
	}

	fprintf(fp, "\n");

	// facevarying normal
	for (size_t k = 0; k < attributes.normals.size(); k += 3) {
		if (coordTransform) {
			fprintf(fp, "vn %f %f %f\n",
				attributes.normals[k + 0],
				attributes.normals[k + 2],
				-attributes.normals[k + 1]);
		}
		else {
			fprintf(fp, "vn %f %f %f\n",
				attributes.normals[k + 0],
				attributes.normals[k + 1],
				attributes.normals[k + 2]);
		}
	}

	fprintf(fp, "\n");

	// facevarying texcoord
	for (size_t k = 0; k < attributes.texcoords.size(); k += 2) {
		fprintf(fp, "vt %f %f\n",
			attributes.texcoords[k + 0],
			attributes.texcoords[k + 1]);
	}

	for (size_t i = 0; i < shapes.size(); i++) {
		fprintf(fp, "\n");

		if (shapes[i].name.empty()) {
			fprintf(fp, "g Unknown\n");
		}
		else {
			fprintf(fp, "g %s\n", shapes[i].name.c_str());
		}

		bool has_vn = false;
		bool has_vt = false;
		// Assumes normals and textures are set shape-wise.
		if (shapes[i].mesh.indices.size() > 0) {
			has_vn = shapes[i].mesh.indices[0].normal_index != -1;
			has_vt = shapes[i].mesh.indices[0].texcoord_index != -1;
		}

		// face
		int face_index = 0;
		for (size_t k = 0; k < shapes[i].mesh.indices.size(); k += shapes[i].mesh.num_face_vertices[face_index++]) {
			// Check Materials
			int material_id = shapes[i].mesh.material_ids[face_index];
			if (material_id != prev_material_id) {
				std::string material_name = materials[material_id].name;
				fprintf(fp, "usemtl %s\n", material_name.c_str());
				prev_material_id = material_id;
			}

			unsigned char v_per_f = shapes[i].mesh.num_face_vertices[face_index];
			// Imperformant, but if you want to have variable vertices per face, you need some kind of a dynamic loop.
			fprintf(fp, "f");
			for (int l = 0; l < v_per_f; l++) {
				const tinyobj::index_t& ref = shapes[i].mesh.indices[k + l];
				if (has_vn && has_vt) {
					// v0/t0/vn0
					fprintf(fp, " %d/%d/%d", ref.vertex_index + 1, ref.texcoord_index + 1, ref.normal_index + 1);
					continue;
				}
				if (has_vn && !has_vt) {
					// v0//vn0
					fprintf(fp, " %d//%d", ref.vertex_index + 1, ref.normal_index + 1);
					continue;
				}
				if (!has_vn && has_vt) {
					// v0/vt0
					fprintf(fp, " %d/%d", ref.vertex_index + 1, ref.texcoord_index + 1);
					continue;
				}
				if (!has_vn && !has_vt) {
					// v0 v1 v2
					fprintf(fp, " %d", ref.vertex_index + 1);
					continue;
				}
			}
			fprintf(fp, "\n");
		}
	}

	fclose(fp);

	//
	// Write material file
	//
	bool ret = WriteMat(material_filename, materials);

	return ret;
}

bool TriMesh::writeFile_tinyobj(const std::string& filename)
{
	//update vertices in m_attrib
	for (int i = 0; i < m_x.size() / 3; i++)
	{
		for (int d = 0; d < 3; d++)
			m_attrib.vertices[3 * i + d] = float(m_x[3 * i + d]);
	}

	WriteObj(filename, m_attrib, m_shapes, m_tinyMaterials, false);

	//bool ret = tinyobj::LoadObj(&m_attrib, &m_shapes, &m_tinyMaterials, &warning, &errorMessage, filepath.c_str(), directory.c_str(), true);
	return false;
}



void TriMesh::buildHingeStructure()
{
	m_hinges.clear();
	int ntris = numTriangles();
	std::map<std::pair<int, int>, int> edge2index;
	for (int i = 0; i < ntris; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			int i1 = m_f[3 * i + j];
			int i2 = m_f[3 * i + (j + 1) % 3];
			int i1t = i1;
			int i2t = i2;
			bool swapped = false;
			if (i1t > i2t)
			{
				std::swap(i1t, i2t);
				swapped = true;
			}
			
			auto ei = std::make_pair(i1t, i2t);
			auto ite = edge2index.find(ei);
			if (ite == edge2index.end())
			{
				//insert new hinge
				edge2index[ei] = m_hinges.size();
				m_hinges.push_back(Hinge());
				Hinge& hinge = m_hinges.back();
				hinge.edge[0] = i1t;
				hinge.edge[1] = i2t;
				int itmp = swapped ? 1 : 0;
				hinge.tris[itmp] = i;
				hinge.flaps[itmp] = m_f[3 * i + (j + 2) % 3];
			}
			else
			{
				//hinge for this edge already exists, add missing information for this triangle
				Hinge& hinge = m_hinges[ite->second];
				//printf("edge %d %d, %d %d-->", hinge.edge[0], hinge.edge[1], hinge.flaps[0], hinge.flaps[1]);
				int itmp = swapped ? 1 : 0;
				hinge.tris[itmp] = i;
				hinge.flaps[itmp] = m_f[3 * i + (j + 2) % 3];
				//printf("%d %d\n", hinge.flaps[0], hinge.flaps[1]);
			}
		}
	}
	//ordering for edges

}

void TriMesh::buildCoHingeStructure()
{
	//This function can compute several hinges attched into one edge
	//This will return all hinges
	//E.g. this structure _|_ , this function will return 3 hinges
	struct CoHinge
	{
		CoHinge()
		{
			for (int i = 0; i < 2; i++)
			{
				edge[i] = -1;
			}
		}
		int edge[2];
		std::vector<int> flaps;
		std::vector<int> tris;
	};

	m_hinges.clear();
	std::vector<CoHinge> m_cohinges;
	int ntris = numTriangles();
	std::map<std::pair<int, int>, int> edge2index;
	for (int i = 0; i < ntris; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			int i1 = m_f[3 * i + j];
			int i2 = m_f[3 * i + (j + 1) % 3];
			int i1t = i1;
			int i2t = i2;
			bool swapped = false;
			if (i1t > i2t)
			{
				std::swap(i1t, i2t);
				swapped = true;
			}

			auto ei = std::make_pair(i1t, i2t);
			auto ite = edge2index.find(ei);
			if (ite == edge2index.end())
			{
				//insert new hinge
				edge2index[ei] = m_cohinges.size();
				m_cohinges.push_back(CoHinge());
				CoHinge& cohinge = m_cohinges.back();
				cohinge.edge[0] = i1t;
				cohinge.edge[1] = i2t;

				cohinge.tris.push_back(i);
				cohinge.flaps.push_back(m_f[3 * i + (j + 2) % 3]);
			}
			else
			{
				//hinge for this edge already exists, add missing information for this triangle
				CoHinge& hinge = m_cohinges[ite->second];
				hinge.tris.push_back(i);
				hinge.flaps.push_back(m_f[3 * i + (j + 2) % 3]);
			}
		}
	}
	//ordering for edges

	//printf("hinge info:\n");
	//then build hinge from cohinge
	for (int i = 0; i < m_cohinges.size(); i++)
	{
		assert(m_cohinges[i].tris.size() <= 3);
		int num_hinges = 0;
		if (m_cohinges[i].tris.size() == 2)
			num_hinges = 1;
		else
			num_hinges = m_cohinges[i].tris.size();
		//printf("edge %d %d--", m_cohinges[i].edge[0], m_cohinges[i].edge[1]);
		for (int j = 0; j < num_hinges; j++)
		{
			m_hinges.push_back(Hinge());
			Hinge& hinge = m_hinges.back();
			hinge.edge[0] = m_cohinges[i].edge[0];
			hinge.edge[1] = m_cohinges[i].edge[1];

			hinge.tris[0] = m_cohinges[i].tris[j % 3];
			if(m_cohinges[i].tris.size()>1)
				hinge.tris[1] = m_cohinges[i].tris[(j + 1) % 3];

			hinge.flaps[0] = m_cohinges[i].flaps[j % 3];
			if (m_cohinges[i].tris.size()>1)
				hinge.flaps[1] = m_cohinges[i].flaps[(j + 1) % 3];

			//printf("flaps %d %d--", hinge.flaps[0], hinge.flaps[1]);
		}
		//printf("\n");
		
	}
}

void TriMesh::buildTriangleSurroundingTriangleStructure()
{

	std::map<Eigen::Vector3i, std::pair<int, Eigen::Vector3i>, compareVector3<int>> triangleSurroundingVertices; //triangle vertices indices -> tri_index, surroundingVertices

	if (m_hinges.size() == 0)
		buildHingeStructure();

	int ntris = numTriangles();
	for (int i = 0; i < ntris; i++)
	{
		Eigen::Vector3i tri_i( m_f[3 * i + 0], m_f[3 * i + 1], m_f[3 * i + 2]);
		triangleSurroundingVertices[tri_i] = std::make_pair(i, Eigen::Vector3i(-1, -1, -1));

	}

	auto findTriangle = [triangleSurroundingVertices](int v0, int v1, int v2)
	{
		Eigen::Vector3i tri_vertices_try[6];
		tri_vertices_try[0] = Eigen::Vector3i(v0, v1, v2);
		tri_vertices_try[1] = Eigen::Vector3i(v0, v2, v1);
		tri_vertices_try[2] = Eigen::Vector3i(v1, v0, v2);
		tri_vertices_try[3] = Eigen::Vector3i(v1, v2, v0);
		tri_vertices_try[4] = Eigen::Vector3i(v2, v0, v1);
		tri_vertices_try[5] = Eigen::Vector3i(v2, v1, v0);

		for (int i = 0; i < 6; i++)
		{
			if (triangleSurroundingVertices.find(tri_vertices_try[i]) != triangleSurroundingVertices.end())
			{
				return tri_vertices_try[i];
			}
		}
		throw logic_error("We should find a triangle but did not");
	};

	//we build this structure based on hinges
	for (int i = 0; i < m_hinges.size(); i++)
	{
		auto& hinge = m_hinges[i];
		if ((hinge.tris[0] == -1) || (hinge.tris[1] == -1))
			continue; //skip boundary edges

		std::vector<int> inds(4);
		int e0 = hinge.edge[0];
		int e1 = hinge.edge[1];
		int surrounding_v0 = hinge.flaps[0];
		int surrounding_v1 = hinge.flaps[1];
		
		auto face0_v_indices = findTriangle(e0, e1, surrounding_v0);
		auto face1_v_indices = findTriangle(e0, e1, surrounding_v1);

		auto& face0_surroundingVertices = triangleSurroundingVertices[face0_v_indices];
		auto& face1_surroundingVertices = triangleSurroundingVertices[face1_v_indices];

		//for triangle 0, we add surrounding_v1
		//we need to keep the order
		//		 x4
		//     /   \
		//    x3---x2
		//   /  \ /  \
		//  x5---x1---x6
		//x4, x5, x5 are 0,1,2 in surrounding vertices index
		//x1,x2,x3 are the order in the current triangle
		auto setSurroundingVertex = [](int edge_v0,int edge_v1, Eigen::Vector3i face_vindices, int surrounding_v, std::pair<int, Eigen::Vector3i>& face_surroundingVertices)
		{
			if ((edge_v0 == face_vindices[0] && edge_v1 == face_vindices[1])
				|| (edge_v0 == face_vindices[1] && edge_v1 == face_vindices[0]))
			{//this edge is the edge x1, x2 of the triangle
				face_surroundingVertices.second[2] = surrounding_v;
			}
			else if ((edge_v0 == face_vindices[1] && edge_v1 == face_vindices[2])
				|| (edge_v0 == face_vindices[2] && edge_v1 == face_vindices[1]))
			{//this edge is the edge x2, x3 of the triangle
				face_surroundingVertices.second[0] = surrounding_v;
			}
			else if ((edge_v0 == face_vindices[0] && edge_v1 == face_vindices[2])
				|| (edge_v0 == face_vindices[2] && edge_v1 == face_vindices[0]))
			{//this edge is the edge x3, x1 of the triangle
				face_surroundingVertices.second[1] = surrounding_v;
			}
		};
		//for face0, we put surrounding_v1 into surroundingVertices of face0
		setSurroundingVertex(e0, e1, face0_v_indices, surrounding_v1, face0_surroundingVertices);
		//for face1, we put surrounding_v0 into surroundingVertices of face1
		setSurroundingVertex(e0, e1, face1_v_indices, surrounding_v0, face1_surroundingVertices);

	}


	//after building the triangleSurroundingVertices, some triangles may only have two or one surrounding vertices
	//we rearrange the order such that putting -1 to the end of triangleSurroundingVertices
	m_face_ordered_FaceV_surroundingV.clear();
	m_face_ordered_FaceV_surroundingV.reserve(triangleSurroundingVertices.size());

	//for (auto iter = triangleSurroundingVertices.begin(); iter != triangleSurroundingVertices.end(); iter++)
	for(auto &iter : triangleSurroundingVertices)
	{
		int numSurroundingVertices = 0;

		//test how many surrounding vertices we have
		for (int v_i = 0; v_i < 3; v_i++)
		{
			if (iter.second.second[v_i] != -1)
				numSurroundingVertices++;
		}

		//we need to keep the order
		//		 x4
		//     /   \
		//    x3---x2
		//   /  \ /  \
		//  x5---x1---x6
		//x4, x5, x5 are 0,1,2 in surrounding vertices index
		//x1, x2, x3 are the order in the current triangle

		if (numSurroundingVertices == 3)
		{
			m_face_ordered_FaceV_surroundingV.push_back(std::make_pair(iter.first, iter.second.second));
		}
		else if (numSurroundingVertices == 2)
		{
			//re-order
			if (iter.second.second[0] == -1)
			{
				auto f_vIndices = iter.first;
				auto f_surroundingVIndices = iter.second.second;
				
				m_face_ordered_FaceV_surroundingV.push_back(std::make_pair(
					Eigen::Vector3i(f_vIndices[1], f_vIndices[2], f_vIndices[0]),
					Eigen::Vector3i(f_surroundingVIndices[1], f_surroundingVIndices[2], -1)));
			}
			else if (iter.second.second[1] == -1)
			{
				auto f_vIndices = iter.first;
				auto f_surroundingVIndices = iter.second.second;

				m_face_ordered_FaceV_surroundingV.push_back(std::make_pair(
					Eigen::Vector3i(f_vIndices[2], f_vIndices[0], f_vIndices[1]),
					Eigen::Vector3i(f_surroundingVIndices[2], f_surroundingVIndices[0], -1)));
			}
			else
			{
				auto f_vIndices = iter.first;
				auto f_surroundingVIndices = iter.second.second;

				m_face_ordered_FaceV_surroundingV.push_back(std::make_pair(
					f_vIndices,
					f_surroundingVIndices));
			}
		}
		else if (numSurroundingVertices == 1)
		{
			if (iter.second.second[0] != -1)
			{
				auto f_vIndices = iter.first;
				auto f_surroundingVIndices = iter.second.second;

				m_face_ordered_FaceV_surroundingV.push_back(std::make_pair(
					f_vIndices,
					f_surroundingVIndices));
			}
			else if (iter.second.second[1] != -1)
			{
				auto f_vIndices = iter.first;
				auto f_surroundingVIndices = iter.second.second;

				m_face_ordered_FaceV_surroundingV.push_back(std::make_pair(
					Eigen::Vector3i(f_vIndices[1], f_vIndices[2], f_vIndices[0]),
					Eigen::Vector3i(f_surroundingVIndices[1], -1, -1)));
			}
			else
			{
				auto f_vIndices = iter.first;
				auto f_surroundingVIndices = iter.second.second;

				m_face_ordered_FaceV_surroundingV.push_back(std::make_pair(
					Eigen::Vector3i(f_vIndices[2], f_vIndices[0], f_vIndices[1]),
					Eigen::Vector3i(f_surroundingVIndices[2], -1, -1)));
			}
		}
	}
}

void TriMesh::mirror(int mirror_dim)
{
	for (int i = 0; i < numVertices(); i++)
		m_x[3 * i + mirror_dim] *= -1.0;
}

void TriMesh::flip(int k, int l)
{
	assert(k >= 0);
	assert(l >= 0);
	assert(k < 3);
	assert(l < 3);
	for (int i = 0; i < numVertices(); i++)
		std::swap(m_x[3 * i + k], m_x[3 * i + l]);
}


void const TriMesh::computeIsolatedMesh(Eigen::MatrixXd& isolatedVertices, Eigen::MatrixXi& isolatedFaces, Eigen::MatrixXd& isolatedVerticesColors)
{
	int numFaces = faces().size() / 3;

	isolatedVertices = Eigen::MatrixXd(3, numFaces * 3);
	isolatedFaces = Eigen::MatrixXi(3, numFaces);

	isolatedVerticesColors = Eigen::MatrixXd(3, numFaces * 3);
	for (int f_i = 0; f_i < numFaces; f_i++)
	{
		for (int v_i = 0; v_i < 3; v_i++)
		{
			int vertexIndex = faces()[3 * f_i + v_i];
			isolatedVertices.col(f_i * 3 + v_i) = vertices().segment<3>(vertexIndex * 3);
			isolatedFaces(v_i, f_i) = f_i * 3 + v_i;
		}

		for (int v_i = 0; v_i < 3; v_i++)
		{
			//meshColors.col(f_i * 3 + v_i) = facesStrainColors.col(f_i);
			isolatedVerticesColors.col(f_i * 3 + v_i) = faceColors().segment<3>(f_i * 3);
		}

	}
}


void TriMesh::combineMeshes(const std::vector<Eigen::VectorXd>& vertices, const std::vector<std::vector<int>>& faces, Eigen::MatrixXd& output_vertices, Eigen::MatrixXi& output_faces)
{
	output_vertices = Eigen::MatrixXd(3, 0);
	output_faces = Eigen::MatrixXi(3, 0);
	for (int i = 0; i < vertices.size(); i++)
	{
		int oldNumVerices = output_vertices.cols();
		int numVerices_i = vertices[i].size() / 3;
		output_vertices.conservativeResize(3, oldNumVerices + numVerices_i);
		output_vertices.rightCols(numVerices_i) = Eigen::Map<const Eigen::MatrixXd>(vertices[i].data(), 3, numVerices_i);

		int oldNumFaces = output_faces.cols();
		int numFaces_i = faces[i].size() / 3;
		output_faces.conservativeResize(3, oldNumFaces + numFaces_i);
		output_faces.rightCols(numFaces_i) = Eigen::Map<const Eigen::MatrixXi>(faces[i].data(), 3, numFaces_i);
		output_faces.rightCols(numFaces_i).colwise() += Eigen::Vector3i(oldNumVerices, oldNumVerices, oldNumVerices);
	}
}


void TriMesh::combineMeshes(const std::vector<Eigen::MatrixXd>& vertices, const std::vector<Eigen::MatrixXi>& faces, Eigen::MatrixXd& output_vertices, Eigen::MatrixXi& output_faces)
{
	output_vertices = Eigen::MatrixXd(3, 0);
	output_faces = Eigen::MatrixXi(3, 0);
	for (int i = 0; i < vertices.size(); i++)
	{
		int oldNumVerices = output_vertices.cols();
		int numVerices_i = vertices[i].cols();
		output_vertices.conservativeResize(3, oldNumVerices + numVerices_i);
		output_vertices.rightCols(numVerices_i) = vertices[i];

		int oldNumFaces = output_faces.cols();
		int numFaces_i = faces[i].cols();
		if(numFaces_i > 0)
		{
			output_faces.conservativeResize(3, oldNumFaces + numFaces_i);
			output_faces.rightCols(numFaces_i) = faces[i];
			output_faces.rightCols(numFaces_i).colwise() += Eigen::Vector3i(oldNumVerices, oldNumVerices, oldNumVerices);
		}
	}
}
