#pragma once

#include "../Core/StringSubsequence.h"
#include <vector>
#include <Eigen/Core>

//deprecated mesh obj code, use tinyobjloader instead

class MeshObjVertex
{
public:
	MeshObjVertex(double x, double y, double z, double w = 1.0)
		:m_x(x), m_y(y), m_z(z), m_w(w)
	{

	}

	double x() const { return m_x; }
	double y() const { return m_y; }
	double z() const { return m_z; }
	double w() const { return m_w; }

private:
	double m_x, m_y, m_z, m_w;
};
class MeshObjNormal
{
public:
	MeshObjNormal(double x, double y, double z)
		:m_x(x), m_y(y), m_z(z)
	{

	}

	double x() const { return m_x; }
	double y() const { return m_y; }
	double z() const { return m_z; }

private:
	double m_x, m_y, m_z;
};
class MeshObjLine
{
public:
	MeshObjLine(std::vector<int> &&vertices)
	{
		m_vertices.swap(vertices);
	}
	const std::vector<int>& getVertices() const
	{
		return m_vertices;
	}

private:
	std::vector<int> m_vertices;
};
class MeshObjFace
{
public:
	MeshObjFace()
	{

	}
	MeshObjFace(std::vector<int> &&vertices, std::vector<int> &&textureCoordinates, std::vector<int> &&normalIdcs)
	{
		m_vertices.swap(vertices);
		m_textureIndices.swap(textureCoordinates);
		m_normalIndices.swap(normalIdcs);
	}
	const std::vector<int>& getVertices() const
	{
		return m_vertices;
	}

private:
	std::vector<int> m_vertices;
	std::vector<int> m_textureIndices;
	std::vector<int> m_normalIndices;
};

class MeshObj
{

	bool readVertex(const StringSubsequence &subsequence);
	bool readNormal(const StringSubsequence &subsequence);
	bool readLineElement(const StringSubsequence &subsequence);
	bool readFaceElement(const StringSubsequence &subsequence);

public:
	MeshObj();
	~MeshObj();
	void clear();

	void addVertex(MeshObjVertex &&vertex);
	void addLine(MeshObjLine &&line);
	void addFace(MeshObjFace&& face);

	bool load(const std::string &fileContents, std::string &errorMessage);
	bool loadFromFile(const std::string &filepath, std::string &errorMessage);
	bool writeToFile(const std::string &filepath, std::string &errorMessage);
	void centeringVertices();

	Eigen::MatrixXd getVerticesAsEigenCols() const;
	Eigen::MatrixXi getFacesAsEigenCols() const;

	const std::vector<MeshObjVertex>& getVertices() const
	{
		return m_vertices;
	}

	const std::vector<MeshObjNormal>& getNormals() const
	{
		return m_normals;
	}

	const std::vector<MeshObjLine>& getLines() const
	{
		return m_lines;
	}

	const std::vector<MeshObjFace>& getFaces() const
	{
		return m_faces;
	}

private:
	std::vector<MeshObjVertex> m_vertices;
	std::vector<MeshObjNormal> m_normals;
	std::vector<MeshObjLine> m_lines;
	std::vector<MeshObjFace> m_faces;
};
