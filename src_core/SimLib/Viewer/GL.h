#pragma once

#include "../Geom/Normals.h"
#include <igl/opengl/gl.h>

#include <Eigen/Core>
#include <memory>
#include <vector>


/** this function is for simple wireframe overdrawing, it changes the depth values or tests! and rasterization rules*/
void startWireframe();
void stopWireframe();

class ArrayBufferGL
{
public:
	ArrayBufferGL();
	ArrayBufferGL(const ArrayBufferGL&) = delete;
	ArrayBufferGL& operator=(const ArrayBufferGL&) = delete;

	~ArrayBufferGL();

	void setData(
		GLsizeiptr sizeInBytes,
		const GLvoid* data,
		GLenum usage = GL_STATIC_DRAW);

	void bind();

	void unbind();

	GLuint getId();

protected:
	GLuint m_buffer;
	int allocatedSize;
};

class ElementArrayBufferGL
{
public:
	ElementArrayBufferGL();
	ElementArrayBufferGL(const ElementArrayBufferGL&) = delete;
	ElementArrayBufferGL& operator=(const ElementArrayBufferGL&) = delete;

	~ElementArrayBufferGL();

	void bind();

	void unbind();

	void setData(
		GLsizeiptr sizeInBytes,
		const GLvoid * data,
		GLenum usage = GL_STATIC_DRAW);

	GLuint getId();
protected:
	GLuint m_buffer;
};

class VAOGL
{
public:
	VAOGL();
	VAOGL(const VAOGL&) = delete;
	VAOGL& operator=(const VAOGL&) = delete;

	~VAOGL();

	void bind();

	void unbind();

	GLuint getId();
private:
	GLuint m_vao;
};

class MeshCPU
{
public:

	void setMesh(Eigen::MatrixXd vertices, Eigen::MatrixXi triangles, bool flipNorm = false, WeightType::Type normWeightType = WeightType::ANGLE_AREA);
	void setMesh(Eigen::MatrixXd vertices, Eigen::MatrixXi triangles, Eigen::MatrixXd colors, bool flipNorm = false, WeightType::Type normWeightType = WeightType::ANGLE_AREA);
	void setMesh(Eigen::MatrixXd svertices, Eigen::MatrixXd normals, Eigen::MatrixXi triangles, bool flipNorm = false);

	void updateVertices(Eigen::MatrixXd vertices);
	void updateColors(Eigen::MatrixXd colors);
	void updateMesh(Eigen::MatrixXd vertices, Eigen::MatrixXi triangles);
	void updateMesh(Eigen::MatrixXd vertices, Eigen::MatrixXi triangles, Eigen::MatrixXd colors);

	const Eigen::MatrixXd& getVertices() const;

	const Eigen::MatrixXd& getVertexNormals() const;

	const Eigen::MatrixXi & getTriangles() const;

	const Eigen::MatrixXd& getColors() const;

	int getTriangleCount() const;

private:
	Eigen::MatrixXd m_vertices;
	Eigen::MatrixXd m_normals;
	Eigen::MatrixXi m_triangles;
	Eigen::MatrixXd m_colors;
};

class MeshGPU
{
public:
	MeshGPU(const MeshCPU& cpuMesh);

	void updateVertices(const Eigen::MatrixXd& vertices);
	void updateVertexNormals(const Eigen::MatrixXd& vertexNormals);
	void updateTriangles(const Eigen::MatrixXi& triangles);
	void updateColors(const Eigen::MatrixXd& colors);
	void updateMesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& vertexNormals, const Eigen::MatrixXi& triangles);
	void updateMesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& vertexNormals, const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& colors);
	VAOGL& getVAO();

	ArrayBufferGL& getVertexBuffer();
	ArrayBufferGL& getNormalBuffer();
	ArrayBufferGL& getColorBuffer();
	ElementArrayBufferGL& getIndexBuffer();

	void draw();
	void drawWireframe();

	int getTriangleCount() const
	{
		return m_triangleCount;
	}
public:
	ArrayBufferGL m_vertexBuffer;
	ArrayBufferGL m_normalBuffer;
	ArrayBufferGL m_colorBuffer;
	ElementArrayBufferGL m_indexBuffer;
	VAOGL m_vao;
	int m_triangleCount;
};

class MeshManaged
{
public:
	MeshManaged();

	bool hasTriangles() const;

	void setMesh(Eigen::MatrixXd vertices, Eigen::MatrixXi triangles,bool flipNorm = false, WeightType::Type normWeightType = WeightType::ANGLE_AREA);
	void setMesh(Eigen::MatrixXd vertices, Eigen::MatrixXi triangles, Eigen::MatrixXd colors, bool flipNorm = false, WeightType::Type normWeightType = WeightType::ANGLE_AREA);
	void updateVertices(Eigen::MatrixXd vertices);
	void updateColors(Eigen::MatrixXd colors);
	void updateMesh(Eigen::MatrixXd vertices, Eigen::MatrixXi triangles);
	void updateMesh(Eigen::MatrixXd vertices, Eigen::MatrixXi triangles, Eigen::MatrixXd colors);

	void initGL();
	void shutGL();

	MeshGPU& getGPUMesh();
	const MeshCPU& getCPUMesh();

	void draw();
	void drawWireframe();

private:
	MeshCPU m_cpuMesh;
	std::unique_ptr<MeshGPU> m_gpuMesh;
	bool m_glInitialized;
};

class MeshObj;

std::unique_ptr<MeshManaged> loadMeshManaged(const std::string& filepath);

class LinesCPU
{
public:

	void setLines(const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &lines);
	void updateVertices(Eigen::MatrixXd vertices);
	const Eigen::MatrixXd& getVertices() const;

	const Eigen::MatrixXi & getIndices() const;

	int getLineCount() const;
private:
	Eigen::MatrixXd m_vertices;
	Eigen::MatrixXd m_normals;
	Eigen::MatrixXi m_lines;
};
class LinesGPU
{
public:
	LinesGPU(const LinesCPU& cpuLInes);
	void updateVertices(const Eigen::MatrixXd& vertices);

	VAOGL& getVAO();

	ArrayBufferGL& getVertexBuffer();
	ElementArrayBufferGL& getIndexBuffer();

	void draw();
public:
	ArrayBufferGL m_vertexBuffer;
	ElementArrayBufferGL m_indexBuffer;
	VAOGL m_vao;
	int m_linesCount;
};

class LinesManaged
{
public:
	LinesManaged();
	
	bool hasLines() const;
	
	void setLines(const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &lines);
	void updateVertices(const Eigen::MatrixXd& vertices);
	void setGrid(int n);

	void initGL();
	void shutGL();

	LinesGPU& getGPULines();

	void draw();

private:
	LinesCPU m_cpuLines;
	std::unique_ptr<LinesGPU> m_gpuLines;
	bool m_glInitialized;
};


/** collects vertex attributes and some faces */
class GeneralMeshManaged
{
public:
	GeneralMeshManaged();

	void setTriangles(Eigen::MatrixXi triangles);
	void addVertexAttribute(Eigen::MatrixXf vertices, const std::string &name); // name is for debugging purposes

	void buildVAO();

	void draw();

private:
	std::vector< std::shared_ptr<Eigen::MatrixXf> > m_vertexAttributes;
	std::vector< std::string > m_vertexAttributeNames;
	Eigen::MatrixXi m_triangles;
	std::vector< std::shared_ptr<ArrayBufferGL> > m_vertexAttributesGPU;
	ElementArrayBufferGL m_indexBuffer;
	VAOGL m_vao;
};