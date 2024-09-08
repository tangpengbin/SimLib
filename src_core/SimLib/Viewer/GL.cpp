#include "GL.h"

#include <cassert>


#include "../Geom/MeshObj.h"

void startWireframe() {
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glEnable(GL_POLYGON_OFFSET_LINE);
	glPolygonOffset(-0.5, -0.5);
}
void stopWireframe() {
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glDisable(GL_POLYGON_OFFSET_LINE);
}

ArrayBufferGL::ArrayBufferGL()
{
	glGenBuffers(1, &m_buffer);
	assert(glGetError() == GL_NO_ERROR);
	allocatedSize = -1;
}
ArrayBufferGL::~ArrayBufferGL()
{
	glDeleteBuffers(1, &m_buffer);
}
void ArrayBufferGL::bind()
{
	glBindBuffer(GL_ARRAY_BUFFER, m_buffer);
}

void ArrayBufferGL::unbind()
{
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void ArrayBufferGL::setData(
	GLsizeiptr sizeInBytes,
	const GLvoid * data,
	GLenum usage)
{
	assert(glGetError() == GL_NO_ERROR);

	bind();
	assert(glGetError() == GL_NO_ERROR);

	if (allocatedSize == -1 || allocatedSize != sizeInBytes)
	{
		glBufferData(GL_ARRAY_BUFFER, sizeInBytes, data, usage);
		allocatedSize = sizeInBytes;
	}
	else
	{
		GLintptr offset = 0;
		glBufferSubData(GL_ARRAY_BUFFER, offset, sizeInBytes, data);
	}
	GLuint error = glGetError();
	assert(error == GL_NO_ERROR);
	unbind();
	assert(glGetError() == GL_NO_ERROR);
}

GLuint ArrayBufferGL::getId()
{
	return m_buffer;
}

ElementArrayBufferGL::ElementArrayBufferGL()
{
	glGenBuffers(1, &m_buffer);
	assert(glGetError() == GL_NO_ERROR);
}

ElementArrayBufferGL::~ElementArrayBufferGL()
{
	glDeleteBuffers(1, &m_buffer);
}

void ElementArrayBufferGL::bind()
{
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_buffer);
}

void ElementArrayBufferGL::unbind()
{
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void ElementArrayBufferGL::setData(
	GLsizeiptr sizeInBytes,
	const GLvoid * data,
	GLenum usage)
{
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_buffer);
	assert(glGetError() == GL_NO_ERROR);

	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeInBytes, data, usage);
	assert(glGetError() == GL_NO_ERROR);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	assert(glGetError() == GL_NO_ERROR);
}

GLuint ElementArrayBufferGL::getId()
{
	return m_buffer;
}

VAOGL::VAOGL()
{
	glGenVertexArrays(1, &m_vao);
}

VAOGL::~VAOGL()
{
	glDeleteVertexArrays(1, &m_vao);
}

void VAOGL::bind()
{
	glBindVertexArray(m_vao);
}

void VAOGL::unbind()
{
	glBindVertexArray(0);
}

GLuint VAOGL::getId()
{
	return m_vao;
}

void MeshCPU::setMesh(Eigen::MatrixXd vertices, Eigen::MatrixXi triangles, bool flipNorm, WeightType::Type normWeightType)
{
	m_vertices = std::move(vertices);
	m_triangles = std::move(triangles);
	perVertexNormals(m_vertices, m_triangles, m_normals, normWeightType);
	if (flipNorm)
		m_normals *= -1.0;
}


void MeshCPU::setMesh(Eigen::MatrixXd vertices, Eigen::MatrixXi triangles, Eigen::MatrixXd colors, bool flipNorm, WeightType::Type normWeightType)
{
	m_vertices = std::move(vertices);
	m_triangles = std::move(triangles);
	perVertexNormals(m_vertices, m_triangles, m_normals, normWeightType);
	m_colors = std::move(colors);
	if (flipNorm)
		m_normals *= -1.0;
}

void MeshCPU::setMesh(Eigen::MatrixXd vertices, Eigen::MatrixXd normals, Eigen::MatrixXi triangles, bool flipNorm)
{
	m_vertices = std::move(vertices);
	m_normals = std::move(normals);
	m_triangles = std::move(triangles);
	if (flipNorm)
		m_normals *= -1.0;
}
void MeshCPU::updateVertices(Eigen::MatrixXd vertices)
{
	m_vertices = std::move(vertices);
	perVertexNormals(m_vertices, m_triangles, m_normals);
}

void MeshCPU::updateColors(Eigen::MatrixXd colors)
{
	m_colors = std::move(colors);
}
void MeshCPU::updateMesh(Eigen::MatrixXd vertices, Eigen::MatrixXi triangles)
{
	setMesh(vertices, triangles);
}

void MeshCPU::updateMesh(Eigen::MatrixXd vertices, Eigen::MatrixXi triangles, Eigen::MatrixXd colors)
{
	setMesh(vertices, triangles, colors);
}

const Eigen::MatrixXd& MeshCPU::getVertices() const
{
	return m_vertices;
}

const Eigen::MatrixXd& MeshCPU::getVertexNormals() const
{
	return m_normals;
}

const Eigen::MatrixXi & MeshCPU::getTriangles() const
{
	return m_triangles;
}
const Eigen::MatrixXd& MeshCPU::getColors() const
{
	return m_colors;
}

int MeshCPU::getTriangleCount() const
{
	return m_triangles.cols();
}
MeshGPU::MeshGPU(const MeshCPU& cpuMesh)
{
	m_triangleCount = cpuMesh.getTriangleCount();

	Eigen::MatrixXf vertices = cpuMesh.getVertices().cast<float>();
	m_vertexBuffer.setData(vertices.rows()*vertices.cols() * sizeof(Eigen::MatrixXf::Scalar), vertices.data());

	assert(glGetError() == GL_NO_ERROR);

	Eigen::MatrixXf normals = cpuMesh.getVertexNormals().cast<float>();
	m_normalBuffer.setData(normals.rows()*normals.cols() * sizeof(Eigen::MatrixXf::Scalar), normals.data());

	assert(glGetError() == GL_NO_ERROR);

	Eigen::MatrixXf colors = cpuMesh.getColors().cast<float>();
	m_colorBuffer.setData(colors.rows() * colors.cols() * sizeof(Eigen::MatrixXf::Scalar), colors.data());

	assert(glGetError() == GL_NO_ERROR);
	
	const Eigen::MatrixXi &indices = cpuMesh.getTriangles();
	m_indexBuffer.setData(indices.rows()*indices.cols() * sizeof(Eigen::MatrixXi::Scalar), indices.data());

	assert(glGetError() == GL_NO_ERROR);

	VAOGL &vao = m_vao;
	vao.bind();

	assert(glGetError() == GL_NO_ERROR);

	//vertex
	this->getVertexBuffer().bind();

	assert(glGetError() == GL_NO_ERROR);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
	glEnableVertexAttribArray(0);

	assert(glGetError() == GL_NO_ERROR);

	//normal
	this->getNormalBuffer().bind();

	assert(glGetError() == GL_NO_ERROR);

	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
	glEnableVertexAttribArray(1);

	assert(glGetError() == GL_NO_ERROR);

	//color
	this->getColorBuffer().bind();

	assert(glGetError() == GL_NO_ERROR);

	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
	glEnableVertexAttribArray(2);

	assert(glGetError() == GL_NO_ERROR);
	

	this->getIndexBuffer().bind(); // need to bind element array buffer

	assert(glGetError() == GL_NO_ERROR);

	vao.unbind();

	assert(glGetError() == GL_NO_ERROR);
}

void MeshGPU::updateVertices(const Eigen::MatrixXd& verticesDouble)
{
	Eigen::MatrixXf vertices = verticesDouble.cast<float>();
	assert(glGetError() == GL_NO_ERROR);
	m_vertexBuffer.setData(vertices.rows()*vertices.cols() * sizeof(Eigen::MatrixXf::Scalar), vertices.data());
	assert(glGetError() == GL_NO_ERROR);
}
void MeshGPU::updateVertexNormals(const Eigen::MatrixXd& vertexNormalsDouble)
{
	Eigen::MatrixXf vertexNormals = vertexNormalsDouble.cast<float>();
	assert(glGetError() == GL_NO_ERROR);
	m_normalBuffer.setData(vertexNormals.rows()*vertexNormals.cols() * sizeof(Eigen::MatrixXf::Scalar), vertexNormals.data());
	assert(glGetError() == GL_NO_ERROR);
}
void MeshGPU::updateTriangles(const Eigen::MatrixXi& triangles)
{
	assert(glGetError() == GL_NO_ERROR);
	m_indexBuffer.setData(triangles.rows() * triangles.cols() * sizeof(Eigen::MatrixXf::Scalar), triangles.data());
	assert(glGetError() == GL_NO_ERROR);
}

void MeshGPU::updateColors(const Eigen::MatrixXd& colorssDouble)
{
	Eigen::MatrixXf colors = colorssDouble.cast<float>();
	assert(glGetError() == GL_NO_ERROR);
	m_colorBuffer.setData(colors.rows() * colors.cols() * sizeof(Eigen::MatrixXf::Scalar), colors.data());
	assert(glGetError() == GL_NO_ERROR);

}
void MeshGPU::updateMesh(const Eigen::MatrixXd& verticesDouble, const Eigen::MatrixXd& vertexNormals, const Eigen::MatrixXi& triangles)
{
	updateVertices(verticesDouble);
	updateVertexNormals(vertexNormals);
	updateTriangles(triangles);
}
void MeshGPU::updateMesh(const Eigen::MatrixXd& verticesDouble, const Eigen::MatrixXd& vertexNormals, const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& colors)
{
	updateVertices(verticesDouble);
	updateVertexNormals(vertexNormals);
	updateTriangles(triangles);
	updateColors(colors);
}

VAOGL& MeshGPU::getVAO()
{
	return m_vao;
}
ArrayBufferGL& MeshGPU::getVertexBuffer()
{
	return m_vertexBuffer;
}
ArrayBufferGL& MeshGPU::getNormalBuffer()
{
	return m_normalBuffer;
}

ArrayBufferGL& MeshGPU::getColorBuffer()
{
	return m_colorBuffer;
}
ElementArrayBufferGL& MeshGPU::getIndexBuffer()
{
	return m_indexBuffer;
}
void MeshGPU::draw()
{
	VAOGL& vao = this->getVAO();

	assert(glGetError() == GL_NO_ERROR);
	vao.bind();
	assert(glGetError() == GL_NO_ERROR);

	glDrawElements(GL_TRIANGLES, 3 * m_triangleCount, GL_UNSIGNED_INT, 0);

	assert(glGetError() == GL_NO_ERROR);
	vao.unbind();
	assert(glGetError() == GL_NO_ERROR);
}
void MeshGPU::drawWireframe()
{
	startWireframe();
	VAOGL &vao = this->getVAO();

	assert(glGetError() == GL_NO_ERROR);
	vao.bind();
	assert(glGetError() == GL_NO_ERROR);

	glDrawElements(GL_TRIANGLES, 3 * m_triangleCount, GL_UNSIGNED_INT, 0);

	assert(glGetError() == GL_NO_ERROR);
	vao.unbind();
	assert(glGetError() == GL_NO_ERROR);
	stopWireframe();
}
MeshManaged::MeshManaged()
{
	m_glInitialized = false;
}
bool MeshManaged::hasTriangles() const
{
	return m_cpuMesh.getTriangleCount() != 0;
}
void MeshManaged::setMesh(Eigen::MatrixXd vertices, Eigen::MatrixXi triangles, bool flipNorm, WeightType::Type normWeightType)
{
	m_cpuMesh.setMesh(std::move(vertices), std::move(triangles), flipNorm, normWeightType);
	if (m_glInitialized)
	{
		m_gpuMesh.reset(new MeshGPU(m_cpuMesh));
	}
}

void MeshManaged::setMesh(Eigen::MatrixXd vertices, Eigen::MatrixXi triangles, Eigen::MatrixXd colors, bool flipNorm, WeightType::Type normWeightType)
{
	m_cpuMesh.setMesh(std::move(vertices), std::move(triangles), std::move(colors), flipNorm, normWeightType);
	if (m_glInitialized)
	{
		m_gpuMesh.reset(new MeshGPU(m_cpuMesh));
	}
}

void MeshManaged::updateVertices(Eigen::MatrixXd vertices)
{
	m_cpuMesh.updateVertices(std::move(vertices));
	if (m_glInitialized)
	{
		m_gpuMesh->updateVertices(m_cpuMesh.getVertices());
		m_gpuMesh->updateVertexNormals(m_cpuMesh.getVertexNormals());
	}
}
void MeshManaged::updateColors(Eigen::MatrixXd colors)
{
	m_cpuMesh.updateColors(std::move(colors));
	if (m_glInitialized)
	{
		m_gpuMesh->updateColors(m_cpuMesh.getColors());
	}

}
void MeshManaged::updateMesh(Eigen::MatrixXd vertices, Eigen::MatrixXi triangles)
{
	m_cpuMesh.updateMesh(vertices, triangles);
	if (m_glInitialized)
	{
		m_gpuMesh->updateMesh(m_cpuMesh.getVertices(), m_cpuMesh.getVertexNormals(), m_cpuMesh.getTriangles());
	}
}
void MeshManaged::updateMesh(Eigen::MatrixXd vertices, Eigen::MatrixXi triangles, Eigen::MatrixXd colors)
{
	m_cpuMesh.updateMesh(vertices, triangles, colors);
	if (m_glInitialized)
	{
		m_gpuMesh->updateMesh(m_cpuMesh.getVertices(), m_cpuMesh.getVertexNormals(), m_cpuMesh.getTriangles(), m_cpuMesh.getColors());
	}
}
void MeshManaged::initGL()
{
	if (hasTriangles())
	{
		m_gpuMesh.reset(new MeshGPU(m_cpuMesh));
	}
	m_glInitialized = true;

}
void MeshManaged::shutGL()
{
	m_glInitialized = false;
	m_gpuMesh.reset();
}
MeshGPU& MeshManaged::getGPUMesh()
{
	return *m_gpuMesh;
}
const MeshCPU& MeshManaged::getCPUMesh()
{
	return m_cpuMesh;
}
void MeshManaged::draw()
{
	if (hasTriangles())
	{
		m_gpuMesh->draw();
	}
}
void MeshManaged::drawWireframe()
{
	if (hasTriangles())
	{
		m_gpuMesh->drawWireframe();
	}
}

std::unique_ptr<MeshManaged> loadMeshManaged(const std::string &filepath)
{
	std::unique_ptr<MeshManaged> meshManaged(new MeshManaged);
	MeshObj meshobj;
	std::string errorMessage;
	bool success = meshobj.loadFromFile(filepath, errorMessage);
	if (meshobj.getNormals().size() > 0) std::cout << " warning file normals ignored " << std::endl;
	meshManaged->setMesh(meshobj.getVerticesAsEigenCols(), meshobj.getFacesAsEigenCols());
	return std::move(meshManaged);
}


void LinesCPU::setLines(const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &lines)
{
	m_vertices = vertices;
	m_lines = lines;
}


void LinesCPU::updateVertices(Eigen::MatrixXd vertices)
{
	m_vertices = std::move(vertices);
}

const Eigen::MatrixXd& LinesCPU::getVertices() const
{
	return m_vertices;
}

const Eigen::MatrixXi & LinesCPU::getIndices() const
{
	return m_lines;
}
int LinesCPU::getLineCount() const
{
	return m_lines.cols();
}

LinesGPU::LinesGPU(const LinesCPU& cpuLines)
{
	m_linesCount = cpuLines.getLineCount();

	Eigen::MatrixXf vertices = cpuLines.getVertices().cast<float>();
	m_vertexBuffer.setData(vertices.rows()*vertices.cols() * sizeof(Eigen::MatrixXf::Scalar), vertices.data());

	assert(glGetError() == GL_NO_ERROR);

	const Eigen::MatrixXi &indices = cpuLines.getIndices();
	m_indexBuffer.setData(indices.rows()*indices.cols() * sizeof(Eigen::MatrixXi::Scalar), indices.data());

	assert(glGetError() == GL_NO_ERROR);

	VAOGL &vao = getVAO();
	vao.bind();

	assert(glGetError() == GL_NO_ERROR);

	this->getVertexBuffer().bind();

	assert(glGetError() == GL_NO_ERROR);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
	glEnableVertexAttribArray(0);

	assert(glGetError() == GL_NO_ERROR);

	this->getIndexBuffer().bind(); // need to bind element array buffer

	assert(glGetError() == GL_NO_ERROR);

	vao.unbind();

	assert(glGetError() == GL_NO_ERROR);
}

void LinesGPU::updateVertices(const Eigen::MatrixXd& verticesDouble)
{
	Eigen::MatrixXf vertices = verticesDouble.cast<float>();
	assert(glGetError() == GL_NO_ERROR);
	m_vertexBuffer.setData(vertices.rows() * vertices.cols() * sizeof(Eigen::MatrixXf::Scalar), vertices.data());
	assert(glGetError() == GL_NO_ERROR);
}

VAOGL& LinesGPU::getVAO()
{
	return m_vao;
}
ArrayBufferGL& LinesGPU::getVertexBuffer()
{
	return m_vertexBuffer;
}
ElementArrayBufferGL& LinesGPU::getIndexBuffer()
{
	return m_indexBuffer;
}
void LinesGPU::draw()
{
	VAOGL &vao = this->getVAO();

	assert(glGetError() == GL_NO_ERROR);
	vao.bind();
	assert(glGetError() == GL_NO_ERROR);

	glDrawElements(GL_LINES, 2 * m_linesCount, GL_UNSIGNED_INT, 0);

	assert(glGetError() == GL_NO_ERROR);
	vao.unbind();
	assert(glGetError() == GL_NO_ERROR);
}

LinesManaged::LinesManaged()
{
	m_glInitialized = false;
}
bool LinesManaged::hasLines() const
{
	return m_cpuLines.getLineCount() != 0;
}
void LinesManaged::setLines(const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &lines)
{
	m_cpuLines.setLines(vertices, lines);
	if (m_glInitialized)
	{
		m_gpuLines.reset(new LinesGPU(m_cpuLines));
	}
}

void LinesManaged::updateVertices(const Eigen::MatrixXd& vertices)
{
	m_cpuLines.updateVertices(std::move(vertices));
	if (m_glInitialized)
	{
		m_gpuLines->updateVertices(m_cpuLines.getVertices());
	}
}

void LinesManaged::setGrid(int n)
{
	int nVertices = n * n;
	Eigen::MatrixXd vertices(3, nVertices);
	double x0 = -double(n - 1) / 2.0;
	double x1 = double(n - 1) / 2.0;
	double z0 = -double(n - 1) / 2.0;
	double z1 = double(n - 1) / 2.0;
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			double l1 = i / double(n - 1);
			double l2 = j / double(n - 1);
			Eigen::Vector3d x = Eigen::Vector3d::Zero();
			x[0] = (1.0 - l1) * x0 + l1 * x1;
			x[2] = (1.0 - l2) * z0 + l2 * z1;
			vertices.col(i * n + j) = x;
		}
	}
	int nLines = 2 * (n - 1) * n;
	Eigen::MatrixXi lines(2, nLines);
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < (n - 1); j++)
		{
			int v0 = i * n + j;
			int v1 = i * n + j + 1;
			int lIdx = i * (n - 1) + j;
			lines(0, lIdx) = v0;
			lines(1, lIdx) = v1;

			v0 = j * n + i;
			v1 = (j + 1) * n + i;
			lIdx = i * (n - 1) + j + n * (n - 1);
			lines(0, lIdx) = v0;
			lines(1, lIdx) = v1;
		}
	}
	this->setLines(vertices, lines);
}
void LinesManaged::initGL()
{
	if (hasLines())
	{
		m_gpuLines.reset(new LinesGPU(m_cpuLines));
	}
	m_glInitialized = true;
}

void LinesManaged::shutGL()
{
	m_gpuLines.reset();
	m_glInitialized = false;
}
LinesGPU& LinesManaged::getGPULines()
{
	return *m_gpuLines;
}
void LinesManaged::draw()
{
	if (hasLines())
	{
		m_gpuLines->draw();
	}
}
GeneralMeshManaged::GeneralMeshManaged()
{
}
void GeneralMeshManaged::setTriangles(Eigen::MatrixXi triangles)
{
	m_triangles = std::move(triangles);
}
void GeneralMeshManaged::addVertexAttribute(Eigen::MatrixXf vertices, const std::string &name)
{
	m_vertexAttributes.push_back(std::shared_ptr<Eigen::MatrixXf>(new Eigen::MatrixXf(std::move(vertices))));
	m_vertexAttributeNames.push_back(name);
}
void GeneralMeshManaged::buildVAO()
{
	int m_triangleCount = m_triangles.cols();

	for (int vtxAttributeIdx = 0; vtxAttributeIdx < m_vertexAttributes.size(); vtxAttributeIdx++)
	{
		const Eigen::MatrixXf &vertexData = *m_vertexAttributes[vtxAttributeIdx];
		m_vertexAttributesGPU.emplace_back(new ArrayBufferGL);
		m_vertexAttributesGPU.back()->setData(vertexData.rows()*vertexData.cols() * sizeof(Eigen::MatrixXf::Scalar), vertexData.data());

		assert(glGetError() == GL_NO_ERROR);
	}


	const Eigen::MatrixXi &indices = m_triangles;
	m_indexBuffer.setData(indices.rows()*indices.cols() * sizeof(Eigen::MatrixXi::Scalar), indices.data());

	assert(glGetError() == GL_NO_ERROR);

	VAOGL &vao = m_vao;
	vao.bind();

	assert(glGetError() == GL_NO_ERROR);

	for (int vtxAttributeIdx = 0; vtxAttributeIdx < m_vertexAttributes.size(); vtxAttributeIdx++)
	{
		m_vertexAttributesGPU[vtxAttributeIdx]->bind();
		assert(glGetError() == GL_NO_ERROR);
		const Eigen::MatrixXf &vertexData = *m_vertexAttributes[vtxAttributeIdx];
		glVertexAttribPointer(vtxAttributeIdx, vertexData.rows(), GL_FLOAT, GL_FALSE, vertexData.rows() * sizeof(float), 0);
		glEnableVertexAttribArray(vtxAttributeIdx);

		assert(glGetError() == GL_NO_ERROR);
	}

	m_indexBuffer.bind(); // need to bind element array buffer

	assert(glGetError() == GL_NO_ERROR);

	vao.unbind();

	assert(glGetError() == GL_NO_ERROR);
}
void GeneralMeshManaged::draw()
{
	int m_triangleCount = m_triangles.cols();
	VAOGL &vao = m_vao;

	assert(glGetError() == GL_NO_ERROR);
	vao.bind();
	assert(glGetError() == GL_NO_ERROR);

	glDrawElements(GL_TRIANGLES, 3 * m_triangleCount, GL_UNSIGNED_INT, 0);

	assert(glGetError() == GL_NO_ERROR);
	vao.unbind();
	assert(glGetError() == GL_NO_ERROR);
}