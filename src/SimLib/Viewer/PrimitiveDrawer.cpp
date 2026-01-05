#include "PrimitiveDrawer.h"

#include "../Geom/Mesh.h"
#include "../Core/SOUtils.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "../Geom/SimpleTriMesh.h"
#include "../Geom/Primitive.h"


#include <fstream>

#include <iostream>


PrimitiveDrawer::PrimitiveDrawer()
{

	m_glInitialized = false;

	Eigen::MatrixXd vertices;
	Eigen::MatrixXi triangles;

	initializeSphere(1);

	SimOpt::cylinderSeperatedCaps(20, 2, vertices, triangles); //set divisions to 2, so we can use trick in conical transform
	//cylinderNoCaps(10, 2, vertices, triangles);
	m_cylinder.setMesh(vertices, triangles);

	SimOpt::coneSeperatedCap(10, vertices, triangles); //set height divisions to 3 because of vertex normals
	m_cone.setMesh(vertices, triangles);

	vertices.resize(3, 3);
	vertices << 0.0, 1.0, 0.0,
		0.0, 0.0, 1.0,
		0.0, 0.0, 0.0;
	triangles.resize(3, 1);
	triangles(0, 0) = 0;
	triangles(1, 0) = 1;
	triangles(2, 0) = 2;
	m_triangle.setMesh(vertices, triangles);


	SimOpt::cuboid(2.0, 2.0, 2.0, true, vertices, triangles);
	m_cuboid.setMesh(vertices, triangles, false);
}
void PrimitiveDrawer::initializeSphere(int nSubdivisions)
{
	Eigen::MatrixXd vertices;
	Eigen::MatrixXi triangles;

	SimpleTriMesh::icosphere(nSubdivisions).toMatrices(vertices, triangles);
	std::unique_ptr<MeshManaged> sphereMesh(new MeshManaged());
	sphereMesh->setMesh(vertices, triangles);
	if (m_glInitialized) sphereMesh->initGL();
	m_spheres.emplace(nSubdivisions, std::move(sphereMesh));
}

void PrimitiveDrawer::initGL()
{
	if (m_glInitialized) shutGL();

	assert(glGetError() == GL_NO_ERROR);
	
	m_program = m_shaderCache->getProgram(ShaderPipelineDesc::VertexFragment(shaderFolder + "PrimitiveDrawer.vs", shaderFolder + "PrimitiveDrawer.fs"));

	assert(glGetError() == GL_NO_ERROR);
	m_texturedProgram = m_shaderCache->getProgram(ShaderPipelineDesc::VertexFragment(shaderFolder + "PrimitiveDrawerTextureQuad.vs", shaderFolder + "PrimitiveDrawerTextureQuad.fs"));
	assert(glGetError() == GL_NO_ERROR);

	assert(glGetError() == GL_NO_ERROR);
	m_screenTextureProgram = m_shaderCache->getProgram(ShaderPipelineDesc::VertexFragment(shaderFolder + "ScreenTexture.vs", shaderFolder + "ScreenTexture.fs"));
	assert(glGetError() == GL_NO_ERROR);

	for (auto &sphere : m_spheres) sphere.second->initGL();

	assert(glGetError() == GL_NO_ERROR);

	m_cylinder.initGL();

	assert(glGetError() == GL_NO_ERROR);

	m_cone.initGL();

	assert(glGetError() == GL_NO_ERROR);

	m_triangle.initGL();

	assert(glGetError() == GL_NO_ERROR);

	m_cuboid.initGL();

	assert(glGetError() == GL_NO_ERROR);

	m_glInitialized = true;
}

void PrimitiveDrawer::shutGL()
{
	if (!m_glInitialized) return;
	m_cone.shutGL();
	m_cylinder.shutGL();
	for(auto &sphere : m_spheres) sphere.second->shutGL();
	m_cuboid.shutGL();

	m_glInitialized = false;
}

Eigen::Matrix4d PrimitiveDrawer::computeSphereTransformation(
	double radius,
	const Eigen::Vector3d &center)
{
	Eigen::Matrix4d model = Eigen::Matrix4d::Identity();
	model(0, 0) = radius;
	model(1, 1) = radius;
	model(2, 2) = radius;
	model.col(3).head(3) = center;
	return model;
}

void PrimitiveDrawer::addSphereMesh(
	const Eigen::Matrix4d& model,
	SimpleTriMesh& mesh,
	int nSubdivisions
	)
{
	if (m_spheres.find(nSubdivisions) == m_spheres.end())
	{
		initializeSphere(nSubdivisions);
	}

	mesh.addMesh(m_spheres[nSubdivisions]->getCPUMesh().getVertices(), m_spheres[nSubdivisions]->getCPUMesh().getTriangles(), model);
}
void PrimitiveDrawer::addSphereMesh(
	double radius,
	const Eigen::Vector3d& center,
	SimpleTriMesh& mesh,
	int nSubdivisions
	)
{
	Eigen::Matrix4d model = computeSphereTransformation(radius, center);
	addSphereMesh(model, mesh, nSubdivisions);
}
void PrimitiveDrawer::drawSphere(
	double radius,
	const Eigen::Vector3d &center,
	const Eigen::Vector4d &color,
	int nSubdivisions
)
{
	Eigen::Matrix4d model = computeSphereTransformation(radius, center);
	setupProgram(model, Viewer::Material(color));
	if (m_spheres.find(nSubdivisions) == m_spheres.end())
	{
		initializeSphere(nSubdivisions);
	}
	m_spheres[nSubdivisions]->draw();
	assert(glGetError() == GL_NO_ERROR);
}

Eigen::Matrix4d PrimitiveDrawer::computeCuboidTransformation(std::tuple<double, double, double> cuboidDimensions,
	const Eigen::Vector3d& frame1,
	const Eigen::Vector3d& frame2,
	const Eigen::Vector3d& center)
{
	Eigen::Quaternion<double> rot = Eigen::Quaternion<double>::FromTwoVectors(Eigen::Vector3d(0.0, 1.0, 0.0), frame1);
	Eigen::Quaternion<double> rot_next = Eigen::Quaternion<double>::FromTwoVectors(rot * Eigen::Vector3d(0.0, 0.0, 1.0), frame2);

	Eigen::DiagonalMatrix<double, 3> scale = Eigen::Scaling(std::get<0>(cuboidDimensions) / 2.0, std::get<1>(cuboidDimensions) / 2.0, std::get<2>(cuboidDimensions) / 2.0);

	Eigen::Matrix4d model = (rot_next * rot * scale).matrix();
	model.col(3).head(3) = center;

	return model;
}

void PrimitiveDrawer::drawCuboid(
	std::tuple<double, double, double> cuboidDimensions,
	const Eigen::Vector3d& frame1,
	const Eigen::Vector3d& frame2,
	const Eigen::Vector3d& center,
	const Eigen::Vector4d& color)
{
	Eigen::Matrix4d model = computeCuboidTransformation(cuboidDimensions, frame1, frame2, center);
	setupProgram(model, Viewer::Material(color));
	
	m_cuboid.draw();
	assert(glGetError() == GL_NO_ERROR);
}

Eigen::Matrix4d PrimitiveDrawer::computeEllipsoidTransformation(std::pair<double, double> radius,
	const Eigen::Vector3d& frame1,
	const Eigen::Vector3d& frame2,
	const Eigen::Vector3d& center)
{
	Eigen::Quaternion<double> rot = Eigen::Quaternion<double>::FromTwoVectors(Eigen::Vector3d(0.0, 1.0, 0.0), frame1);
	Eigen::Quaternion<double> rot_next = Eigen::Quaternion<double>::FromTwoVectors(rot * Eigen::Vector3d(0.0, 0.0, 1.0), frame2);

	Eigen::DiagonalMatrix<double, 3> scale = Eigen::Scaling(radius.first, radius.first, radius.second);

	Eigen::Matrix4d model = (rot_next * rot * scale).matrix();
	model.col(3).head(3) = center;

	return model;
}

void PrimitiveDrawer::drawEllipsoid(std::pair<double, double> radius,
	const Eigen::Vector3d& frame1,
	const Eigen::Vector3d& frame2,
	const Eigen::Vector3d& center,
	const Eigen::Vector4d& color,
	int nSubdivisions)
{
	Eigen::Matrix4d model = computeEllipsoidTransformation(radius, frame1, frame2, center);
	setupProgram(model, Viewer::Material(color));
	if (m_spheres.find(nSubdivisions) == m_spheres.end())
	{
		initializeSphere(nSubdivisions);
	}
	m_spheres[nSubdivisions]->draw();
	assert(glGetError() == GL_NO_ERROR);
}


Eigen::Matrix4d PrimitiveDrawer::computeCylinderTransformation(
	double radius,
	const Eigen::Vector3d &pFrom,
	const Eigen::Vector3d &pTo)
{
	Eigen::Vector3d diff = (pTo - pFrom);
	double length = diff.norm();
	Eigen::Quaternion<double> rot = Eigen::Quaternion<double>::FromTwoVectors(Eigen::Vector3d(0.0, 0.0, 1.0), diff);

	Eigen::DiagonalMatrix<double, 3> scale = Eigen::Scaling(radius, radius, length);

	Eigen::Matrix4d model = (rot * scale).matrix();
	model.col(3).head(3) = pFrom;
	return model;
}


Eigen::Matrix4d PrimitiveDrawer::computeCylinderTransformation(std::pair<double, double> radius, 
	const Eigen::Vector3d& frame1, const Eigen::Vector3d& frame2,
	const Eigen::Vector3d& pFrom, const Eigen::Vector3d& pTo)
{
	Eigen::Vector3d diff = (pTo - pFrom);
	double length = diff.norm();

	Eigen::Vector3d t = diff / length;
	Eigen::Matrix3d R = soutil::computeRotationFromTwoCoordinateSystems(Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0),
		t, frame1, frame2);

	Eigen::DiagonalMatrix<double, 3> scale = Eigen::Scaling(radius.second, radius.first, length);

	Eigen::Matrix4d model; model.setIdentity();
	model.topLeftCorner(3, 3) = R * scale;
	//Eigen::Matrix4d model = (rot * scale).matrix();
	model.col(3).head(3) = pFrom;
	return model;
}

void PrimitiveDrawer::addCylinderMesh(
	const Eigen::Matrix4d& model,
	SimpleTriMesh& mesh
	)
{
	mesh.addMesh(m_cylinder.getCPUMesh().getVertices(), m_cylinder.getCPUMesh().getTriangles(), model);
}
void PrimitiveDrawer::addCylinderMesh(
	const Eigen::Matrix4d& model,
	int axis_devisions,
	SimpleTriMesh& mesh)
{
	Eigen::MatrixXd vertices;
	Eigen::MatrixXi triangles;
	SimOpt::cylinderSeperatedCaps(axis_devisions, 2, vertices, triangles);
	mesh.addMesh(vertices, triangles, model);
}
void PrimitiveDrawer::addCylinderMesh(
	double radius,
	const Eigen::Vector3d& pFrom,
	const Eigen::Vector3d& pTo,
	SimpleTriMesh& mesh
)
{
	Eigen::Matrix4d model = computeCylinderTransformation(radius, pFrom, pTo);
	mesh.addMesh(m_cylinder.getCPUMesh().getVertices(), m_cylinder.getCPUMesh().getTriangles(), model);
}
void PrimitiveDrawer::drawCylinder(
	double radius,
	const Eigen::Vector3d &pFrom,
	const Eigen::Vector3d &pTo,
	const Viewer::Material &material
)
{
	Eigen::Matrix4d model = computeCylinderTransformation(radius, pFrom, pTo);
	setupProgram(model, material);
	m_cylinder.draw();
	assert(glGetError() == GL_NO_ERROR);
}

void PrimitiveDrawer::drawCylinder(double radius, const Eigen::Vector3d& pFrom, const Eigen::Vector3d& pTo, 
	const Viewer::Material& material1, const Viewer::Material& material2)
{
	Eigen::Matrix4d model = computeCylinderTransformation(radius, pFrom, pTo);
	setupProgram(pFrom, pTo, model, material1,material2);
	m_cylinder.draw();
	assert(glGetError() == GL_NO_ERROR);
}

void PrimitiveDrawer::drawCylinder(std::pair<double, double> radius, const Eigen::Vector3d& frame1, const Eigen::Vector3d& frame2, const Eigen::Vector3d& pFrom, const Eigen::Vector3d& pTo, const Viewer::Material& material)
{
	Eigen::Matrix4d model = computeCylinderTransformation(radius, frame1, frame2, pFrom, pTo);
	setupProgram(pFrom, pTo, model, material, material);
	m_cylinder.draw();
}

void PrimitiveDrawer::drawCylinder(std::pair<double, double> radius, const Eigen::Vector3d& frame1, const Eigen::Vector3d& frame2, const Eigen::Vector3d& pFrom, const Eigen::Vector3d& pTo, const Viewer::Material& material1, const Viewer::Material& material2)
{
	Eigen::Matrix4d model = computeCylinderTransformation(radius, frame1, frame2, pFrom, pTo);
	setupProgram(pFrom, pTo, model, material1, material2);
	m_cylinder.draw();
}

void PrimitiveDrawer::addConeMesh(
	const Eigen::Matrix4d& model,
	SimpleTriMesh& mesh
	)
{
	mesh.addMesh(m_cone.getCPUMesh().getVertices(), m_cone.getCPUMesh().getTriangles(), model);
}
void PrimitiveDrawer::addConeMesh(
	double radius,
	const Eigen::Vector3d& pFrom,
	const Eigen::Vector3d& pTo,
	SimpleTriMesh& mesh
	)
{
	Eigen::Matrix4d model = computeCylinderTransformation(radius, pFrom, pTo);
	addConeMesh(model, mesh);
}
void PrimitiveDrawer::drawCone(
	double radius,
	const Eigen::Vector3d &pFrom,
	const Eigen::Vector3d &pTo,
	const Eigen::Vector4d &color
)
{
	Eigen::Matrix4d model = computeCylinderTransformation(radius, pFrom, pTo);
	setupProgram(model, color);
	m_cone.draw();
	assert(glGetError() == GL_NO_ERROR);
}
void PrimitiveDrawer::drawCone(double radius, const Eigen::Vector3d& pFrom, const Eigen::Vector3d& pTo, const Eigen::Vector4d& color1, const Eigen::Vector4d& color2)
{
	Eigen::Matrix4d model = computeCylinderTransformation(radius, pFrom, pTo);
	setupProgram(pFrom, pTo, model, color1,color2);
	m_cone.draw();
	assert(glGetError() == GL_NO_ERROR);
}
Eigen::Matrix4d PrimitiveDrawer::computeConicalFrustumTransformation(
	double radiusFrom,
	double radiusTo,
	const Eigen::Vector3d &pFrom,
	const Eigen::Vector3d &pTo)
{
	double s = radiusTo / radiusFrom;

	if (radiusFrom == radiusTo) return computeCylinderTransformation(radiusFrom, pFrom, pTo);

	Eigen::Vector3d diff = (pTo - pFrom);
	double length = diff.norm();

	Eigen::Matrix4d modelCF2;
	modelCF2 <<
		1.0, 0, 0, 0,
		0, 1.0, 0, 0,
		0, 0, 1.0 / s, 0.0,
		0, 0, 1.0/s - 1.0, 1.0;


	Eigen::Quaternion<double> rot = Eigen::Quaternion<double>::FromTwoVectors(Eigen::Vector3d(0.0, 0.0, 1.0), diff);

	Eigen::DiagonalMatrix<double, 3> scale = Eigen::Scaling(radiusFrom, radiusFrom, length);

	Eigen::Matrix4d model = (rot * scale).matrix();
	model.col(3).head(3) = pFrom;
	Eigen::Matrix4d result = model * modelCF2;

	//SimpleTriMesh testMesh;
	//testMesh.setFromMatrices(m_cylinder.getCPUMesh().getVertices(), m_cylinder.getCPUMesh().getTriangles());
	//testMesh.transform(modelCF2);
	//for (int i = 0; i < testMesh.getVertexCount(); i++)
	//{
	//	Eigen::Vector3d v = testMesh.getVertex(i);
	//	Eigen::Vector3d v2 = m_cylinder.getCPUMesh().getVertices().col(i);
	//	Eigen::Vector4d v24;
	//	v24 << v2, 1.0;
	//	//Eigen::Vector4d Mv24 = (modelS  * modelCF2) * v24;
	//	Eigen::Vector4d Mv242 = modelCF2 * v24;
	//	double z = v[2];
	//	if (std::abs(z - 0.0) > 1e-8 && std::abs(z - 1.0) > 1e-8)
	//	{
	//		int k = 3; // error detected
	//	}
	//}
	return result;
}
void PrimitiveDrawer::addConicalFrustumMesh(
	double radiusFrom,
	double radiusTo,
	const Eigen::Vector3d& pFrom,
	const Eigen::Vector3d& pTo,
	SimpleTriMesh& mesh
)
{
	Eigen::Matrix4d model = computeConicalFrustumTransformation(radiusFrom, radiusTo, pFrom, pTo);
	addCylinderMesh(model, mesh);
}
void PrimitiveDrawer::addConicalFrustumMesh(
	double radiusFrom,
	double radiusTo,
	const Eigen::Vector3d& pFrom,
	const Eigen::Vector3d& pTo,
	int axis_devisions,
	SimpleTriMesh& mesh
){
	Eigen::Matrix4d model = computeConicalFrustumTransformation(radiusFrom, radiusTo, pFrom, pTo);
	addCylinderMesh(model, axis_devisions, mesh);
}
void PrimitiveDrawer::drawConicalFrustum(
	double radiusFrom,
	double radiusTo,
	const Eigen::Vector3d &pFrom,
	const Eigen::Vector3d &pTo,
	const Eigen::Vector4d &color
)
{
	Eigen::Matrix4d model = computeConicalFrustumTransformation(radiusFrom, radiusTo, pFrom, pTo);
	setupProgram(model, color);
	m_cylinder.draw();
	assert(glGetError() == GL_NO_ERROR);
}

void PrimitiveDrawer::addArrowMeshes(
	double radius,
	const Eigen::Vector3d& pFrom,
	const Eigen::Vector3d& pTo,
	SimpleTriMesh& mesh
	)
{
	Eigen::Vector3d e = (pTo - pFrom).normalized();

	double tipRadius = 1.7 * radius;
	double tipHeight = 4.0 * radius;
	if (tipHeight >= (pTo - pFrom).norm())
	{
		Eigen::Matrix4d model = computeCylinderTransformation(tipRadius, pFrom, pTo);
		addConeMesh(model, mesh);
	}
	else
	{
		Eigen::Vector3d tipStartPoint = pTo - tipHeight * e;

		Eigen::Matrix4d model = computeCylinderTransformation(radius, pFrom, tipStartPoint);
		addCylinderMesh(model, mesh);

		model = computeCylinderTransformation(tipRadius, tipStartPoint, pTo);
		addConeMesh(model, mesh);
	}

	assert(glGetError() == GL_NO_ERROR);
}
void PrimitiveDrawer::drawArrow(
	double radius,
	const Eigen::Vector3d &pFrom,
	const Eigen::Vector3d &pTo,
	const Viewer::Material &material
)
{
	Eigen::Vector3d e = (pTo - pFrom).normalized();

	double tipRadius = 1.7*radius;
	double tipHeight = 4.0*radius;
	if (tipHeight >= (pTo - pFrom).norm())
	{
		Eigen::Matrix4d model = computeCylinderTransformation(tipRadius, pFrom, pTo);
		setupProgram(model, material);
		m_cone.draw();
	}
	else
	{
		Eigen::Vector3d tipStartPoint = pTo - tipHeight * e;

		Eigen::Matrix4d model = computeCylinderTransformation(radius, pFrom, tipStartPoint);
		setupProgram(model, material);
		m_cylinder.draw();

		model = computeCylinderTransformation(tipRadius, tipStartPoint, pTo);
		setupProgram(model, material);
		m_cone.draw();
	}

	assert(glGetError() == GL_NO_ERROR);
}
void PrimitiveDrawer::drawArrow(double radius, const Eigen::Vector3d& pFrom, const Eigen::Vector3d& pTo, const Viewer::Material& material1, const Viewer::Material& material2)
{
	Eigen::Vector3d e = (pTo - pFrom).normalized();

	double tipRadius = 1.7 * radius;
	double tipHeight = 4.0 * radius;
	if (tipHeight >= (pTo - pFrom).norm())
	{
		Eigen::Matrix4d model = computeCylinderTransformation(tipRadius, pFrom, pTo);
		setupProgram(pFrom, pTo, model, material1, material2);
		m_cone.draw();
	}
	else
	{
		Eigen::Vector3d tipStartPoint = pTo - tipHeight * e;

		Eigen::Matrix4d model = computeCylinderTransformation(radius, pFrom, tipStartPoint);
		setupProgram(pFrom, tipStartPoint, model, material1, material2);
		m_cylinder.draw();

		model = computeCylinderTransformation(tipRadius, tipStartPoint, pTo);
		setupProgram(tipStartPoint, pTo, model, material1, material2);
		m_cone.draw();
	}

	assert(glGetError() == GL_NO_ERROR);
}
void PrimitiveDrawer::drawTriangle(
	const Eigen::Vector3d &x1,
	const Eigen::Vector3d &x2,
	const Eigen::Vector3d &x3,
	const Viewer::Material &material
)
{
	// A * [[0;0;0] [1; 0; 0] [0; 1; 0]] =  [x1 x2 x3]
	// A * [[0;0;0] [1; 0; 0] [0; 1; 0]] = [x1 x2 x3] - [x1 x1 x1]
	// thats obviously not invertible since we are looking for 3x3 transformation matrix even though we are only prescribing inplane deformation
	// multiplying in [0;0;0] gives 0;0;0 anyway so forget about it 
	// A * [[1; 0; 0] [0; 1; 0] [0; 0; 1] = [x2 - x1, x3 - x1, z - x1]
	// => A = [x2 - x1, x3 - x1, z]
	// where z is some arbitrary point defined in normal direction i.e. x1 + n

	Eigen::Vector3d n = (x2 - x1).cross(x3 - x1).normalized();

	Eigen::Matrix3d B;
	B << (x2 - x1), (x3 - x1), n;
	Eigen::Matrix4d model = Eigen::Matrix4d::Identity();
	model.block<3, 3>(0, 0) = B;
	model.block<3, 1>(0, 3) = x1;
	
	setupProgram(model, material);
	m_triangle.draw();
}

void PrimitiveDrawer::drawTexturedQuad(
	const TextureGPU &gpuTexture,
	const Eigen::Matrix4d &modelMatrix
)
{
	//Texture coordinates are baked into positions for now, a bit of a hack but we dont have time to fix it.
	Eigen::MatrixXd vertices(3, 4);
	vertices.col(0) = Eigen::Vector3d(0.0, 0.0, 0.0);
	vertices.col(1) = Eigen::Vector3d(1.0, 0.0, 0.0);
	vertices.col(2) = Eigen::Vector3d(1.0, 1.0, 0.0);
	vertices.col(3) = Eigen::Vector3d(0.0, 1.0, 0.0);
	Eigen::MatrixXi triangles(3, 2);
	triangles << 0, 0,
		1, 2,
		2, 3;
	MeshCPU mesh;
	mesh.setMesh(vertices, triangles);

	MeshManaged mesh_i;
	mesh_i.setMesh(mesh.getVertices(), mesh.getTriangles());
	mesh_i.initGL();

	assert(glGetError() == GL_NO_ERROR);
	glActiveTexture(GL_TEXTURE0);
	assert(glGetError() == GL_NO_ERROR);
	//glEnable(GL_TEXTURE);
	assert(glGetError() == GL_NO_ERROR);
	gpuTexture.bind();

	assert(glGetError() == GL_NO_ERROR);
	setupTexturedProgram(modelMatrix.cast<float>(), 0);
	mesh_i.draw();
	assert(glGetError() == GL_NO_ERROR);
	mesh_i.shutGL();
	assert(glGetError() == GL_NO_ERROR);

	gpuTexture.unbind();
}
void PrimitiveDrawer::drawTexturedQuad(
	GLuint textureId,
	const Eigen::Matrix4d &modelMatrix
)
{
	//Texture coordinates are baked into positions for now, a bit of a hack but we dont have time to fix it.
	Eigen::MatrixXd vertices(3, 4);
	vertices.col(0) = Eigen::Vector3d(0.0, 0.0, 0.0);
	vertices.col(1) = Eigen::Vector3d(1.0, 0.0, 0.0);
	vertices.col(2) = Eigen::Vector3d(1.0, 1.0, 0.0);
	vertices.col(3) = Eigen::Vector3d(0.0, 1.0, 0.0);
	Eigen::MatrixXi triangles(3, 2);
	triangles << 0, 0,
		1, 2,
		2, 3;
	MeshCPU mesh;
	mesh.setMesh(vertices, triangles);

	MeshManaged mesh_i;
	mesh_i.setMesh(mesh.getVertices(), mesh.getTriangles());
	mesh_i.initGL();

	assert(glGetError() == GL_NO_ERROR);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureId);
	assert(glGetError() == GL_NO_ERROR);

	assert(glGetError() == GL_NO_ERROR);
	setupTexturedProgram(modelMatrix.cast<float>(), 0);
	mesh_i.draw();
	assert(glGetError() == GL_NO_ERROR);
	mesh_i.shutGL();
	assert(glGetError() == GL_NO_ERROR);

	glBindTexture(GL_TEXTURE_2D, 0);
}
void PrimitiveDrawer::drawScreenQuad(
	GLuint textureId
)
{
	//Texture coordinates are baked into positions for now, a bit of a hack but we dont have time to fix it.
	Eigen::MatrixXd vertices(3, 4);
	vertices.col(0) = Eigen::Vector3d(-1.0, -1.0, 0.5);
	vertices.col(1) = Eigen::Vector3d(1.0, -1.0, 0.5);
	vertices.col(2) = Eigen::Vector3d(1.0, 1.0, 0.5);
	vertices.col(3) = Eigen::Vector3d(-1.0, 1.0, 0.5);
	Eigen::MatrixXi triangles(3, 2);
	triangles << 0, 0,
		1, 2,
		2, 3;
	MeshCPU mesh;
	mesh.setMesh(vertices, triangles);

	MeshManaged mesh_i;
	mesh_i.setMesh(mesh.getVertices(), mesh.getTriangles());
	mesh_i.initGL();

	assert(glGetError() == GL_NO_ERROR);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureId);
	assert(glGetError() == GL_NO_ERROR);

	m_screenTextureProgram.use();
	mesh_i.draw();
	assert(glGetError() == GL_NO_ERROR);
	mesh_i.shutGL();
	assert(glGetError() == GL_NO_ERROR);

	glBindTexture(GL_TEXTURE_2D, 0);
}

meshDrawer::meshDrawer()
{
	m_glInitialized = false;

	m_dimensionOrientationRendering = false;
	m_dor_width = 0.1;
	m_dor_height = 0.1;
	m_dor_pFrom = Eigen::Vector3d(0, 1, 0);
	m_dor_pTo = Eigen::Vector3d(0, 0, 0);
	m_dor_color = Eigen::Vector4d(0.7, 0.7, 0.7, 1.0);
	m_drawInViewer = true;
	//assert(glGetError() == GL_NO_ERROR);
}

meshDrawer::meshDrawer(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& triangles)
{
	m_glInitialized = false;
	setMesh(vertices, triangles);
	assert(glGetError() == GL_NO_ERROR);
}

meshDrawer::meshDrawer(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& colors)
{
	m_glInitialized = false;
	setMesh(vertices, triangles, colors);
	assert(glGetError() == GL_NO_ERROR);
}

meshDrawer::meshDrawer(Mesh* tri_mesh)
{
	m_glInitialized = false;
	setMesh(tri_mesh);
	assert(glGetError() == GL_NO_ERROR);
}

void meshDrawer::setMesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& triangles, bool flipNorm)
{
	Eigen::MatrixXd colors(3, vertices.cols() );
	for (int i = 0; i < vertices.cols(); i++)
		colors.col(i) = Eigen::Vector3d(0.7, 0.7, 0.7);

	setMesh(vertices, triangles, colors, flipNorm);
}


void meshDrawer::setMesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& colors, bool flipNorm)
{
	const auto& vertices_old = m_mesh.getCPUMesh().getVertices();
	
	if (vertices_old.size() == vertices.size())
		if(vertices_old.isApprox(vertices))
			return;

	m_mesh.setMesh(vertices, triangles, colors, flipNorm);

	m_dimensionOrientationRendering = false;
	m_dor_width = 0.1;
	m_dor_height = 0.1;
	m_dor_pFrom = Eigen::Vector3d(0, 1, 0);
	m_dor_pTo = Eigen::Vector3d(0, 0, 0);
	m_dor_color = Eigen::Vector4d(0.7, 0.7, 0.7, 1.0);
	m_drawInViewer = true;
}


template <typename DerivedV, typename DerivedF>
void meshDrawer::loadFromMesh(
	const Mesh* tri_mesh,
	Eigen::PlainObjectBase<DerivedV>& V,
	Eigen::PlainObjectBase<DerivedF>& F)
{
	const Mesh* loadMesh = tri_mesh;
	const Eigen::VectorXd& vertices = loadMesh->vertices();
	const std::vector<int>& faces = loadMesh->faces();

	V.resize(3, loadMesh->numVertices());
	F.resize(3, loadMesh->numTriangles());

	for (int i = 0; i < loadMesh->numVertices(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			V(j, i) = vertices(3 * i + j);
		}
	}

	for (int i = 0; i < loadMesh->numTriangles(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			F(j, i) = faces[3 * i + j];
		}
	}
}

void meshDrawer::setMesh(Mesh* tri_mesh, bool flipNorm)
{
	Eigen::MatrixXd vertices;
	Eigen::MatrixXi triangles;
	if (tri_mesh->getFlipNorm())
		flipNorm = tri_mesh->getFlipNorm();

	
	loadFromMesh(tri_mesh, vertices, triangles);
	m_mesh.setMesh(vertices, triangles, flipNorm);// , flipNorm

	m_dimensionOrientationRendering = false;
	m_dor_width = 0.1;
	m_dor_height = 0.1;
	m_dor_pFrom = Eigen::Vector3d(0, 1, 0);
	m_dor_pTo = Eigen::Vector3d(0, 0, 0);
	m_dor_color = Eigen::Vector4d(0.7, 0.7, 0.7, 1.0);
	m_drawInViewer = true;
}



void meshDrawer::setLineSegmentMesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& triangles)
{
	m_lines.setLines(vertices, triangles);
	m_dimensionOrientationRendering = false;
	m_dor_width = 0.1;
	m_dor_height = 0.1;
	m_dor_pFrom = Eigen::Vector3d(0, 1, 0);
	m_dor_pTo = Eigen::Vector3d(0, 0, 0);
	m_dor_color = Eigen::Vector4d(0.7, 0.7, 0.7, 1.0);
	m_drawInViewer = true;
}

void meshDrawer::updateLineSegmentVertices(const Eigen::MatrixXd& vertices)
{
	m_lines.updateVertices(vertices);
}

void meshDrawer::drawLineSegment()
{
	Eigen::Matrix4d model = computeTransformation(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));

	setupProgram(model);
	m_lines.draw();
	assert(glGetError() == GL_NO_ERROR);
}

void meshDrawer::initGL()
{
	if (m_glInitialized) shutGL();
	assert(glGetError() == GL_NO_ERROR);
	
	//program setup
	m_program = m_shaderCache->getProgram(ShaderPipelineDesc::VertexFragment(shaderFolder + "meshDrawer.vs", shaderFolder + "meshDrawer.fs"));
	//mesh initialization to GPU
	m_mesh.initGL();
	m_lines.initGL();
	
	m_glInitialized = true;
}

void meshDrawer::shutGL()
{
	if (!m_glInitialized) return;
	m_mesh.shutGL();
	m_lines.shutGL();
	m_glInitialized = false;
}

void meshDrawer::setupProgram(const Eigen::Matrix4d& model, float alpha, bool use_lighting)
{
	Eigen::Matrix4f model_float = Eigen::Matrix4f(model.cast<float>());
	setupShader(m_shadingSettings, model_float, m_program);
	assert(glGetError() == GL_NO_ERROR);

	m_program.setUniform1f("roughness", 0.1f);
	m_program.setUniform1f("color_alpha", alpha);
	m_program.setUniform1f("use_lighting", use_lighting);
}

void meshDrawer::updateVertices(const Eigen::MatrixXd& vertices)
{
	if(m_mesh.getCPUMesh().getVertices().rows()== vertices.rows() && m_mesh.getCPUMesh().getVertices().cols() == vertices.cols())
		m_mesh.updateVertices(vertices);
}

void meshDrawer::updateColors(const Eigen::MatrixXd& colors)
{
	//if ((m_mesh.getCPUMesh().getColors().rows() == colors.rows() && m_mesh.getCPUMesh().getColors().cols() == colors.cols())
	//	|| (m_mesh.getCPUMesh().getColors().rows() == 0 || m_mesh.getCPUMesh().getColors().cols() == 0))
		m_mesh.updateColors(colors);
}

void meshDrawer::updateMesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi triangles)
{
	m_mesh.updateMesh(vertices, triangles);
}

void meshDrawer::updateMesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi triangles, const Eigen::MatrixXd colors)
{
	m_mesh.updateMesh(vertices, triangles, colors);
}

Eigen::Matrix4d meshDrawer::computeTransformation(
	double scale,
	const Eigen::Vector3d& center)
{
	Eigen::Matrix4d model = Eigen::Matrix4d::Identity();
	model(0, 0) = scale;
	model(1, 1) = scale;
	model(2, 2) = scale;
	model.col(3).head(3) = center;
	return model;
}

void meshDrawer::draw(const Eigen::MatrixXd& colors, float alpha, bool use_lighting)
{
	Eigen::Matrix4d model = computeTransformation(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));

	updateColors(colors);

	setupProgram(model, alpha, use_lighting);
	m_mesh.draw();
	assert(glGetError() == GL_NO_ERROR);
}

void meshDrawer::drawWireframe(const Eigen::Vector4d& color)
{
	Eigen::Matrix4d model = computeTransformation(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));

	Eigen::MatrixXd colors(3, m_mesh.getCPUMesh().getVertices().cols());
	for (int i = 0; i < m_mesh.getCPUMesh().getVertices().cols(); i++)
		colors.col(i) = color.head<3>();
	updateColors(colors);

	setupProgram(model);
	m_mesh.drawWireframe();
	assert(glGetError() == GL_NO_ERROR);
}

void meshDrawer::drawWireframe(const Eigen::MatrixXd& colors)
{
	Eigen::Matrix4d model = computeTransformation(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));

	updateColors(colors);

	setupProgram(model);
	m_mesh.drawWireframe();
	assert(glGetError() == GL_NO_ERROR);
}

void meshDrawer::draw(
	double scale,
	const Eigen::Vector3d& center,
	const Eigen::MatrixXd& colors, 
	float alpha, bool use_lighting)
{

	Eigen::Matrix4d model = computeTransformation(scale, center);

	updateColors(colors);

	setupProgram(model, alpha, use_lighting);
	m_mesh.draw();
	assert(glGetError() == GL_NO_ERROR);
}

void meshDrawer::draw(const Eigen::Vector4d& color, bool use_lighting)
{
	Eigen::Matrix4d model = computeTransformation(1.0, Eigen::Vector3d(0.0, 0.0, 0.0));
	
	Eigen::MatrixXd colors(3, m_mesh.getCPUMesh().getVertices().cols());
	for (int i = 0; i < m_mesh.getCPUMesh().getVertices().cols(); i++)
		colors.col(i) = color.head<3>();
	updateColors(colors);

	//we use the first vertex alpha
	float alpha = color[3];
	setupProgram(model, alpha, use_lighting);
	m_mesh.draw();
	assert(glGetError() == GL_NO_ERROR);
}

void meshDrawer::draw(
	double scale,
	const Eigen::Vector3d& center,
	const Eigen::Vector4d& color,
	bool use_lighting
)
{
	Eigen::Matrix4d model = computeTransformation(scale, center);
	
	Eigen::MatrixXd colors(3, m_mesh.getCPUMesh().getVertices().cols());
	for (int i = 0; i < m_mesh.getCPUMesh().getVertices().cols(); i++)
		colors.col(i) = color.head<3>();
	updateColors(colors);

	//we use the first vertex alpha
	float alpha = color[3];
	setupProgram(model, alpha, use_lighting);
	m_mesh.draw();
	assert(glGetError() == GL_NO_ERROR);
}

Eigen::Matrix4d meshDrawer::computeDimensionOrientationTransformation(
	double width,
	double height,
	const Eigen::Vector3d& pFrom,
	const Eigen::Vector3d& pTo)
{
	Eigen::Vector3d diff = (pTo - pFrom);
	double length = diff.norm();
	Eigen::Quaternion<double> rot = Eigen::Quaternion<double>::FromTwoVectors(Eigen::Vector3d(0.0, 1.0, 0.0), diff);

	Eigen::DiagonalMatrix<double, 3> scale = Eigen::Scaling(width, length, height);

	Eigen::Matrix4d model = (rot * scale).matrix();
	model.col(3).head(3) = pFrom;
	return model;
}

void meshDrawer::drawDimensionOrientation(
	double width,
	double height,
	const Eigen::Vector3d& pFrom,
	const Eigen::Vector3d& pTo,
	const Eigen::Vector4d& color
)
{
	Eigen::Matrix4d model = computeDimensionOrientationTransformation(width, height, pFrom, pTo);
	Viewer::Material material(color);
	
	Eigen::MatrixXd colors(3, m_mesh.getCPUMesh().getVertices().cols());
	for (int i = 0; i < m_mesh.getCPUMesh().getVertices().cols(); i++)
		colors.col(i) = color.head<3>();
	updateColors(colors);

	m_mesh.draw();
	assert(glGetError() == GL_NO_ERROR);

}

void meshDrawer::draw_m_dimensionOrientation()
{
	Eigen::Matrix4d model = computeDimensionOrientationTransformation(m_dor_width, m_dor_height, m_dor_pFrom, m_dor_pTo);
	Viewer::Material material(m_dor_color);
	
	Eigen::MatrixXd colors(3, m_mesh.getCPUMesh().getVertices().cols());
	for (int i = 0; i < m_mesh.getCPUMesh().getVertices().cols(); i++)
		colors.col(i) = m_dor_color.head<3>();
	updateColors(colors);

	m_mesh.draw();
	assert(glGetError() == GL_NO_ERROR);
}
