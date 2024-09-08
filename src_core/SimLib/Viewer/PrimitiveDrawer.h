#pragma once
#ifndef SIM_OPT_PRIMITIVE_DRAWER
#define SIM_OPT_PRIMITIVE_DRAWER



#include "GL.h"
#include "Texture.h"
#include "Program.h"
#include "SimpleShadingSettings.h"
#include "ShaderCache.h"

#include <Eigen/Core>

#include <map>
#include <memory>
#include <vector>

class SimpleTriMesh;

namespace Viewer
{

class Material
{
public:
	Material(const Eigen::Vector3d &color, double alpha)
	{
		this->ambient = color;
		this->diffuse = color;
		this->specular = color;
		this->alpha = alpha;
	}
	Material(const Eigen::Vector4d &color)
	{
		this->ambient = color.segment<3>(0);
		this->diffuse = color.segment<3>(0);
		this->specular = color.segment<3>(0);
		this->alpha = color[3];
	}
	Material(const Eigen::Vector3d &ambient, const Eigen::Vector3d &diffuse, const Eigen::Vector3d &specular, double alpha)
	{
		this->ambient = ambient;
		this->diffuse = diffuse;
		this->specular = specular;
		this->alpha = alpha;
	}

	static Material Ambient(const Eigen::Vector3d &color)
	{
		return Material(color, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1.0);
	}

	Eigen::Vector3d	ambient;
	Eigen::Vector3d	diffuse;
	Eigen::Vector3d	specular;
	double alpha;
};

}

class shapeDrawer
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	shapeDrawer() {}
	~shapeDrawer() {}

	virtual void initGL() = 0;
	virtual void shutGL() = 0;
	void setShaderFolder(const std::string& folder)
	{
		shaderFolder = folder;
	}
	void setShaderCache(const std::shared_ptr<ShaderCache>& shaderCache)
	{
		m_shaderCache = shaderCache;
	}
	void setShadingSettings(const SimpleShadingSettings& settings)
	{
		m_shadingSettings = settings;
	}
protected:
	void setupProgram(const Eigen::Matrix4f& model, const Viewer::Material& material)
	{
		setupShader(m_shadingSettings, model, m_program);
		assert(glGetError() == GL_NO_ERROR);
		Eigen::Vector3d pFrom(model.col(3)[0], model.col(3)[1], model.col(3)[2]);
		m_program.setUniform("endPos1", Eigen::Vector3f(pFrom.cast<float>()));
		m_program.setUniform("endPos2", Eigen::Vector3f(pFrom.cast<float>()));

		m_program.setUniform("color_ambient1", Eigen::Vector3f(material.ambient.cast<float>()));
		m_program.setUniform("color_diffuse1", Eigen::Vector3f(material.diffuse.cast<float>()));
		m_program.setUniform("color_specular1", Eigen::Vector3f(material.specular.cast<float>()));
		m_program.setUniform1f("roughness1", 0.1f);
		m_program.setUniform1f("color_alpha1", float(material.alpha));

		m_program.setUniform("color_ambient2", Eigen::Vector3f(material.ambient.cast<float>()));
		m_program.setUniform("color_diffuse2", Eigen::Vector3f(material.diffuse.cast<float>()));
		m_program.setUniform("color_specular2", Eigen::Vector3f(material.specular.cast<float>()));
		m_program.setUniform1f("roughness2", 0.1f);
		m_program.setUniform1f("color_alpha2", float(material.alpha));
		assert(glGetError() == GL_NO_ERROR);
	}
	void setupProgram(const Eigen::Matrix4d& model, const Viewer::Material& material)
	{
		Eigen::Matrix4f m = model.cast<float>();
		this->setupProgram(m, material);
	}
	void setupProgram(const Eigen::Vector3d& pFrom, const Eigen::Vector3d& pTo, const Eigen::Matrix4f& model, const Viewer::Material& material1, const Viewer::Material& material2)
	{
		setupShader(m_shadingSettings, model, m_program);
		assert(glGetError() == GL_NO_ERROR);

		m_program.setUniform("endPos1", Eigen::Vector3f(pFrom.cast<float>()));
		m_program.setUniform("endPos2", Eigen::Vector3f(pTo.cast<float>()));

		m_program.setUniform("color_ambient1", Eigen::Vector3f(material1.ambient.cast<float>()));
		m_program.setUniform("color_diffuse1", Eigen::Vector3f(material1.diffuse.cast<float>()));
		m_program.setUniform("color_specular1", Eigen::Vector3f(material1.specular.cast<float>()));
		m_program.setUniform1f("roughness1", 0.1f);
		m_program.setUniform1f("color_alpha1", float(material1.alpha));

		m_program.setUniform("color_ambient2", Eigen::Vector3f(material2.ambient.cast<float>()));
		m_program.setUniform("color_diffuse2", Eigen::Vector3f(material2.diffuse.cast<float>()));
		m_program.setUniform("color_specular2", Eigen::Vector3f(material2.specular.cast<float>()));
		m_program.setUniform1f("roughness2", 0.1f);
		m_program.setUniform1f("color_alpha2", float(material2.alpha));
		assert(glGetError() == GL_NO_ERROR);
	}
	void setupProgram(const Eigen::Vector3d& pFrom, const Eigen::Vector3d& pTo, const Eigen::Matrix4d& model, const Viewer::Material& material1, const Viewer::Material& material2)
	{
		Eigen::Matrix4f m = model.cast<float>();
		this->setupProgram(pFrom, pTo, m, material1, material2);
	}
	void setupTexturedProgram(const Eigen::Matrix4f& model, int textureUnitIdx)
	{
		setupShader(m_shadingSettings, model, m_texturedProgram);
		assert(glGetError() == GL_NO_ERROR);
		m_texturedProgram.setUniform1i("texture_0", textureUnitIdx);
		assert(glGetError() == GL_NO_ERROR);
	}
	
protected:
	SimpleShadingSettings m_shadingSettings;

	ProgramGL m_program;
	ProgramGL m_texturedProgram;
	ProgramGL m_screenTextureProgram;

	std::shared_ptr<ShaderCache> m_shaderCache;

	bool m_glInitialized = false;

	std::string shaderFolder = "";
};


class PrimitiveDrawer: public shapeDrawer
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	PrimitiveDrawer();

	void initializeSphere(int nSubdivisions);

	void initGL();
	void shutGL();

	Eigen::Matrix4d computeSphereTransformation(
		double radius,
		const Eigen::Vector3d &center);

	void addSphereMesh(
		const Eigen::Matrix4d& model,
		SimpleTriMesh& mesh,
		int nSubdivisions = -1
		);
	void addSphereMesh(
		double radius,
		const Eigen::Vector3d& center,
		SimpleTriMesh& mesh,
		int nSubdivisions = -1
		);
	void drawSphere(
		double radius,
		const Eigen::Vector3d &center,
		const Eigen::Vector4d &color,
		int nSubdivisions = 1
	);

	Eigen::Matrix4d computeCuboidTransformation(std::tuple<double, double, double> cuboidDimensions,
		const Eigen::Vector3d& frame1,
		const Eigen::Vector3d& frame2,
		const Eigen::Vector3d& center);
	void drawCuboid(
		std::tuple<double, double, double> cuboidDimensions,
		const Eigen::Vector3d& frame1,
		const Eigen::Vector3d& frame2,
		const Eigen::Vector3d& center,
		const Eigen::Vector4d& color);

	Eigen::Matrix4d computeEllipsoidTransformation(std::pair<double, double> radius,
		const Eigen::Vector3d& frame1,
		const Eigen::Vector3d& frame2,
		const Eigen::Vector3d& center);

	void drawEllipsoid(
		std::pair<double, double> radius,
		const Eigen::Vector3d& frame1,
		const Eigen::Vector3d& frame2,
		const Eigen::Vector3d& center,
		const Eigen::Vector4d& color,
		int nSubdivisions = 1
	);

	Eigen::Matrix4d computeCylinderTransformation(
		double radius,
		const Eigen::Vector3d &pFrom,
		const Eigen::Vector3d &pTo);
	Eigen::Matrix4d computeCylinderTransformation(
		std::pair<double, double> radius,
		const Eigen::Vector3d& frame1,
		const Eigen::Vector3d& frame2,
		const Eigen::Vector3d& pFrom,
		const Eigen::Vector3d& pTo);

	void addCylinderMesh(
		const Eigen::Matrix4d& model,
		SimpleTriMesh& mesh
	);
	void addCylinderMesh(
		const Eigen::Matrix4d& model,
		int axis_devisions,
		SimpleTriMesh& mesh
	);
	void addCylinderMesh(
		double radius,
		const Eigen::Vector3d& pFrom,
		const Eigen::Vector3d& pTo,
		SimpleTriMesh& mesh
	);
	//draw circular cross section
	void drawCylinder(
		double radius,
		const Eigen::Vector3d &pFrom,
		const Eigen::Vector3d &pTo,
		const Viewer::Material &material
	);
	void drawCylinder(
		double radius,
		const Eigen::Vector3d& pFrom,
		const Eigen::Vector3d& pTo,
		const Viewer::Material& material1,
		const Viewer::Material& material2
	);
	//draw elliptical cross section
	void drawCylinder(
		std::pair<double, double> radius,
		const Eigen::Vector3d& frame1, 
		const Eigen::Vector3d& frame2,
		const Eigen::Vector3d& pFrom,
		const Eigen::Vector3d& pTo,
		const Viewer::Material& material
	);
	void drawCylinder(
		std::pair<double, double> radius,
		const Eigen::Vector3d& frame1,
		const Eigen::Vector3d& frame2,
		const Eigen::Vector3d& pFrom,
		const Eigen::Vector3d& pTo,
		const Viewer::Material& material1,
		const Viewer::Material& material2
	);

	void addConeMesh(
		const Eigen::Matrix4d &model,
		SimpleTriMesh& mesh
		);
	void addConeMesh(
		double radius,
		const Eigen::Vector3d& pFrom,
		const Eigen::Vector3d& pTo,
		SimpleTriMesh& mesh
		);

	void drawCone(
		double radius,
		const Eigen::Vector3d &pFrom,
		const Eigen::Vector3d &pTo,
		const Eigen::Vector4d &color
	);
	void drawCone(
		double radius,
		const Eigen::Vector3d& pFrom,
		const Eigen::Vector3d& pTo,
		const Eigen::Vector4d& color1,
		const Eigen::Vector4d& color2
	);

	Eigen::Matrix4d computeConicalFrustumTransformation(
		double radiusFrom,
		double radiusTo,
		const Eigen::Vector3d &pFrom,
		const Eigen::Vector3d &pTo);
	void addConicalFrustumMesh(
		double radiusFrom,
		double radiusTo,
		const Eigen::Vector3d& pFrom,
		const Eigen::Vector3d& pTo,
		SimpleTriMesh& mesh
	);
	void addConicalFrustumMesh(
		double radiusFrom,
		double radiusTo,
		const Eigen::Vector3d& pFrom,
		const Eigen::Vector3d& pTo,
		int axis_devisions,
		SimpleTriMesh& mesh
	);
	void drawConicalFrustum(
		double radiusFrom,
		double radiusTo,
		const Eigen::Vector3d &pFrom,
		const Eigen::Vector3d &pTo,
		const Eigen::Vector4d &color
	);
	/** alias for drawConicalFrustum */
	void drawTruncatedCone(
		double radiusFrom,
		double radiusTo,
		const Eigen::Vector3d &pFrom,
		const Eigen::Vector3d &pTo,
		const Eigen::Vector4d &color
	)
	{
		drawConicalFrustum(radiusFrom, radiusFrom, pFrom, pTo, color);
	}

	void addArrowMeshes(
		double radius,
		const Eigen::Vector3d& pFrom,
		const Eigen::Vector3d& pTo,
		SimpleTriMesh& mesh
	);

	void drawArrow(
		double radius,
		const Eigen::Vector3d &pFrom,
		const Eigen::Vector3d &pTo,
		const Viewer::Material &material
	);

	void drawArrow(
		double radius,
		const Eigen::Vector3d& pFrom,
		const Eigen::Vector3d& pTo,
		const Viewer::Material& material1,
		const Viewer::Material& material2
	);
	void drawTriangle(
		const Eigen::Vector3d &x1,
		const Eigen::Vector3d &x2,
		const Eigen::Vector3d &x3,
		const Viewer::Material &material
	);
	void drawTexturedQuad(
		const TextureGPU &gpuTexture,
		const Eigen::Matrix4d &modelMatrix
	);
	void drawTexturedQuad(
		GLuint textureId,
		const Eigen::Matrix4d &modelMatrix
	);
	void drawScreenQuad(
		GLuint textureId
	);

private:
	std::map<int, std::unique_ptr<MeshManaged> > m_spheres;
	MeshManaged m_cylinder;
	MeshManaged m_cone;
	MeshManaged m_triangle;
	MeshManaged m_cuboid;
};

class Mesh;
class meshDrawer :public shapeDrawer
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	meshDrawer();
	//vertices As Cols, tris As Cols
	meshDrawer(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& triangles);
	meshDrawer(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& colors);
	meshDrawer(Mesh* tri_mesh);
	template <typename DerivedV, typename DerivedF>
	void loadFromMesh(
		const Mesh *tri_mesh,
		Eigen::PlainObjectBase<DerivedV> &V,
		Eigen::PlainObjectBase<DerivedF> &F);
	void setMesh(const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &triangles, bool flipNorm = false);
	void setMesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& colors, bool flipNorm = false);
	void setMesh(Mesh* tri_mesh, bool flipNorm = false);

	void setLineSegmentMesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& triangles);
	void updateLineSegmentVertices(const Eigen::MatrixXd& vertices);
	void drawLineSegment();

	void initGL();
	void shutGL();
	void setupProgram(const Eigen::Matrix4d& model, float alpha = 1.0f, bool use_lighting = true);

	void updateVertices(const Eigen::MatrixXd& vertices);
	void updateColors(const Eigen::MatrixXd& colors);
	void updateMesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi triangles);
	void updateMesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi triangles, const Eigen::MatrixXd colors);

	Eigen::Matrix4d computeTransformation(
		double scale,
		const Eigen::Vector3d& center);
	
	void draw(const Eigen::MatrixXd& colors, float alpha = 1.0, bool use_lighting = true);
	void drawWireframe(const Eigen::Vector4d& color);
	void drawWireframe(const Eigen::MatrixXd& colors);
	void draw(
		double scale,
		const Eigen::Vector3d& center,
		const Eigen::MatrixXd& colors, 
		float alpha = 1.0, bool use_lighting = true);
	void draw(const Eigen::Vector4d& color, bool use_lighting = true);

	void draw(
		double scale,
		const Eigen::Vector3d& center,
		const Eigen::Vector4d& color, 
		bool use_lighting = true);

	Eigen::Matrix4d computeDimensionOrientationTransformation(
		double width,
		double height,
		const Eigen::Vector3d& pFrom,
		const Eigen::Vector3d& pTo);

	void drawDimensionOrientation(
		double width,
		double height,
		const Eigen::Vector3d& pFrom,
		const Eigen::Vector3d& pTo,
		const Eigen::Vector4d& color
	);
	void draw_m_dimensionOrientation();

	bool getDimensionOrientationRendering() { return m_dimensionOrientationRendering; }
	void setDimensionOrientationRendering(bool dimensionOrientationRendering) { m_dimensionOrientationRendering = dimensionOrientationRendering; }
	void setDimensionOrientationRenderingSetting(double dor_width,
		double dor_height,
		const Eigen::Vector3d& dor_pFrom,
		const Eigen::Vector3d& dor_pTo,
		const Eigen::Vector4d& dor_color = Eigen::Vector4d(0.7, 0.7, 0.7, 1.0)) {
		m_dor_width = dor_width; m_dor_height = dor_height; m_dor_pFrom = dor_pFrom; m_dor_pTo = dor_pTo; m_dor_color = dor_color;
	}
	void getDimensionOrientationRenderingSetting(double& dor_width,
		double& dor_height,
		Eigen::Vector3d& dor_pFrom,
		Eigen::Vector3d& dor_pTo) {
		dor_width = m_dor_width; dor_height = m_dor_height; dor_pFrom = m_dor_pFrom; dor_pTo = m_dor_pTo;
	}
	//string getNameFlag() { return m_name_flag; }
	//void setNameFlag(const string& nameflag) { m_name_flag = nameflag; }
	void setDrawInViwer(bool drawInViewer) { m_drawInViewer = drawInViewer; }
	bool getDrawInViwer() { return m_drawInViewer; }
private:
	bool m_dimensionOrientationRendering;//default is false
	double m_dor_width;
	double m_dor_height;
	Eigen::Vector3d m_dor_pFrom;
	Eigen::Vector3d m_dor_pTo;
	Eigen::Vector4d m_dor_color;
	//string m_name_flag;
	bool m_drawInViewer; //if this is true, then it will automatically draw in viewer, otherwise you can write your own code to draw it

	MeshManaged m_mesh;
	LinesManaged m_lines;
};


#endif