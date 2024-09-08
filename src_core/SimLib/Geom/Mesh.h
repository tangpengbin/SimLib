#ifndef MESH_H
#define MESH_H

#include <vector>
#include "../Core/SOTypes.h"

using namespace std;

class Mesh
{
public:
	Mesh();
	Mesh(const std::vector<Mesh*>& meshes);
	virtual bool loadFromFile(const char* szFileName) { return false; }
	const Eigen::VectorXd& vertices() const { return m_x; }
	Eigen::VectorXd& vertices() { return m_x; }
	Eigen::VectorXd& faceColors() { return m_faceColors; }
	const Eigen::VectorXd& faceColors() const { return m_faceColors; }
	const vector<int> & faces() const { return m_f; }
	vector<int> & faces() { return m_f; }
	bool isDirty() const { return m_dirty; }
	void setDirty(bool b) { m_dirty = b; }
	int numVertices() const { return m_x.size() / 3; }
	void translate(const Eigen::Vector3d& translation);
	void translate(double dx, double dy, double dz);
	void rotate(double rotation[3], int rotationOrder[3]);
	void rotate(const Eigen::Quaternion<double>& q);
	void scale(double scale[3]);
	//for now just assume all triangles. Should be extended to returning the number of triangles required for rendering polygonal mesh
	int numTriangles() const { return m_f.size() / 3; }
	void setFaceColor(const V3D& vc);
	void setFaceColor(const V3D& vc, int i);
	virtual void clear();
	bool getFlipNorm() { return m_flipNorm; }
	void setFlipNorm(bool flipNorm) { m_flipNorm = flipNorm; }

protected:
	//vertex positions
	Eigen::VectorXd m_x;
	//face indices
	vector<int> m_f;
	//face face to edges
	//face colors
	Eigen::VectorXd m_faceColors;
	//vector<int> m_f2e;
	//face edges to vertices
	vector<int> m_e2v;
	//dirty flag
	bool m_dirty;
	bool m_flipNorm; //this flag is used when using triangles to compute norms, we might need to flip these norms
};

#endif