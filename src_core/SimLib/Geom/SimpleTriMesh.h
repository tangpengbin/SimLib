#pragma once

#include "TriPair.h"

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/QR>

#include <vector>
#include <cmath>
#include <array>
#include <set>

template<typename Scalar, int Dim>
class Plane;
typedef Plane<double, 3> Plane3d;

struct EdgeWithOrientationFlag {
	std::array<int, 2> v;
	bool flipped;
};

class TriBaryCoords
{
public:
	TriBaryCoords(int triIdx, const Eigen::Vector2d &u)
		:m_triIdx(triIdx),
		m_u(u)
	{
	}

	int getTriIdx() const
	{
		return m_triIdx;
	}

	Eigen::Vector2d getXi() const
	{
		return m_u;
	}

	Eigen::Vector3d getN() const
	{
		Eigen::Vector3d N;
		N << 1.0 - m_u.sum(), m_u;
		return N;
	}

	Eigen::Vector2d computePosition(const Eigen::Vector2d &x0, const Eigen::Vector2d &x1, const Eigen::Vector2d &x2) const
	{
		Eigen::Vector3d n = getN();
		return n[0] * x0 + n[1] * x1 + n[2] * x2;
	}

	static Eigen::Vector2d compute(const Eigen::Vector2d &p, const Eigen::Vector2d &x0, const Eigen::Vector2d &x1, const Eigen::Vector2d &x2)
	{
		// p = [x0, x1, x2] * n(u) = [x0, x1, x2] * [1.0 - u[0] - u[1], u[0], u[1]
		// (p - x0)  = [x1 - x0, x2 - x0] * u
		Eigen::Matrix2d mat;
		mat << (x1 - x0), (x2 - x0);
		Eigen::Vector2d u = mat.colPivHouseholderQr().solve(p - x0);
		return u;
	}

	static Eigen::Vector3d computeN(const Eigen::Vector2d &bary)
	{
		Eigen::Vector3d N;
		N << 1.0 - bary.sum(), bary;
		return N;
	}
	// eps < 0 relaxed to the outside, eps > 0 more strict (covered area smaller)
	static bool isInterior(const Eigen::Vector2d & bary, double eps)
	{
		Eigen::Vector3d N = computeN(bary);
		return (N.array() > eps).all();
	}

private:
	Eigen::Vector2d m_u;
	int m_triIdx;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void upsampleTriMesh2(
	int oldVertexCount,
	const Eigen::MatrixXi &oldTris,
	int nAdditionalPointsPerEdge,
	std::vector<TriBaryCoords, Eigen::aligned_allocator<TriBaryCoords> > &baryCoords,
	Eigen::MatrixXi &newFaces);
void upsampleTriMeshKeepPointsAndIndexing( // we need to keep all old points to keep indexing
	int oldVertexCount,
	int oldTriCount,
	const std::function<Eigen::Vector3i(int i)>& getOldTri,
	int nAdditionalPointsPerEdge,
	std::vector<TriBaryCoords, Eigen::aligned_allocator<TriBaryCoords> >& newBaryCoords, //these are the added points, all original points stay!
	Eigen::MatrixXi& newFaces);

class SimpleTriMesh
{
public:
	typedef std::vector<Eigen::Vector3d>& VertexRange;

	static SimpleTriMesh mergeMeshes(const std::vector<SimpleTriMesh>& meshes);

	int addVertex(double x, double y, double z);
	int addVertex(const Eigen::Vector3d &v);
	const Eigen::Vector3d& getVertex(int vtxIdx) const;
	const Eigen::Vector3i& getTri(int triIdx) const { return m_triangles[triIdx]; }
	void addTri(int i1, int i2, int i3);
	void addTri(const Eigen::Vector3i &i)
	{
		return addTri(i[0], i[1], i[2]);
	}
	
	void addMesh(const Eigen::MatrixXd& verticesAsCols, const Eigen::MatrixXi& trisAsCols, const Eigen::Matrix4d& model);

	void setFromMatrices(const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &tris);
	void setFromVectors(const std::vector<Eigen::Vector3d> &vertices, const std::vector<std::array<int, 3> > &tris);
	Eigen::MatrixXd verticesToCols() const;
	Eigen::MatrixXi trianglesToCols() const;
	void toMatrices(Eigen::MatrixXd &vertices, Eigen::MatrixXi &tris) const;

	Eigen::VectorXd verticesToVector() const;

	std::vector<std::array<int, 2> > computeUniqueNotOrientedEdges() const;

	std::vector<EdgeWithOrientationFlag> computeEdgesWithOrientationFlags(std::function<bool(const EdgeWithOrientationFlag &e)> predicate) const;
	std::vector<EdgeWithOrientationFlag> computeUniqueEdgesWithOrientationFlags(const std::set<int> &vtxSelection) const;
	std::vector<int> computeFaces(std::function<bool(int faceIdx)> predicate) const;
	std::vector<int> computeFaces(const std::set<int> &vtxSelection) const;
	std::vector<SimOpt::TriPair> computeTriPairs() const;

	std::set<int> select(const Plane3d &plane) const;
	//note that these functions currently dont work exactly as in a 3d modelling tool....
	SimpleTriMesh extrudeVertices(const std::set<int> &selection, const Eigen::Vector3d &offset) const;

	TriBaryCoords computeClosestPoint(const Eigen::Vector3d& p);

	std::vector<EdgeWithOrientationFlag> constructBoundaryEdgesWithOrientationFlag() const;
	SimpleTriMesh extrudeFaces(const Eigen::Vector3d &offset) const;

	void translate(const Eigen::Vector3d &offset);
	void transform(const Eigen::Matrix4d& model);

	void upsample(int nAdditionalPointsPerEdge);
	/**this method keeps vertices which are not part of triangles and keeps the indexing order of the old vertices */
	void upsampleKeep(int nAdditionalPointsPerEdge);

	void writeObj(const char * filename) const;

	VertexRange vertices()
	{
		return m_vertices;
	}

	int getVertexCount() const {
		return (int)m_vertices.size();
	}

	int getTriCount() const {
		return (int)m_triangles.size();
	}

	static SimpleTriMesh icosphere();
	static SimpleTriMesh icosphere(int nSubdivisions);

private:
	std::vector<Eigen::Vector3d> m_vertices;
	std::vector<Eigen::Vector3i> m_triangles;
};
