#include "SimpleTriMesh.h"

#include "Plane.h"
#include "../Core/Algorithm.h"

#include <Eigen/StdVector>

#include <map>
#include <fstream>
#include <iomanip>

SimpleTriMesh SimpleTriMesh::mergeMeshes(const std::vector<SimpleTriMesh>& meshes)
{
	SimpleTriMesh result;

	std::vector<int> idcs;
	for (int i = 0; i < meshes.size(); i++)
	{
		const SimpleTriMesh& mesh = meshes[i];
		idcs.clear();
		for (int j = 0; j < mesh.getVertexCount(); j++)
		{
			idcs.push_back(result.addVertex(mesh.getVertex(j)));
		}
		for (int j = 0; j < mesh.getTriCount(); j++)
		{
			Eigen::Vector3i tri = mesh.getTri(j);
			int i1 = idcs[tri[0]];
			int i2 = idcs[tri[1]];
			int i3 = idcs[tri[2]];
			result.addTri(i1, i2, i3);
		}
	}

	return result;
}
int SimpleTriMesh::addVertex(double x, double y, double z)
{
	int vtxIdx = (int)m_vertices.size();
	m_vertices.push_back(Eigen::Vector3d(x, y, z));
	return vtxIdx;
}
int SimpleTriMesh::addVertex(const Eigen::Vector3d &v) {
	int vtxIdx = (int)m_vertices.size();
	m_vertices.push_back(v);
	return vtxIdx;
}
const Eigen::Vector3d& SimpleTriMesh::getVertex(int vtxIdx) const {
	return m_vertices[vtxIdx];
}
void SimpleTriMesh::addTri(int i1, int i2, int i3)
{
	m_triangles.push_back(Eigen::Vector3i(i1, i2, i3));
}
void SimpleTriMesh::addMesh(const Eigen::MatrixXd& verticesAsCols, const Eigen::MatrixXi& trisAsCols, const Eigen::Matrix4d& model)
{
	SimpleTriMesh mesh;
	mesh.setFromMatrices(verticesAsCols, trisAsCols);
	mesh.transform(model);
	std::vector<int> idcs;
	for (int j = 0; j < mesh.getVertexCount(); j++)
	{
		idcs.push_back(this->addVertex(mesh.getVertex(j)));
	}
	for (int j = 0; j < mesh.getTriCount(); j++)
	{
		Eigen::Vector3i tri = mesh.getTri(j);
		int i1 = idcs[tri[0]];
		int i2 = idcs[tri[1]];
		int i3 = idcs[tri[2]];
		this->addTri(i1, i2, i3);
	}
}

void SimpleTriMesh::setFromMatrices(const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &tris)
{
	m_vertices.resize(vertices.cols());
	m_triangles.resize(tris.cols());

	for (int i = 0; i < (int)m_vertices.size(); i++)
	{
		m_vertices[i] = vertices.col(i);
	}
	for (int i = 0; i < (int)m_triangles.size(); i++)
	{
		m_triangles[i] = tris.col(i);
	}
}
void SimpleTriMesh::setFromVectors(const std::vector<Eigen::Vector3d> &vertices, const std::vector<std::array<int, 3> > &tris) {
	m_vertices = vertices;
	m_triangles.resize(tris.size());
	for (int i = 0; i < (int)m_triangles.size(); i++)
	{
		for (int j = 0; j < 3; j++) {
			m_triangles[i][j] = tris[i][j];
		}
	}
}
Eigen::MatrixXd SimpleTriMesh::verticesToCols() const
{
	Eigen::MatrixXd vertices(3, m_vertices.size());
	for (int i = 0; i < (int)m_vertices.size(); i++)
	{
		vertices.col(i) = m_vertices[i];
	}
	return vertices;
}
Eigen::MatrixXi SimpleTriMesh::trianglesToCols() const
{
	Eigen::MatrixXi tris(3, m_triangles.size());
	for (int i = 0; i < (int)m_triangles.size(); i++)
	{
		tris.col(i) = m_triangles[i];
	}
	return tris;
}
void SimpleTriMesh::toMatrices(Eigen::MatrixXd &vertices, Eigen::MatrixXi &tris) const
{
	vertices.resize(3, m_vertices.size());
	tris.resize(3, m_triangles.size());

	for (int i = 0; i < (int)m_vertices.size(); i++)
	{
		vertices.col(i) = m_vertices[i];
	}
	for (int i = 0; i < (int)m_triangles.size(); i++)
	{
		tris.col(i) = m_triangles[i];
	}
}
Eigen::VectorXd SimpleTriMesh::verticesToVector() const {
	Eigen::VectorXd v(getVertexCount() * 3);
	for (int i = 0; i < (int)m_vertices.size(); i++)
	{
		v.segment<3>(3 * i) = m_vertices[i];
	}
	return v;
}
std::vector<std::array<int, 2> > SimpleTriMesh::computeUniqueNotOrientedEdges() const
{
	std::set< std::array<int, 2> > s;
	for (int i = 0; i < (int)m_triangles.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			std::array<int, 2> a = { m_triangles[i][j], m_triangles[i][(j+1)%3] };
			if (a[0] > a[1]) std::swap(a[0], a[1]);
			s.emplace(a);
		}
	}
	return std::vector<std::array<int, 2> >(s.begin(), s.end());
}
std::vector<EdgeWithOrientationFlag> SimpleTriMesh::computeEdgesWithOrientationFlags(std::function<bool(const EdgeWithOrientationFlag &e)> predicate) const {
	std::vector<EdgeWithOrientationFlag> edges;
	for (int i = 0; i < m_triangles.size(); i++) {
		for (int j = 0; j < 3; j++) {
			EdgeWithOrientationFlag edge;
			edge.v[0] = m_triangles[i][j];
			edge.v[1] = m_triangles[i][(j + 1) % 3];
			if (edge.v[1] < edge.v[0]) {
				std::swap(edge.v[0], edge.v[1]);
				edge.flipped = true;
			} else {
				edge.flipped = false;
			}
			if (predicate(edge)) {
				edges.emplace_back(edge);
			}
		}
	}
	return edges;
}
std::vector<EdgeWithOrientationFlag> SimpleTriMesh::computeUniqueEdgesWithOrientationFlags(const std::set<int> &vtxSelection) const {

	std::vector<EdgeWithOrientationFlag> edges = this->computeEdgesWithOrientationFlags([&vtxSelection](const EdgeWithOrientationFlag &e) {
		return vtxSelection.find(e.v[0]) != vtxSelection.end() && vtxSelection.find(e.v[1]) != vtxSelection.end();
	});

	//remove duplicate edges by sorting and then using unique
	std::sort(edges.begin(), edges.end(), [](const EdgeWithOrientationFlag& e1, const EdgeWithOrientationFlag &e2) {
		return e1.v[0] < e2.v[0]
			|| (e1.v[0] == e2.v[0] && e1.v[1] < e2.v[1]);
	});
	SimOpt::unique(edges, [](const EdgeWithOrientationFlag& e1, const EdgeWithOrientationFlag &e2) {
		return e1.v[0] == e2.v[0] && e1.v[1] == e2.v[1];
	});
	return edges;
}
std::vector<int> SimpleTriMesh::computeFaces(std::function<bool(int faceIdx)> predicate) const
{
	std::vector<int> result;
	for (int i = 0; i < m_triangles.size(); i++)
	{
		if (predicate(i))
		{
			result.push_back(i);
		}
	}
	return result;
}
std::vector<int> SimpleTriMesh::computeFaces(const std::set<int> &vtxSelection) const
{
	return computeFaces([this, &vtxSelection](int triIdx) {
		auto &tri = this->m_triangles[triIdx];
		return vtxSelection.find(tri[0]) != vtxSelection.end()
			&& vtxSelection.find(tri[1]) != vtxSelection.end()
			&& vtxSelection.find(tri[2]) != vtxSelection.end();
		});
}
std::vector<SimOpt::TriPair> SimpleTriMesh::computeTriPairs() const
{
	std::map<std::array<int, 2>, std::vector<int> > edgeMap;

	std::vector<SimOpt::TriPair> triPairs;

	auto & faces = m_triangles;
	int nFaces = faces.size();

	for (int triIdx = 0; triIdx < nFaces; triIdx++)
	{
		for (int j = 0; j < 3; j++)
		{
			std::array<int, 2> e = { faces[triIdx][j], faces[triIdx][((j + 1) % 3)] };
			if (e[0] > e[1])
			{
				std::swap(e[0], e[1]);
			}

			for (int triIdx2 : edgeMap[e])
			{
				SimOpt::TriPair p;
				p.tris[0] = triIdx;
				p.tris[1] = triIdx2;
				p.vtcs[0] = faces[triIdx][((j + 2) % 3)];
				p.vtcs[1] = faces[triIdx][((j + 0) % 3)];
				p.vtcs[2] = faces[triIdx][((j + 1) % 3)];

				if (faces[triIdx2][0] != p.vtcs[1] && faces[triIdx2][0] != p.vtcs[2])
				{
					p.vtcs[3] = faces[triIdx2][0];
				}
				else if (faces[triIdx2][1] != p.vtcs[1] && faces[triIdx2][1] != p.vtcs[2])
				{
					p.vtcs[3] = faces[triIdx2][1];
				}
				else if (faces[triIdx2][2] != p.vtcs[1] && faces[triIdx2][2] != p.vtcs[2])
				{
					p.vtcs[3] = faces[triIdx2][2];
				}
				else
				{
					p.vtcs[3] = -1;
					throw std::invalid_argument("error in mesh ");
				}

				triPairs.push_back(p);
			}
			edgeMap[e].emplace_back(triIdx);
		}
	}

	return triPairs;
}
std::set<int> SimpleTriMesh::select(const Plane3d &plane) const {
	std::set<int> result;
	for (std::size_t i = 0; i < m_vertices.size(); i++) {
		if (plane.eval(m_vertices[i]) <= 0.0) {
			result.emplace(i);
		}
	}
	return result;
}
SimpleTriMesh SimpleTriMesh::extrudeVertices(const std::set<int> &selection, const Eigen::Vector3d &offset) const {
	SimpleTriMesh copy = *this;
	
	//we need a map from old to new vertices
	// and for each edge we create two new tris
	// we try to preserve orientation for cases where we are just extruding on the boundary
	std::vector<EdgeWithOrientationFlag> edges = computeUniqueEdgesWithOrientationFlags(selection);

	//copy vertices and construct map from old to new vertex
	std::map<int, int> m;
	for (int vtxIdx : selection) {
		int nVertexIdx = copy.addVertex(copy.getVertex(vtxIdx) + offset);
		m.emplace(vtxIdx, nVertexIdx);
	}

	for (const EdgeWithOrientationFlag &edge : edges) {

		int v0 = edge.v[0];
		int v1 = edge.v[1];
		if (edge.flipped) std::swap(v0, v1);
		// on boundary we now have following orientation
		///  nv1------nv0
		//// |         |
		///  v1 ----- v0
		//      \    /
		//        v2

		int nv0 = m[v0];
		int nv1 = m[v1];

		copy.addTri(v1, v0, nv0);
		copy.addTri(v1, nv0, nv1);
	}

	return copy;
}
TriBaryCoords SimpleTriMesh::computeClosestPoint(const Eigen::Vector3d& p)
{
	TriBaryCoords closestCoords(-1, Eigen::Vector2d(0.0, 0.0));
	double closestDist = 1e300;
	for (int i = 0; i < (int)m_triangles.size(); i++)
	{
		const Eigen::Vector3i& tri = m_triangles[i];
		std::array<Eigen::Vector3d, 3> ps = {
			m_vertices[tri[0]],
			m_vertices[tri[1]],
			m_vertices[tri[2]]
		};

		// p = [x0, x1, x2] * [1.0 - u[0] - u[1], u[0], u[1]]
		// (p - x0)  = [x1 - x0, x2 - x0] * [u[0], u[1]]
		Eigen::Matrix<double, 3, 2> A;
		A << ps[1] - ps[0], ps[2] - ps[0];
		Eigen::Vector2d u = A.colPivHouseholderQr().solve(p - ps[0]);
		if (u[0] < 0.0 || u[1] < 0.0 || u[0] + u[1] > 1.0)
		{
			for (int j = 0; j < 3; j++)
			{
				Eigen::Vector3d x0 = ps[j];
				Eigen::Vector3d x1 = ps[(j + 1) % 3];

				Eigen::Matrix<double, 3, 1> B;
				B << x1 - x0;
				Eigen::Matrix<double, 1, 1> l = B.colPivHouseholderQr().solve(p - x0);
				double lambda = std::max(0.0, std::min(1.0, l[0]));
				Eigen::Vector3d pi = (1.0 - lambda) * x0 + lambda * x1;
				double dist = (p - pi).squaredNorm();
				if (dist < closestDist)
				{
					closestDist = dist;
					Eigen::Vector3d q(0.0, 0.0, 0.0);
					q[j] = 1.0 - lambda;
					q[(j + 1) % 3] = lambda;
					closestCoords = TriBaryCoords(i, Eigen::Vector2d(q[1], q[2]));

					Eigen::Vector3d NTest = closestCoords.getN();
					Eigen::Vector3d pos = NTest[0] * ps[0] + NTest[1] * ps[1] + NTest[2] * ps[2];
					Eigen::Vector3d pos2 = (1.0 - lambda) * x0 + lambda * x1;
					int kdkfjo = 3;
				}
			}
		}
		else
		{
			TriBaryCoords insideCoords(i, u);

			Eigen::Vector3d N = insideCoords.getN();
			Eigen::Vector3d pos = N[0] * ps[0] + N[1] * ps[1] + N[2] * ps[2];
			double dist = (p - pos).squaredNorm();
			if (dist < closestDist)
			{
				closestDist = dist;
				closestCoords = insideCoords;
			}
		}

	}

	return closestCoords;
}
template <class Type, class Predicate>
void remove_if_duplicates_exist(std::vector<Type> &values,
	Predicate predEqual)
{
	//auto first = values.begin();
	//auto last = values.end();
	//if (first == last)
	//	return last;

	//auto result = first;
	//++first;
	//while (first != last)
	//{
	//	if (!predEqual(*result, *first) && ++result != first)
	//	{
	//		*result = std::move(*first);
	//	}
	//	++first;
	//}
	//return ++result;

	auto last = values.end();
	auto target = values.begin();
	for (auto first = values.begin(); first != last; )
	{
		//first determine whether we remove this set of values
		auto next = first;
		++next;
		for (; next != last; next++)
		{
			if (!predEqual(*next, *first))
			{
				break;
			}
		}
		std::size_t elementCount = next - first;
		if (elementCount > 1)
		{
			//we want to remove this set of values, so we dont move them

		}
		else {
			// we want to keep this value so copy it to the beginning
			if (target != first) (*target) = std::move(*first);
			++target;
		}
		first = next;
	}
	values.resize(target - values.begin());

}
std::vector<EdgeWithOrientationFlag> SimpleTriMesh::constructBoundaryEdgesWithOrientationFlag() const
{
	std::vector<EdgeWithOrientationFlag> edges = this->computeEdgesWithOrientationFlags([](const EdgeWithOrientationFlag &e) {
		return true;
		});

	//remove edges which exist more than once by sorting and then removing
	std::sort(edges.begin(), edges.end(), [](const EdgeWithOrientationFlag& e1, const EdgeWithOrientationFlag &e2) {
		return e1.v[0] < e2.v[0]
			|| (e1.v[0] == e2.v[0] && e1.v[1] < e2.v[1]);
		});
	remove_if_duplicates_exist(edges, [](const EdgeWithOrientationFlag& e1, const EdgeWithOrientationFlag &e2) {
		return e1.v[0] == e2.v[0] && e1.v[1] == e2.v[1];
		});
	return edges;
}
SimpleTriMesh SimpleTriMesh::extrudeFaces(const Eigen::Vector3d &offset) const
{
	SimpleTriMesh copy = *this;

	int vtxOffset = (int)m_vertices.size();

	std::vector<EdgeWithOrientationFlag> boundaryEdges = constructBoundaryEdgesWithOrientationFlag();

	//copy all vertices and edges offset
	for (int vtxIdx = 0; vtxIdx < this->m_vertices.size(); vtxIdx++)
	{
		copy.addVertex(m_vertices[vtxIdx] + offset);
	}
	//copy triangles inverted!
	for (int triIdx = 0; triIdx < this->m_triangles.size(); triIdx++)
	{
		auto &tri = m_triangles[triIdx];
		copy.addTri(tri[0] + vtxOffset, tri[2] + vtxOffset, tri[1] + vtxOffset);
	}

	for (const EdgeWithOrientationFlag &edge : boundaryEdges)
	{

		int v0 = edge.v[0];
		int v1 = edge.v[1];
		if (edge.flipped) std::swap(v0, v1);
		// on boundary we now have following orientation
		///  nv1------nv0
		//// |         |
		///  v1 ----- v0
		//      \    /
		//        v2

		int nv0 = v0 + vtxOffset;
		int nv1 = v1 + vtxOffset;

		copy.addTri(v1, v0, nv0);
		copy.addTri(v1, nv0, nv1);
	}

	return copy;
}
void SimpleTriMesh::translate(const Eigen::Vector3d &offset)
{
	for (int i = 0; i < m_vertices.size(); i++)
	{
		m_vertices[i] += offset;
	}
}
void SimpleTriMesh::transform(const Eigen::Matrix4d& model)
{
	for (int i = 0; i < m_vertices.size(); i++)
	{
		Eigen::Vector4d v(m_vertices[i][0], m_vertices[i][1], m_vertices[i][2], 1.0);
		Eigen::Vector4d v2 = model * v;
		m_vertices[i] = v2.head<3>() / v2[3];
	}
}

template<typename F>
void upsampleTriMesh(
	int oldVertexCount,
	int oldTriCount,
	F &getOldTri,
	int nAdditionalPointsPerEdge,
	std::vector<TriBaryCoords, Eigen::aligned_allocator<TriBaryCoords> > &baryCoords,
	Eigen::MatrixXi &newFaces)
{
	baryCoords.clear();

	int numPointsPerEdge = nAdditionalPointsPerEdge + 2;
	int numTrisPerOldFace = (numPointsPerEdge - 1)*(numPointsPerEdge - 1);
	int numTris = numTrisPerOldFace * oldTriCount;

	std::vector<int> oldVertexToBaryCoords(oldVertexCount, -1);
	std::vector<std::map<std::pair<int, int>, int> > edgeVtxToVtxIdx(oldVertexCount);
	std::vector<std::map<std::pair<int, int>, int> > faceVtxToVtxIdx(oldTriCount);

	auto getOldVertexIdx = [&oldVertexToBaryCoords, &baryCoords](int oldTriIdx, int v, const Eigen::Vector2d &u) {
		if (oldVertexToBaryCoords[v] == -1)
		{
			baryCoords.push_back(TriBaryCoords(oldTriIdx, u));
			oldVertexToBaryCoords[v] = (int)baryCoords.size() - 1;
		}
		return oldVertexToBaryCoords[v];
	};
	auto getVertexIdxOnEdge = [numPointsPerEdge, &getOldVertexIdx, &baryCoords, &edgeVtxToVtxIdx](
		int oldTriIdx, int v0, const Eigen::Vector2d &u0, int v1, const Eigen::Vector2d &u1, int vtxIdxOnEdge) {
			if (vtxIdxOnEdge == 0) return getOldVertexIdx(oldTriIdx, v0, u0);
			if (vtxIdxOnEdge == numPointsPerEdge - 1) return getOldVertexIdx(oldTriIdx, v1, u1);

			//idea always store wrt smaller vertex idx
			// but we have (numPointsPerEdge -2) vertices inbetween which are indexed

			Eigen::Vector2d uCopy0 = u0;
			Eigen::Vector2d uCopy1 = u1;
			if (v0 > v1)
			{
				std::swap(v0, v1);
				Eigen::Vector2d ut = uCopy0;
				uCopy0 = uCopy1;
				uCopy1 = ut;
				vtxIdxOnEdge = numPointsPerEdge - 1 - vtxIdxOnEdge;
			}

			std::pair<int, int> p = std::make_pair(v1, vtxIdxOnEdge);

			if (edgeVtxToVtxIdx[v0].find(p) == edgeVtxToVtxIdx[v0].end())
			{
				double lambda = vtxIdxOnEdge / double(numPointsPerEdge - 1);
				Eigen::Vector2d u = (1.0 - lambda) * uCopy0 + lambda * uCopy1;
				baryCoords.push_back(TriBaryCoords(oldTriIdx, u));
				int newVtxIdx = (int)baryCoords.size() - 1;
				edgeVtxToVtxIdx[v0].emplace(p, newVtxIdx);
			}

			return edgeVtxToVtxIdx[v0].find(p)->second;
	};

	auto getVertexIdx = [&getOldTri, &getVertexIdxOnEdge, &baryCoords, numPointsPerEdge, &faceVtxToVtxIdx](int oldTriIdx, int row, int col) {
		Eigen::Vector3i tri = getOldTri(oldTriIdx);

		Eigen::Vector2d u0(0.0, 0.0);
		Eigen::Vector2d u1(1.0, 0.0);
		Eigen::Vector2d u2(0.0, 1.0);

		if (row == 0)
		{
			return getVertexIdxOnEdge(oldTriIdx, tri[0], u0, tri[1], u1, col);
		}
		if (col == 0)
		{
			return getVertexIdxOnEdge(oldTriIdx, tri[0], u0, tri[2], u2, row);
		}
		if (col + row == numPointsPerEdge - 1)
		{
			return getVertexIdxOnEdge(oldTriIdx, tri[1], u1, tri[2], u2, row);
		}

		std::pair<int, int> p = std::make_pair(row, col);
		if (faceVtxToVtxIdx[oldTriIdx].find(p) == faceVtxToVtxIdx[oldTriIdx].end())
		{
			int numPointsInRow = numPointsPerEdge - row;

			double lambda = row / double(numPointsPerEdge - 1);
			double lambda2 = col / double(numPointsInRow - 1);

			//Eigen::Vector2d p1 = (1.0 - lambda) * u0 + lambda * u2;
			//Eigen::Vector2d p2 = (1.0 - lambda) * u1 + lambda * u2;

			//Eigen::Vector2d u = (1.0 - lambda2) * p1 + lambda2 * p2;

			Eigen::Vector2d u(lambda2 * (1.0 - lambda), lambda);

			//if (numPointsInRow == 1) u = Eigen::Vector2d(0.0, lambda);

			baryCoords.push_back(TriBaryCoords(oldTriIdx, u));

			int newVtxIdx = (int)baryCoords.size() - 1;
			faceVtxToVtxIdx[oldTriIdx].emplace(p, newVtxIdx);
		}
		return faceVtxToVtxIdx[oldTriIdx].find(p)->second;
	};

	newFaces.resize(3, numTris);

	for (int oldTriIdx = 0; oldTriIdx < oldTriCount; oldTriIdx++)
	{
		int ciTri = 0;
		for (int row = 0; row < numPointsPerEdge - 1; row++)
		{
			//  |
			//  | \           ^
			//  |===\...      |
			//  |\ | \       row
			//  | \|  \
			//  |------\...
			//    col ->
			int numTrisInRow = 2 * numPointsPerEdge - 3 - 2 * row;
			for (int k = 0, baseVertex = 0; k < numTrisInRow; k++, baseVertex++)
			{
				Eigen::Vector3i newTri;
				newTri[0] = getVertexIdx(oldTriIdx, row, baseVertex);
				newTri[1] = getVertexIdx(oldTriIdx, row, baseVertex + 1);
				newTri[2] = getVertexIdx(oldTriIdx, row + 1, baseVertex);
				newFaces.col(oldTriIdx * numTrisPerOldFace + ciTri) = newTri;
				ciTri += 1;

				k++;
				if (k >= numTrisInRow) break;

				newTri[0] = getVertexIdx(oldTriIdx, row, baseVertex + 1);
				newTri[1] = getVertexIdx(oldTriIdx, row + 1, baseVertex + 1);
				newTri[2] = getVertexIdx(oldTriIdx, row + 1, baseVertex);
				newFaces.col(oldTriIdx * numTrisPerOldFace + ciTri) = newTri;
				ciTri += 1;
			}
		}
		assert(ciTri == numTrisPerOldFace);
	}

}
void upsampleTriMesh2(
	int oldVertexCount,
	const Eigen::MatrixXi &oldTris,
	int nAdditionalPointsPerEdge,
	std::vector<TriBaryCoords, Eigen::aligned_allocator<TriBaryCoords> > &baryCoords,
	Eigen::MatrixXi &newFaces)
{
	auto getOldTri = [&oldTris](int i) { return oldTris.col(i); };
	upsampleTriMesh<decltype(getOldTri)>(oldVertexCount, (int)oldTris.cols(), getOldTri, nAdditionalPointsPerEdge, baryCoords, newFaces);
}
void upsampleTriMesh3(
	const std::vector<Eigen::Vector3d> &vertices,
	const std::vector<Eigen::Vector3i> &tris,
	int nAdditionalPointsPerEdge,
	Eigen::MatrixXd &newVertices,
	Eigen::MatrixXi &newFaces
	)
{
	std::vector<TriBaryCoords, Eigen::aligned_allocator<TriBaryCoords> > baryCoords;
	auto getOldTri = [&tris](int i) { return tris[i]; };
	upsampleTriMesh<decltype(getOldTri)>((int)vertices.size(), (int)tris.size(), getOldTri, nAdditionalPointsPerEdge, baryCoords, newFaces);
	newVertices.resize(3, baryCoords.size());
	for (int i = 0; i < baryCoords.size(); i++)
	{
		const Eigen::Vector3i &tri = tris[baryCoords[i].getTriIdx()];
		Eigen::Vector3d N = baryCoords[i].getN();
		Eigen::Vector3d v = Eigen::Vector3d::Zero();
		for (int j = 0; j < 3; j++)
		{
			v += N[j] * vertices[tri[j]];
		}
		newVertices.col(i) = v;
	}
}

void SimpleTriMesh::upsample(int nAdditionalPointsPerEdge)
{
	if (nAdditionalPointsPerEdge == 0) return;
	//the following implementation removes non used vertices

	Eigen::MatrixXd newVertices;
	Eigen::MatrixXi newFaces;
	upsampleTriMesh3(
		m_vertices, m_triangles,
		nAdditionalPointsPerEdge,
		newVertices, newFaces
	);

	setFromMatrices(newVertices, newFaces);
}

void upsampleTriMeshKeepPointsAndIndexing( // we need to keep all old points to keep indexing
	int oldVertexCount,
	int oldTriCount,
	const std::function<Eigen::Vector3i(int i)>& getOldTri,
	int nAdditionalPointsPerEdge,
	std::vector<TriBaryCoords, Eigen::aligned_allocator<TriBaryCoords> >& newBaryCoords, //these are the added points, all original points stay!
	Eigen::MatrixXi& newFaces)
{
	newBaryCoords.clear();

	//this method doesnt take edges as input explicitly so we cannot upsample it...

	//int newEdgeVertexCount = nEdges * nAdditionalPointsPerEdge;
	//int newVertexCount = oldVertexCount + newEdgeVertexCount + newInteriorVertexCount ;

	int numPointsPerEdge = nAdditionalPointsPerEdge + 2;
	int numTrisPerOldFace = (numPointsPerEdge - 1) * (numPointsPerEdge - 1);
	int numTris = numTrisPerOldFace * oldTriCount;

	std::vector<std::map<std::pair<int, int>, int> > edgeVtxToVtxIdx(oldVertexCount);
	std::vector<std::map<std::pair<int, int>, int> > faceVtxToVtxIdx(oldTriCount);

	auto getOldVertexIdx = [](int oldTriIdx, int v, const Eigen::Vector2d& u) {
		return v;
	};
	auto getVertexIdxOnEdge = [oldVertexCount, numPointsPerEdge, &getOldVertexIdx, &newBaryCoords, &edgeVtxToVtxIdx](
		int oldTriIdx, int v0, const Eigen::Vector2d& u0, int v1, const Eigen::Vector2d& u1, int vtxIdxOnEdge) {
			if (vtxIdxOnEdge == 0) return getOldVertexIdx(oldTriIdx, v0, u0);
			if (vtxIdxOnEdge == numPointsPerEdge - 1) return getOldVertexIdx(oldTriIdx, v1, u1);

			//idea always store wrt smaller vertex idx
			// but we have (numPointsPerEdge -2) vertices inbetween which are indexed

			Eigen::Vector2d uCopy0 = u0;
			Eigen::Vector2d uCopy1 = u1;
			if (v0 > v1)
			{
				std::swap(v0, v1);
				Eigen::Vector2d ut = uCopy0;
				uCopy0 = uCopy1;
				uCopy1 = ut;
				vtxIdxOnEdge = numPointsPerEdge - 1 - vtxIdxOnEdge;
			}

			std::pair<int, int> p = std::make_pair(v1, vtxIdxOnEdge);

			if (edgeVtxToVtxIdx[v0].find(p) == edgeVtxToVtxIdx[v0].end())
			{
				double lambda = vtxIdxOnEdge / double(numPointsPerEdge - 1);
				Eigen::Vector2d u = (1.0 - lambda) * uCopy0 + lambda * uCopy1;
				newBaryCoords.push_back(TriBaryCoords(oldTriIdx, u));
				int newVtxIdx = oldVertexCount + (int)newBaryCoords.size() - 1;
				edgeVtxToVtxIdx[v0].emplace(p, newVtxIdx);
			}

			return edgeVtxToVtxIdx[v0].find(p)->second;
	};

	auto getVertexIdx = [oldVertexCount, &getOldTri, &getVertexIdxOnEdge, &newBaryCoords, numPointsPerEdge, &faceVtxToVtxIdx](int oldTriIdx, int row, int col) {
		Eigen::Vector3i tri = getOldTri(oldTriIdx);

		Eigen::Vector2d u0(0.0, 0.0);
		Eigen::Vector2d u1(1.0, 0.0);
		Eigen::Vector2d u2(0.0, 1.0);

		if (row == 0)
		{
			return getVertexIdxOnEdge(oldTriIdx, tri[0], u0, tri[1], u1, col);
		}
		if (col == 0)
		{
			return getVertexIdxOnEdge(oldTriIdx, tri[0], u0, tri[2], u2, row);
		}
		if (col + row == numPointsPerEdge - 1)
		{
			return getVertexIdxOnEdge(oldTriIdx, tri[1], u1, tri[2], u2, row);
		}

		std::pair<int, int> p = std::make_pair(row, col);
		if (faceVtxToVtxIdx[oldTriIdx].find(p) == faceVtxToVtxIdx[oldTriIdx].end())
		{
			int numPointsInRow = numPointsPerEdge - row;

			double lambda = row / double(numPointsPerEdge - 1);
			double lambda2 = col / double(numPointsInRow - 1);

			//Eigen::Vector2d p1 = (1.0 - lambda) * u0 + lambda * u2;
			//Eigen::Vector2d p2 = (1.0 - lambda) * u1 + lambda * u2;

			//Eigen::Vector2d u = (1.0 - lambda2) * p1 + lambda2 * p2;

			Eigen::Vector2d u(lambda2 * (1.0 - lambda), lambda);

			//if (numPointsInRow == 1) u = Eigen::Vector2d(0.0, lambda);

			newBaryCoords.push_back(TriBaryCoords(oldTriIdx, u));

			int newVtxIdx = oldVertexCount + (int)newBaryCoords.size() - 1;
			faceVtxToVtxIdx[oldTriIdx].emplace(p, newVtxIdx);
		}
		return faceVtxToVtxIdx[oldTriIdx].find(p)->second;
	};

	newFaces.resize(3, numTris);

	for (int oldTriIdx = 0; oldTriIdx < oldTriCount; oldTriIdx++)
	{
		int ciTri = 0;
		for (int row = 0; row < numPointsPerEdge - 1; row++)
		{
			//  |
			//  | \           ^
			//  |===\...      |
			//  |\ | \       row
			//  | \|  \
			//  |------\...
			//    col ->
			int numTrisInRow = 2 * numPointsPerEdge - 3 - 2 * row;
			for (int k = 0, baseVertex = 0; k < numTrisInRow; k++, baseVertex++)
			{
				Eigen::Vector3i newTri;
				newTri[0] = getVertexIdx(oldTriIdx, row, baseVertex);
				newTri[1] = getVertexIdx(oldTriIdx, row, baseVertex + 1);
				newTri[2] = getVertexIdx(oldTriIdx, row + 1, baseVertex);
				newFaces.col(oldTriIdx * numTrisPerOldFace + ciTri) = newTri;
				ciTri += 1;

				k++;
				if (k >= numTrisInRow) break;

				newTri[0] = getVertexIdx(oldTriIdx, row, baseVertex + 1);
				newTri[1] = getVertexIdx(oldTriIdx, row + 1, baseVertex + 1);
				newTri[2] = getVertexIdx(oldTriIdx, row + 1, baseVertex);
				newFaces.col(oldTriIdx * numTrisPerOldFace + ciTri) = newTri;
				ciTri += 1;
			}
		}
		assert(ciTri == numTrisPerOldFace);
	}

}
void SimpleTriMesh::upsampleKeep(int nAdditionalPointsPerEdge)
{
	if (nAdditionalPointsPerEdge == 0) return;
	//the following implementation removes non used vertices

	std::vector<TriBaryCoords, Eigen::aligned_allocator<TriBaryCoords> > newBaryCoords;
	Eigen::MatrixXi newFaces;
	upsampleTriMeshKeepPointsAndIndexing(
		m_vertices.size(),
		m_triangles.size(),
		[this](int i)->Eigen::Vector3i { return m_triangles[i]; },
		nAdditionalPointsPerEdge,
		newBaryCoords, newFaces
		);

	for (int i = 0; i < newBaryCoords.size(); i++)
	{
		const Eigen::Vector3i& tri = m_triangles[newBaryCoords[i].getTriIdx()];
		Eigen::Vector3d N = newBaryCoords[i].getN();
		Eigen::Vector3d v = Eigen::Vector3d::Zero();
		for (int j = 0; j < 3; j++)
		{
			v += N[j] * m_vertices[tri[j]];
		}
		
		addVertex(v);
	}

	m_triangles.resize(newFaces.cols());
	for (int i = 0; i < (int)m_triangles.size(); i++)
	{
		m_triangles[i] = newFaces.col(i);
	}

}
void SimpleTriMesh::writeObj(const char * filename) const
{

	std::ofstream filestream(filename);
	for (int i = 0; i < m_vertices.size(); i++)
	{
		filestream << std::setprecision(18);
		filestream << "v " << m_vertices[i][0] << ' ' << m_vertices[i][1] << ' ' << m_vertices[i][2] << '\n';
	}
	for (int i = 0; i < m_triangles.size(); i++)
	{
		filestream << "f " << (m_triangles[i][0] + 1) << ' ' << (m_triangles[i][1] + 1) << ' ' << (m_triangles[i][2] + 1) << '\n';
	}
}

SimpleTriMesh SimpleTriMesh::icosphere()
{
	double t = (1.0 + std::sqrt(5.0)) / 2.0;
	double normalizationFactor = 2.0 / (1.0 + t*t);
	double u = t * normalizationFactor;
	double o = 1.0 * normalizationFactor;
	SimpleTriMesh mesh;

	mesh.addVertex(-o, u, 0);
	mesh.addVertex(o, u, 0);
	mesh.addVertex(-o, -u, 0);
	mesh.addVertex(o, -u, 0);

	mesh.addVertex(0, -o, u);
	mesh.addVertex(0, o, u);
	mesh.addVertex(0, -o, -u);
	mesh.addVertex(0, o, -u);

	mesh.addVertex(u, 0, -o);
	mesh.addVertex(u, 0, o);
	mesh.addVertex(-u, 0, -o);
	mesh.addVertex(-u, 0, o);

	mesh.addTri(0, 5, 1);
	mesh.addTri(0, 11, 5);
	mesh.addTri(0, 1, 7);
	mesh.addTri(0, 10, 11);
	mesh.addTri(0, 7, 10);

	mesh.addTri(3, 9, 4);
	mesh.addTri(3, 4, 2);
	mesh.addTri(3, 2, 6);
	mesh.addTri(3, 6, 8);
	mesh.addTri(3, 8, 9);

	mesh.addTri(1, 5, 9);
	mesh.addTri(7, 1, 8);
	mesh.addTri(10, 7, 6);
	mesh.addTri(5, 11, 4);
	mesh.addTri(11, 10, 2);

	mesh.addTri(4, 9, 5);
	mesh.addTri(9, 8, 1);
	mesh.addTri(6, 2, 10);
	mesh.addTri(2, 4, 11);
	mesh.addTri(8, 6, 7);

	return mesh;
}
SimpleTriMesh SimpleTriMesh::icosphere(int nSubdivisions)
{
	SimpleTriMesh mesh = SimpleTriMesh::icosphere();
	for (int i = 0; i < nSubdivisions; i++)
	{
		mesh.upsample(1);
		for (auto &vertex : mesh.vertices())
		{
			vertex.normalize();
		}
	}
	return mesh;
}