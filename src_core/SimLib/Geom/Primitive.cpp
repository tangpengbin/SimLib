#include "Primitive.h"

#include <Eigen/Geometry>

static Eigen::Vector3d parallelTransportVector(
	const Eigen::Vector3d &m_vFrom,
	const Eigen::Vector3d &m_vTo,
	const Eigen::Vector3d &x)
{
	typedef double ScalarType;
	return x * (m_vFrom.dot(m_vTo)) + (m_vFrom.cross(m_vTo)).cross(x) + (m_vFrom.cross(m_vTo)) *((m_vFrom.cross(m_vTo)).dot(x) / (ScalarType(1.0) + m_vFrom.dot(m_vTo)));
}

void generalizedCylinder(
	const std::vector<Eigen::Vector3d> &centerlinePositions,
	const std::vector<double> &radii,
	int axis_devisions,
	Eigen::MatrixXd & V,
	Eigen::MatrixXi & F)
{
	typedef double Scalar;

	int height_devisions = centerlinePositions.size();
	V.resize(3, 2 + axis_devisions * height_devisions);
	F.resize(3, 2 * axis_devisions + 2 * (axis_devisions*(height_devisions - 1)));
	int f = 0;
	Eigen::Vector3d oldNormal;
	Eigen::Vector3d oldTangent;
	for (int h = 0; h < height_devisions; h++)
	{
		Eigen::Vector3d tangent;
		Eigen::Vector3d normal;
		if (h == 0)
		{
			tangent = (centerlinePositions[1] - centerlinePositions[0]).normalized();
			normal = tangent.unitOrthogonal();
		}
		else if (h != height_devisions - 1)
		{
			Eigen::Vector3d t1 = (centerlinePositions[h] - centerlinePositions[h - 1]).normalized();
			Eigen::Vector3d t2 = (centerlinePositions[h + 1] - centerlinePositions[h]).normalized();
			tangent = (t1 + t2).normalized();
			normal = parallelTransportVector(oldTangent, tangent, oldNormal);
			normal.normalize();
		}
		else
		{
			tangent = (centerlinePositions[h] - centerlinePositions[h - 1]).normalized();
			normal = parallelTransportVector(oldTangent, tangent, oldNormal);
		}
		double radius = radii[h];
		for (int th = 0; th < axis_devisions; th++)
		{
			double angle = 2.0 *3.14159265358979323846 * th / axis_devisions;
			Eigen::AngleAxisd rotation(angle, tangent);
			Eigen::Vector3d p = centerlinePositions[h] + radius * (rotation * normal);
			V(0, th + h * axis_devisions) = p[0];
			V(1, th + h * axis_devisions) = p[1];
			V(2, th + h * axis_devisions) = p[2];
			if (h > 0)
			{
				F(0, f) = ((th + 0) % axis_devisions) + (h - 1)*axis_devisions;
				F(1, f) = ((th + 1) % axis_devisions) + (h - 1)*axis_devisions;
				F(2, f) = ((th + 0) % axis_devisions) + (h + 0)*axis_devisions;
				f++;
				F(0, f) = ((th + 1) % axis_devisions) + (h - 1)*axis_devisions;
				F(1, f) = ((th + 1) % axis_devisions) + (h + 0)*axis_devisions;
				F(2, f) = ((th + 0) % axis_devisions) + (h + 0)*axis_devisions;
				f++;
			}
		}
		oldNormal = normal;
		oldTangent = tangent;
	}

	int bottomCenterVertex = height_devisions * axis_devisions;
	V(0, bottomCenterVertex) = centerlinePositions[0][0];
	V(1, bottomCenterVertex) = centerlinePositions[0][1];
	V(2, bottomCenterVertex) = centerlinePositions[0][2];
	int topCenterVertex = height_devisions * axis_devisions + 1;
	V(0, topCenterVertex) = centerlinePositions.back()[0];
	V(1, topCenterVertex) = centerlinePositions.back()[1];
	V(2, topCenterVertex) = centerlinePositions.back()[2];

	for (int th = 0; th < axis_devisions; th++)
	{
		//first cap
		F(0, f) = ((th + 1) % axis_devisions);
		F(1, f) = ((th + 0) % axis_devisions);
		F(2, f) = bottomCenterVertex;
		f++;

		//second cap
		int h = height_devisions - 1;
		F(0, f) = ((th + 0) % axis_devisions) + h * axis_devisions;
		F(1, f) = ((th + 1) % axis_devisions) + h * axis_devisions;
		F(2, f) = topCenterVertex;
		f++;
	}

	assert(f == F.cols());
}

void generalizedCylinder(
	const std::vector<Eigen::Vector3d> &centerlinePositions,
	const std::vector<Eigen::Vector3d> &nA,
	const std::vector<Eigen::Vector3d> &nB,
	int axis_devisions,
	Eigen::MatrixXd & V,
	Eigen::MatrixXi & F)
{
	typedef double Scalar;

	int height_devisions = centerlinePositions.size();
	V.resize(3, 2 + axis_devisions * height_devisions);
	F.resize(3, 2 * axis_devisions + 2 * (axis_devisions*(height_devisions - 1)));
	int f = 0;
	for (int h = 0; h < height_devisions; h++)
	{
		for (int th = 0; th < axis_devisions; th++)
		{
			double angle = 2.0 *3.14159265358979323846 * th / axis_devisions;
			Eigen::Vector3d p = centerlinePositions[h] + std::cos(angle) * nA[h]  + std::sin(angle) * nB[h];
			V(0, th + h * axis_devisions) = p[0];
			V(1, th + h * axis_devisions) = p[1];
			V(2, th + h * axis_devisions) = p[2];
			if (h > 0)
			{
				F(0, f) = ((th + 0) % axis_devisions) + (h - 1)*axis_devisions;
				F(1, f) = ((th + 1) % axis_devisions) + (h - 1)*axis_devisions;
				F(2, f) = ((th + 0) % axis_devisions) + (h + 0)*axis_devisions;
				f++;
				F(0, f) = ((th + 1) % axis_devisions) + (h - 1)*axis_devisions;
				F(1, f) = ((th + 1) % axis_devisions) + (h + 0)*axis_devisions;
				F(2, f) = ((th + 0) % axis_devisions) + (h + 0)*axis_devisions;
				f++;
			}
		}
	}

	int bottomCenterVertex = height_devisions * axis_devisions;
	V(0, bottomCenterVertex) = centerlinePositions[0][0];
	V(1, bottomCenterVertex) = centerlinePositions[0][1];
	V(2, bottomCenterVertex) = centerlinePositions[0][2];
	int topCenterVertex = height_devisions * axis_devisions + 1;
	V(0, topCenterVertex) = centerlinePositions.back()[0];
	V(1, topCenterVertex) = centerlinePositions.back()[1];
	V(2, topCenterVertex) = centerlinePositions.back()[2];

	for (int th = 0; th < axis_devisions; th++)
	{
		//first cap
		F(0, f) = ((th + 1) % axis_devisions);
		F(1, f) = ((th + 0) % axis_devisions);
		F(2, f) = bottomCenterVertex;
		f++;

		//second cap
		int h = height_devisions - 1;
		F(0, f) = ((th + 0) % axis_devisions) + h * axis_devisions;
		F(1, f) = ((th + 1) % axis_devisions) + h * axis_devisions;
		F(2, f) = topCenterVertex;
		f++;
	}

	assert(f == F.cols());
}


void disk(double s0, double s1, double t0, double t1,
	int numSamples,
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F)
{
	//we use the implementation in the source file https://nmd.web.illinois.edu//quadrics/hypparab.html
	int n = numSamples;

	double ds = (s1 - s0) / n;
	double dt = (t1 - t0) / n;

	auto phi = [](double r, double t) 
	{
		double x = r * cos(t);
		double y = r * sin(t);
		double z = 0;
		return Eigen::Vector3d(x, y, z);
	};

	// Evaluate the parameterization at the sample points to find
	// the vertices.
	V = Eigen::MatrixXd(3, 1 + n * n);
	V.col(0) = Eigen::Vector3d(0, 0, 0);//origin
	for (int i = 1; i <= n; i++) //axial direction
	{
		for (int j = 0; j < n; j++) //radial direction
		{
			double s = s0 + i * ds;
			double t = t0 + j * dt;

			double z = 0.0;
			V.col(1 + (i - 1) * n + j) = phi(s, t);
		}
	}

	F = Eigen::MatrixXi(3, n + 2 * (n-1) * n);

	// Now add in the triangles.
	for (int i = 0; i < n; i++)
	{
		F.col(i) = Eigen::Vector3i(0, i + 2, i + 1);
	}
	for (int i = 0; i < n; i++)
	{
		for (int j = 1; j < n; j++)
		{
			int k = 1 + n * (j - 1) + i;
			if (i != n - 1)
			{
				F.col(n + (i * (n - 1) + (j - 1)) * 2 + 0) = Eigen::Vector3i(k, k + 1, k + n + 1);
				F.col(n + (i * (n - 1) + (j - 1)) * 2 + 1) = Eigen::Vector3i(k, k + n + 1, k + n);
			}
			else
			{
				F.col(n + (i * (n - 1) + (j - 1)) * 2 + 0) = Eigen::Vector3i(k, k - (n - 1), k + 1);
				F.col(n + (i * (n - 1) + (j - 1)) * 2 + 1) = Eigen::Vector3i(k, k + 1, k + n);
			}
		}
	}
}


void cuboid(double length, double width, double height, bool isolatedFaces,
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F)
{
	//x in length
	//z in width
	//y in height

	V.resize(3, 8);
	V.col(0) = Eigen::Vector3d(-1, -1, 1);
	V.col(1) = Eigen::Vector3d(1, -1, 1);
	V.col(2) = Eigen::Vector3d(-1, 1, 1);
	V.col(3) = Eigen::Vector3d(1, 1, 1);
	V.col(4) = Eigen::Vector3d(-1, 1, -1);
	V.col(5) = Eigen::Vector3d(1, 1, -1);
	V.col(6) = Eigen::Vector3d(-1, -1, -1);
	V.col(7) = Eigen::Vector3d(1, -1, -1);

	F.resize(3, 12);
	F.col(0) = Eigen::Vector3i(0, 1, 2);
	F.col(1) = Eigen::Vector3i(1, 3, 2);
	F.col(2) = Eigen::Vector3i(2, 3, 4);
	F.col(3) = Eigen::Vector3i(3, 5, 4);
	F.col(4) = Eigen::Vector3i(4, 5, 6);
	F.col(5) = Eigen::Vector3i(5, 7, 6);
	F.col(6) = Eigen::Vector3i(0, 6, 7);
	F.col(7) = Eigen::Vector3i(0, 7, 1);
	F.col(8) = Eigen::Vector3i(1, 7, 3);
	F.col(9) = Eigen::Vector3i(3, 7, 5);
	F.col(10) = Eigen::Vector3i(0, 4, 6);
	F.col(11) = Eigen::Vector3i(0, 2, 4);
	if (isolatedFaces)
	{

		Eigen::MatrixXd isolatedVertices;
		isolatedVertices.resize(3, 36);

		Eigen::MatrixXi isolatedFaces;
		isolatedFaces.resize(3, 12);

		for (int f_i = 0; f_i < F.cols(); f_i++)
		{
			for (int v_i = 0; v_i < 3; v_i++)
			{
				int vertexIndex = F(v_i, f_i);
				isolatedVertices.col(f_i * 3 + v_i) = V.col(vertexIndex);
				isolatedFaces(v_i, f_i) = f_i * 3 + v_i;
			}
		}
		V = isolatedVertices;
		F = isolatedFaces;
	}

	Eigen::Matrix3d scale;
	scale.setZero();
	scale(0, 0) = length;
	scale(1, 1) = height;
	scale(2, 2) = width;
	V = scale * V;
}