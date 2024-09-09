#pragma once
#ifndef SIM_OPT_PRIMITIVE_H
#define SIM_OPT_PRIMITIVE_H

#include "Mesh.h"
#include <Eigen/Core>
#include <vector>

template <typename DerivedV, typename DerivedF>
void cylinderNoCaps(
	const int axis_devisions,
	const int height_devisions,
	Eigen::PlainObjectBase<DerivedV> & V,
	Eigen::PlainObjectBase<DerivedF> & F)
{
	V.resize(3, axis_devisions*height_devisions);
	F.resize(3, 2 * (axis_devisions*(height_devisions - 1)));
	int f = 0;
	typedef typename DerivedV::Scalar Scalar;
	for (int th = 0; th<axis_devisions; th++)
	{
		Scalar x = cos(2.*M_PI*Scalar(th) / Scalar(axis_devisions));
		Scalar y = sin(2.*M_PI*Scalar(th) / Scalar(axis_devisions));
		for (int h = 0; h<height_devisions; h++)
		{
			Scalar z = Scalar(h) / Scalar(height_devisions - 1);
			V(0, th + h*axis_devisions) = x;
			V(1, th + h*axis_devisions) = y;
			V(2, th + h*axis_devisions) = z;
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
	assert(f == F.cols());
}

template <typename DerivedV, typename DerivedF>
void cylinder(
	const int axis_devisions,
	const int height_devisions,
	Eigen::PlainObjectBase<DerivedV> & V,
	Eigen::PlainObjectBase<DerivedF> & F)
{
	V.resize(3, 2 + axis_devisions*height_devisions);
	F.resize(3, 2 * axis_devisions + 2 * (axis_devisions*(height_devisions - 1)));
	int f = 0;
	typedef typename DerivedV::Scalar Scalar;
	for (int th = 0; th<axis_devisions; th++)
	{
		Scalar x = cos(2.*M_PI*Scalar(th) / Scalar(axis_devisions));
		Scalar y = sin(2.*M_PI*Scalar(th) / Scalar(axis_devisions));
		for (int h = 0; h<height_devisions; h++)
		{
			Scalar z = Scalar(h) / Scalar(height_devisions - 1);
			V(0, th + h*axis_devisions) = x;
			V(1, th + h*axis_devisions) = y;
			V(2, th + h*axis_devisions) = z;
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

	int bottomCenterVertex = height_devisions*axis_devisions;
	V(0, bottomCenterVertex) = 0.0;
	V(1, bottomCenterVertex) = 0.0;
	V(2, bottomCenterVertex) = 0.0;
	int topCenterVertex = height_devisions*axis_devisions + 1;
	V(0, topCenterVertex) = 0.0;
	V(1, topCenterVertex) = 0.0;
	V(2, topCenterVertex) = 1.0;

	for (int th = 0; th < axis_devisions; th++)
	{
		//first cap
		F(0, f) = ((th + 1) % axis_devisions);
		F(1, f) = ((th + 0) % axis_devisions);
		F(2, f) = bottomCenterVertex;
		f++;

		//second cap
		int h = height_devisions - 1;
		F(0, f) = ((th + 0) % axis_devisions) + h*axis_devisions;
		F(1, f) = ((th + 1) % axis_devisions) + h*axis_devisions;
		F(2, f) = topCenterVertex;
		f++;
	}

	assert(f == F.cols());
}


template <typename DerivedV, typename DerivedF>
void cylinderSeperatedCaps(
	const int axis_devisions,
	const int height_devisions,
	Eigen::PlainObjectBase<DerivedV> & V,
	Eigen::PlainObjectBase<DerivedF> & F)
{
	V.resize(3, 2 + 2 * axis_devisions + axis_devisions*height_devisions);
	F.resize(3, 2 * axis_devisions + 2 * (axis_devisions*(height_devisions - 1)));
	int f = 0;
	typedef typename DerivedV::Scalar Scalar;
	for (int th = 0; th<axis_devisions; th++)
	{
		Scalar x = cos(2.*M_PI*Scalar(th) / Scalar(axis_devisions));
		Scalar y = sin(2.*M_PI*Scalar(th) / Scalar(axis_devisions));
		for (int h = 0; h<height_devisions; h++)
		{
			Scalar z = Scalar(h) / Scalar(height_devisions - 1);
			V(0, th + h*axis_devisions) = x;
			V(1, th + h*axis_devisions) = y;
			V(2, th + h*axis_devisions) = z;
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

	int bottomCenterVertex = height_devisions*axis_devisions;
	V(0, bottomCenterVertex) = 0.0;
	V(1, bottomCenterVertex) = 0.0;
	V(2, bottomCenterVertex) = 0.0;
	int topCenterVertex = height_devisions*axis_devisions + 1;
	V(0, topCenterVertex) = 0.0;
	V(1, topCenterVertex) = 0.0;
	V(2, topCenterVertex) = 1.0;

	int capVerticesOffset = height_devisions*axis_devisions + 2;

	//add seperate cap vertices (i.e. to reduce normal based lighting problems)
	for (int th = 0; th < axis_devisions; th++)
	{
		Scalar x = cos(2.*M_PI*Scalar(th) / Scalar(axis_devisions));
		Scalar y = sin(2.*M_PI*Scalar(th) / Scalar(axis_devisions));

		Scalar z = Scalar(0);
		V(0, 2 * th + capVerticesOffset) = x;
		V(1, 2 * th + capVerticesOffset) = y;
		V(2, 2 * th + capVerticesOffset) = z;

		z = Scalar(1);
		V(0, 2 * th + 1 + capVerticesOffset) = x;
		V(1, 2 * th + 1 + capVerticesOffset) = y;
		V(2, 2 * th + 1 + capVerticesOffset) = z;
	}

	for (int th = 0; th < axis_devisions; th++)
	{
		//first cap
		F(0, f) = 2 * ((th + 1) % axis_devisions) + capVerticesOffset;
		F(1, f) = 2 * ((th + 0) % axis_devisions) + capVerticesOffset;
		F(2, f) = bottomCenterVertex;
		f++;

		//second cap
		F(0, f) = 2 * ((th + 0) % axis_devisions) + 1 + capVerticesOffset;
		F(1, f) = 2 * ((th + 1) % axis_devisions) + 1 + capVerticesOffset;
		F(2, f) = topCenterVertex;
		f++;
	}

	assert(f == F.cols());
}

template <typename DerivedV, typename DerivedF>
void coneSeperatedCap(
	const int axis_devisions,
	Eigen::PlainObjectBase<DerivedV> & V,
	Eigen::PlainObjectBase<DerivedF> & F)
{
	typedef typename DerivedV::Scalar Scalar;

	V.resize(3, 2 + 2 * axis_devisions);
	F.resize(3, 2 * axis_devisions);

	int bottomCenterVertex = 0;
	V(0, bottomCenterVertex) = 0.0;
	V(1, bottomCenterVertex) = 0.0;
	V(2, bottomCenterVertex) = 0.0;
	int topCenterVertex = 1;
	V(0, topCenterVertex) = 0.0;
	V(1, topCenterVertex) = 0.0;
	V(2, topCenterVertex) = 1.0;

	int capVerticesOffset = 2;

	//add seperate cap vertices (i.e. to reduce normal based lighting problems)
	for (int th = 0; th < axis_devisions; th++)
	{
		Scalar x = cos(2.*M_PI*Scalar(th) / Scalar(axis_devisions));
		Scalar y = sin(2.*M_PI*Scalar(th) / Scalar(axis_devisions));

		Scalar z = Scalar(0);
		V(0, 2 * th + capVerticesOffset) = x;
		V(1, 2 * th + capVerticesOffset) = y;
		V(2, 2 * th + capVerticesOffset) = z;

		z = Scalar(0);
		V(0, 2 * th + 1 + capVerticesOffset) = x;
		V(1, 2 * th + 1 + capVerticesOffset) = y;
		V(2, 2 * th + 1 + capVerticesOffset) = z;
	}

	int f = 0;
	for (int th = 0; th < axis_devisions; th++)
	{
		//first cap
		F(0, f) = 2 * ((th + 1) % axis_devisions) + capVerticesOffset;
		F(1, f) = 2 * ((th + 0) % axis_devisions) + capVerticesOffset;
		F(2, f) = bottomCenterVertex;
		f++;

		//second cap
		F(0, f) = 2 * ((th + 0) % axis_devisions) + 1 + capVerticesOffset;
		F(1, f) = 2 * ((th + 1) % axis_devisions) + 1 + capVerticesOffset;
		F(2, f) = topCenterVertex;
		f++;
	}

	assert(f == F.cols());
}

template <typename DerivedV, typename DerivedF>
void loadFromMesh(
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

void generalizedCylinder(
	const std::vector<Eigen::Vector3d> &centerlinePositions,
	const std::vector<double> &radii,
	int axis_devisions,
	Eigen::MatrixXd & V,
	Eigen::MatrixXi & F);

void generalizedCylinder(
	const std::vector<Eigen::Vector3d> &centerlinePositions,
	const std::vector<Eigen::Vector3d> &nA,
	const std::vector<Eigen::Vector3d> &nB,
	int axis_devisions,
	Eigen::MatrixXd & V,
	Eigen::MatrixXi & F);

void disk(double s0, double s1, double t0, double t1,
	int numSamples,
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F);


void cuboid(double length, double width, double height, bool isolatedFaces,
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F);
#endif