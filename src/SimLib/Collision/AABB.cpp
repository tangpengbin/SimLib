// This source file is free for any kind of use commercial or non by any party.

// Author: Conor Stokes
// Purpose: Simple geometry primitive operations

#include "AABB.h"
#include <assert.h>
#include "CTriangle.h"


namespace SimOpt
{

	// construct an AABB from a list of triangles.
	AABB::AABB(const std::vector<collisionElement*>& triangles)
	{
		// do nothing if no triangles in the list
		if (triangles.empty())
			return;

		std::vector<collisionElement*>::const_iterator triitr;

		// copy aabb of first CTriangle 
		Cpy(triangles.front()->computeAABB());

		// encompass all other triangles in the list
		for (triitr = triangles.begin(); triitr != triangles.end(); triitr++)
			Encompass((*triitr)->computeAABB());
	}

	void AABB::Build(const std::vector<collisionElement*>& triangles)
	{
		// do nothing if no triangles in the list
		if (triangles.empty())
			return;


		// copy aabb of first CTriangle 
		Cpy(triangles.front()->computeAABB());

		// encompass all other triangles in the list
		for (unsigned int i = 0; i < triangles.size(); i++)
			Encompass(triangles[i]->computeAABB());
	}


	// distance squared to a point from the box (arvos algorithm)
	double AABB::DistSqrd(const Eigen::Vector3d& point)  const
	{
		double dst = 0;
		//for each component, find the point's relative position and the distance contribution
		dst += (point.x() < m_bounds[0].x()) ? (point.x() - m_bounds[0].x()) * (point.x() - m_bounds[0].x()) :
			(point.x() > m_bounds[1].x()) ? (point.x() - m_bounds[1].x()) * (point.x() - m_bounds[1].x()) : 0.0f;

		dst += (point.y() < m_bounds[0].y()) ? (point.y() - m_bounds[0].y()) * (point.y() - m_bounds[0].y()) :
			(point.y() > m_bounds[1].y()) ? (point.y() - m_bounds[1].y()) * (point.y() - m_bounds[1].y()) : 0.0f;

		dst += (point.z() < m_bounds[0].z()) ? (point.z() - m_bounds[0].z()) * (point.z() - m_bounds[0].z()) :
			(point.z() > m_bounds[1].z()) ? (point.z() - m_bounds[1].z()) * (point.z() - m_bounds[1].z()) : 0.0f;
		return (double)dst;
	}

	// calculate the floating point error metric 
	double AABB::ErrorMetric() const
	{
		Eigen::Vector3d result = (m_bounds[1] - m_bounds[0]);
		Eigen::Vector3d one;
		double volume = result.x() * result.y() * result.z();
		return double(1 + volume);  // need to check
	}
	// longest axii of the bounding box
	unsigned int AABB::LongestAxis() const
	{
		Eigen::Vector3d boxdim(m_bounds[1] - m_bounds[0]);

		unsigned int la = 0; // longest axis
		double lav = 0.0f; // longest axis length

		// for each dimension  
			// check if its longer
		if (boxdim.x() > lav) {
			// store it if it is
			la = 0;
			lav = (double)boxdim.x();
		}
		if (boxdim.y() > lav) {
			la = 1;
			lav = (double)boxdim.y();
		}
		if (boxdim.z() > lav) {
			la = 2;
			lav = (double)boxdim.z();
		}

		return la;
	}

	// make this box encompass the current box as well as this one
	void AABB::Encompass(const  AABB& encbox)
	{

		auto cvmin = [](const Eigen::Vector3d& _v0, const Eigen::Vector3d& _v1)
		{
			const double a = (_v0.x() < _v1.x()) ? _v0.x() : _v1.x();
			const double b = (_v0.y() < _v1.y()) ? _v0.y() : _v1.y();
			const double c = (_v0.z() < _v1.z()) ? _v0.z() : _v1.z();
			return Eigen::Vector3d(a, b, c);
		};

		auto cvmax = [](const Eigen::Vector3d& _v0, const Eigen::Vector3d& _v1)
		{

			const double a = (_v0.x() > _v1.x()) ? _v0.x() : _v1.x();
			const double b = (_v0.y() > _v1.y()) ? _v0.y() : _v1.y();
			const double c = (_v0.z() > _v1.z()) ? _v0.z() : _v1.z();
			return Eigen::Vector3d(a, b, c);
		};

		m_bounds[0] = cvmin(m_bounds[0], encbox.m_bounds[0]);
		m_bounds[1] = cvmax(m_bounds[1], encbox.m_bounds[1]);
	}


	// render this box
	void AABB::Render() const
	{
		// This render function is very inefficient and should not be called.
		// If it is necessary, use this code to write a function using the viz component
		// Using a unit box, and using the extent of the AABB to scale the box, and its middle position to move the box would be enough
		assert(false);

		Eigen::Vector3d boxcentre((m_bounds[1] - m_bounds[0]) / 2.0);
		Eigen::Vector3d boxdim(m_bounds[1] - m_bounds[0]);
		Eigen::Vector3d p[8];
		// find the eight vertices of the box
		p[0] = Eigen::Vector3d(m_bounds[0]);
		p[1] = Eigen::Vector3d(m_bounds[0].x(), m_bounds[0].y(), m_bounds[1].z());
		p[2] = Eigen::Vector3d(m_bounds[1].x(), m_bounds[0].y(), m_bounds[1].z());
		p[3] = Eigen::Vector3d(m_bounds[1].x(), m_bounds[0].y(), m_bounds[0].z());
		p[4] = Eigen::Vector3d(m_bounds[1].x(), m_bounds[1].y(), m_bounds[0].z());
		p[5] = Eigen::Vector3d(m_bounds[1].x(), m_bounds[1].y(), m_bounds[1].z());
		p[6] = Eigen::Vector3d(m_bounds[0].x(), m_bounds[1].y(), m_bounds[1].z());
		p[7] = Eigen::Vector3d(m_bounds[0].x(), m_bounds[1].y(), m_bounds[0].z());

		/*glColor3f(1.0, 1.0, 1.0);
		glLineWidth(1.0);

		glBegin(GL_LINE_STRIP);
		for(int i =0; i<4; i++)
		  glVertex3d(p[i].x, p[i].y, p[i].z);
		glVertex3d(p[0].x, p[0].y, p[0].z);
		glEnd();

		glBegin(GL_LINE_STRIP);
		  for(unsigned int i =4; i<8; i++)
			glVertex3d(p[i].x, p[i].y, p[i].z);
		  glVertex3d(p[4].x, p[4].y, p[4].z);
		glEnd();

		glBegin(GL_LINES);
			glVertex3d(p[0].x, p[0].y, p[0].z);
			glVertex3d(p[7].x, p[7].y, p[7].z);
		glEnd();

		glBegin(GL_LINES);
			glVertex3d(p[1].x, p[1].y, p[1].z);
			glVertex3d(p[6].x, p[6].y, p[6].z);
		glEnd();

		glBegin(GL_LINES);
			glVertex3d(p[2].x, p[2].y, p[2].z);
			glVertex3d(p[5].x, p[5].y, p[5].z);
		glEnd();

		glBegin(GL_LINES);
			glVertex3d(p[3].x, p[3].y, p[3].z);
			glVertex3d(p[4].x, p[4].y, p[4].z);
		glEnd();*/


	}

}