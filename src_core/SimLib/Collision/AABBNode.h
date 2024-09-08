#pragma once
#ifndef AABB_NODE_INCLUDED
#define AABB_NODE_INCLUDED

#include "CRay.h"
#include "Sphere.h"
//#include "AABBTree.h"

#include "Heuristic.h"
#include "HitRecord.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace SimOpt
{
	class AABBTree;

	class AABBNode {
	private:
		AABB m_box; // node's bounding box
		unsigned int m_children[2]; // child nodes of this node	
		std::vector<collisionElement*> m_collisionElements; // triangles in this node
		AABBTree* tree;
	public:
		AABBNode();
		~AABBNode();

		void init()
		{
			m_children[0] = -1;
			m_children[1] = -1;
		}
		void set_root(AABBTree* t) { tree = t; }
		void build(const std::vector<collisionElement*>& trilist, int depth, const Heuristic& heurdata);
		
		void CollidingTriangles(Sphere testsphere, std::vector<const collisionElement*>& collideTriangles, Eigen::Vector3d& norm) const;
		bool CollidingTriangles(const CRay& ray, std::vector<const collisionElement*>& collideTriangles, HitRecord& record) const;
		bool CollidingTriangles(const CRay& ray, collisionElement* collideTriangles, HitRecord& record) const;


		bool CollidingTriangles(const CRay& ray, std::vector<const collisionElement*>& collideTriangles, HitRecord& record, const collisionElement* &collidingElement) const;
		
		void findCloestPrimitive(const Eigen::Vector3d& queryPoint, double& currentCloestDistance, const collisionElement* &closestTriangle) const;
		const collisionElement* getTriangleAt(unsigned int i) const { return m_collisionElements[i]; }

		const AABB& get_box() const { return m_box; }
		AABB& get_box() { return m_box; }

		const unsigned int& get_children() const { return m_children[0]; }
		unsigned int& get_children() { return m_children[0]; }

		unsigned int num_triangles() const { return m_collisionElements.size(); }
		std::vector<collisionElement*>& triangles() { return m_collisionElements; }
		const std::vector<collisionElement*>& triangles() const { return m_collisionElements; }

		bool is_a_leaf() const { return m_children[0] == -1; }
	};
}
#endif //AABB_NODE_INCLUDED