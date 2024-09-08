#include "AABBNode.h"
#include "AABBTree.h"

namespace SimOpt
{
	AABBNode::AABBNode()
	{

	}

	AABBNode::~AABBNode()
	{
		for (int i = 0; i < m_collisionElements.size(); i++)
		{
			delete m_collisionElements[i];
			m_collisionElements[i] = NULL;
		}
		m_collisionElements.clear();
		tree = NULL;
	}

	void AABBNode::build(const std::vector<collisionElement*>& trilist, int depth, const Heuristic& heurdata)
	{
		m_box.Build(trilist);
		// test our build heuristic - if passes, make children
		if (depth < heurdata.maxdepth && (int)trilist.size() > heurdata.mintricnt
			&& ((int)trilist.size() > heurdata.tartricnt || m_box.ErrorMetric() > heurdata.minerror))
		{
			std::vector<collisionElement*>::const_iterator triitr; // iterator for looping through trilist 
			std::vector<collisionElement*> tribuckets[2]; // buckets of triangles, two lists for left anf right nodes
			int longaxis = m_box.LongestAxis(); // longest axis

			Eigen::Vector3d geoavg; // geometric average - midpoint of ALL the triangles
			geoavg.setZero();

			// go through all tris and calculate the average of the midpoints
			double m = double(1.) / trilist.size();
			for (triitr = trilist.begin(); triitr != trilist.end(); triitr++)
				geoavg += (*triitr)->MidPoint() * m;

			// bucket tris based on their midpoint's side of the geo average in the longest axis
			for (triitr = trilist.begin(); triitr != trilist.end(); triitr++)
			{
				switch (longaxis)
				{
				case 0: {
					if (geoavg.x() > (*triitr)->MidPoint().x())
						tribuckets[1].push_back(*triitr);
					else
						tribuckets[0].push_back(*triitr);
				}break;
				case 1: {
					if (geoavg.y() > (*triitr)->MidPoint().y())
						tribuckets[1].push_back(*triitr);
					else
						tribuckets[0].push_back(*triitr);
				}break;
				case 2: {
					if (geoavg.z() > (*triitr)->MidPoint().z())
						tribuckets[1].push_back(*triitr);
					else
						tribuckets[0].push_back(*triitr);
				}break;

				}
			}
			// create new children using the buckets
			m_children[0] = tree->add_node();
			m_children[1] = tree->add_node();
			tree->build_node(m_children[0], tribuckets[0], depth + 1, heurdata);
			tree->build_node(m_children[1], tribuckets[1], depth + 1, heurdata);

		}
		else { // otherwise the build heuristic failed, this is
		 // set the first child to null (identifies a leaf)
			m_children[0] = -1;
			m_children[1] = -1;
			// copy collisionElement list
			m_collisionElements = trilist;
		}
	}

	void AABBNode::CollidingTriangles(Sphere testsphere, std::vector<const collisionElement*>& collideTriangles, Eigen::Vector3d& norm) const
	{
		// if no intersection with the node box, don't continue
		if (testsphere.Intersects(m_box))
		{
			if (!is_a_leaf()) // is not a leaf
			{
				// recurse to children
				tree->get_node_at(m_children[0]).CollidingTriangles(testsphere, collideTriangles, norm);
				tree->get_node_at(m_children[1]).CollidingTriangles(testsphere, collideTriangles, norm);

			}
			else // is a leaf
			{
				for (unsigned int i = 0; i < m_collisionElements.size(); i++)
				{
					if (testsphere.Intersects(*m_collisionElements[i]))
					{
						collideTriangles.push_back(m_collisionElements[i]);
						if(dynamic_cast<const CTriangle*>(m_collisionElements[i]) != nullptr)
							norm += dynamic_cast<const CTriangle*>(m_collisionElements[i])->getTriangleNormal();
					}
				}

			}
		}
	}


	bool AABBNode::CollidingTriangles(const CRay& ray, std::vector<const collisionElement*>& collideTriangles, HitRecord& record) const
	{
		bool result = false;
		
		if (ray.Intersects(m_box.getM_bounds()))
		{
			if (!is_a_leaf()) // is not a leaf
			{
				// recurse to children
				result |= tree->get_node_at(m_children[0]).CollidingTriangles(ray, collideTriangles, record);
				result |= tree->get_node_at(m_children[1]).CollidingTriangles(ray, collideTriangles, record);
			}
			else // is a leaf
			{
				for (unsigned int i = 0; i < m_collisionElements.size(); i++)
				{
					Eigen::Vector3d at;
					double t;
					if (ray.Intersects(*m_collisionElements[i], &at, &t))
					{
						collideTriangles.push_back(m_collisionElements[i]);
						result = true;
						if (t < record.t)
						{
							if (dynamic_cast<const CTriangle*>(m_collisionElements[i]) != nullptr)
								record.normal = dynamic_cast<const CTriangle*>(m_collisionElements[i])->getTriangleNormal();
							record.position = at;
							record.t = t;
						}
					}
				}
			}
		}
		
		return result;
	}


	bool AABBNode::CollidingTriangles(const CRay& ray, collisionElement* collideTriangle, HitRecord& record) const
	{
		bool result = false;
		if (ray.Intersects(m_box.getM_bounds()))
		{
			if (!is_a_leaf()) // is not a leaf
			{
				// recurse to children
				result |= tree->get_node_at(m_children[0]).CollidingTriangles(ray, collideTriangle, record);
				result |= tree->get_node_at(m_children[1]).CollidingTriangles(ray, collideTriangle, record);
			}
			else // is a leaf
			{
				for (unsigned int i = 0; i < m_collisionElements.size(); i++)
				{
					Eigen::Vector3d at;
					double t;
					if (ray.Intersects(*m_collisionElements[i], &at, &t))
					{
						result = true;
						if (t < record.t)
						{
							if (dynamic_cast<const CTriangle*>(m_collisionElements[i]) != nullptr)
								record.normal = dynamic_cast<const CTriangle*>(m_collisionElements[i])->getTriangleNormal();
							record.position = at;
							record.t = t;
							collideTriangle = m_collisionElements[i];
						}
					}
				}
			}
		}
		return result;
	}


	bool AABBNode::CollidingTriangles(const CRay& ray, std::vector<const collisionElement*>& collideTriangles, HitRecord& record, const collisionElement* &collidingElement) const
	{
		bool result = false;

		if (ray.Intersects(m_box.getM_bounds()))
		{
			if (!is_a_leaf()) // is not a leaf
			{
				// recurse to children
				result |= tree->get_node_at(m_children[0]).CollidingTriangles(ray, collideTriangles, record, collidingElement);
				result |= tree->get_node_at(m_children[1]).CollidingTriangles(ray, collideTriangles, record, collidingElement);
			}
			else // is a leaf
			{
				for (unsigned int i = 0; i < m_collisionElements.size(); i++)
				{
					Eigen::Vector3d at;
					double t;
					if (ray.Intersects(*m_collisionElements[i], &at, &t))
					{
						collideTriangles.push_back(m_collisionElements[i]);
						result = true;
						if (t < record.t)
						{
							if (dynamic_cast<const CTriangle*>(m_collisionElements[i]) != nullptr)
								record.normal = dynamic_cast<const CTriangle*>(m_collisionElements[i])->getTriangleNormal();
							record.position = at;
							record.t = t;
							collidingElement = m_collisionElements[i];
						}
					}
				}
			}
		}

		return result;
	}

	void AABBNode::findCloestPrimitive(const Eigen::Vector3d& queryPoint, double &currentCloestDistance, const collisionElement* &closestTriangle) const
	{
		double box_distance_squard = m_box.DistSqrd(queryPoint);
		if (box_distance_squard > currentCloestDistance)
			return;//if this box is far than the current cloest distance, the closest primitive will not in this aabb

		//find cloest triangle(maybe edge or vertices) and return distance squared
		if (!is_a_leaf()) // is not a leaf
		{
			// recurse to children
			double child0_dist = tree->get_node_at(m_children[0]).get_box().DistSqrd(queryPoint);
			double child1_dist = tree->get_node_at(m_children[1]).get_box().DistSqrd(queryPoint);

			//we query cloest child first
			if (child0_dist <= child1_dist)
			{
				tree->get_node_at(m_children[0]).findCloestPrimitive(queryPoint, currentCloestDistance, closestTriangle);
				tree->get_node_at(m_children[1]).findCloestPrimitive(queryPoint, currentCloestDistance, closestTriangle);
			}
			else
			{
				tree->get_node_at(m_children[1]).findCloestPrimitive(queryPoint, currentCloestDistance, closestTriangle);
				tree->get_node_at(m_children[0]).findCloestPrimitive(queryPoint, currentCloestDistance, closestTriangle);
			}
		}
		else // is a leaf
		{
			//double distance_squared = std::numeric_limits<double>::infinity();
			for (unsigned int i = 0; i < m_collisionElements.size(); i++)
			{
				double tri_i_distance_squared = m_collisionElements[i]->DistSqrd(queryPoint);
				if (tri_i_distance_squared < currentCloestDistance)
				{
					currentCloestDistance = tri_i_distance_squared;
					closestTriangle = m_collisionElements[i];
				}
			}
		}
	}
}
