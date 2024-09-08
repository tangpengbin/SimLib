// This source file is free for any kind of use commercial or non by any party.

// Author: Conor Stokes
// Purpose: AABB tree operations

#include "AABBTree.h"
#include <assert.h>
#include <fstream>

namespace SimOpt
{
	unsigned int AABBTree::add_node()
	{
		nodes.push_back(new AABBNode());
		return nodes.size() - 1;
	}

	const AABBNode& AABBTree::get_node_at(unsigned int i) const
	{
		return *nodes[i];
	}
	AABBNode& AABBTree::get_node_at(unsigned int i)
	{
		return *nodes[i];
	}

	AABBTree::AABBTree()
	{

	}

	AABBTree::~AABBTree()
	{
		for (unsigned int i = 0; i < nodes.size(); i++)
		{
			delete nodes[i];
			nodes[i] = nullptr;
		}
		nodes.clear();
		root = NULL;
	}


	void AABBTree::build_node(unsigned int id, const std::vector<collisionElement*>& trilist, int depth, const Heuristic& heurdata)
	{
		AABBNode& nodei = get_node_at(id);
		nodei.init();
		nodei.set_root(this);
		nodei.build(trilist, depth, heurdata);
	}
	void AABBTree::build_tree(const std::vector<collisionElement*>& trilist, int depth, const Heuristic& heurdata)
	{
		unsigned int id = add_node();
		root = &get_node_at(id);
		assert(id == 0); // should really be the first node !
		build_node(id, trilist, depth, heurdata);
	}


	bool AABBTree::write_to_file(const std::string& filename)
	{
		std::fstream outfile(filename.c_str(), std::ios::out | std::ios::binary);

		// Code to write file here
		const unsigned int numNodes = nodes.size();
		outfile.write((char*)&numNodes, sizeof(unsigned int));
		for (unsigned int i = 0; i < numNodes; i++)
		{
			const AABBNode& node = get_node_at(i);
			outfile.write((char*)&node.get_box(), sizeof(AABB));
			outfile.write((char*)&node.get_children(), sizeof(unsigned int) * 2);
			const unsigned int num_triangles = node.num_triangles();
			outfile.write((char*)&num_triangles, sizeof(unsigned int));
			if (num_triangles != 0)
				outfile.write((char*)node.getTriangleAt(0), sizeof(collisionElement) * num_triangles);
		}

		outfile.close();
		return true;
	}

	bool AABBTree::read_from_file(const std::string& filename)
	{
		std::fstream infile(filename.c_str(), std::ios::in | std::ios::binary);
		if (!infile.is_open())
			return false;

		// Code to read file here
		unsigned int numNodes(0);
		infile.read((char*)&numNodes, sizeof(unsigned int));
		nodes.reserve(numNodes);
		for (unsigned int i = 0; i < numNodes; i++)
		{
			add_node();
			AABBNode& node = get_node_at(i);
			node.set_root(this);
			infile.read((char*)&node.get_box(), sizeof(AABB));
			infile.read((char*)&node.get_children(), sizeof(unsigned int) * 2);
			unsigned int num_triangles(0);
			infile.read((char*)&num_triangles, sizeof(unsigned int));
			if (num_triangles != 0)
			{
				node.triangles().resize(num_triangles);
				infile.read((char*)&node.triangles()[0], sizeof(collisionElement) * num_triangles);
			}
		}

		root = &get_node_at(0);

		infile.close();

		return true;
	}

	void AABBTree::best_guess(const std::string& filename, const std::vector<collisionElement*>& trilist, int depth, const Heuristic& heurdata)
	{
		if (!read_from_file(filename.c_str()))
		{
			build_tree(trilist, depth, heurdata);
			write_to_file(filename);
		}
	}
}