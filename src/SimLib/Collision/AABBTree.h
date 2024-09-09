// This header file is free for any kind of use commercial or non by any party.

// Author: Conor Stokes
// Purpose: Simple AABB tree implementation

#pragma once
#ifndef _aabbtree_H
#define _aabbtree_H

//#include "AABB.h"
//#include "AABBNode.h"
#include "Heuristic.h"
#include "HitRecord.h"

#include "CTriangle.h"
#include "AABBNode.h"
namespace SimOpt
{
	class AABBTree {
	private:
		AABBNode* root;
		std::vector<AABBNode*> nodes;
	public:

		void build_tree(const std::vector<collisionElement*>& trilist, int depth, const Heuristic& heurdata);

		unsigned int add_node();
		void build_node(unsigned int id, const std::vector<collisionElement*>& trilist, int depth, const Heuristic& heurdata);

		const AABBNode& get_node_at(unsigned int i) const;
		AABBNode& get_node_at(unsigned int i);

		const AABBNode& get_root() const { return *root; }
		AABBNode& get_root() { return *root; }


		bool write_to_file(const std::string& filename);
		// Returns true if succeeded, if it fails, aabbtree has to be built!
		bool read_from_file(const std::string& filename);

		void best_guess(const std::string& filename, const std::vector<collisionElement*>& trilist, int depth, const Heuristic& heurdata);

		AABBTree();
		~AABBTree();

	};
}
#endif