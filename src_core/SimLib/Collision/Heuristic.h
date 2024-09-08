#pragma once
#ifndef HEURISTIC_INCLUDED
#define HEURISTIC_INCLUDED
namespace SimOpt
{

	struct Heuristic {
		int maxdepth; // maximum depth
		int mintricnt; // minimum CTriangle count 
		int tartricnt; // target CTriangle count
		float minerror; // minimum error required

		Heuristic(int depth, int minTri, int targetTri, float tolerancee) : maxdepth(depth), mintricnt(minTri), tartricnt(targetTri), minerror(tolerancee) {}
	};
}

#endif //HEURISTIC_INCLUDED