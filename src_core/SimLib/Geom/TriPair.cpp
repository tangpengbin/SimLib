#include "TriPair.h"

#include <stdexcept>

namespace SimOpt
{
TriKeiborhoods::TriKeiborhoods()
{

}
TriKeiborhoods::TriKeiborhoods(
	std::vector<TriKeiborhood> keiborhoods
)
	: m_triKeiborhoodsAll(std::move(keiborhoods))
{
	for (auto &keiborhood : m_triKeiborhoodsAll)
	{
		switch (keiborhood.vtcs.size())
		{
		case 4: {
			std::array<int, 4> a;
			std::copy(keiborhood.vtcs.begin(), keiborhood.vtcs.end(), a.begin());
			m_triKeiborhoods4.push_back(a);
			break;
		}
		case 5: {
			std::array<int, 5> a;
			std::copy(keiborhood.vtcs.begin(), keiborhood.vtcs.end(), a.begin());
			m_triKeiborhoods5.push_back(a);
			break;
		}
		case 6: {
			std::array<int, 6> a;
			std::copy(keiborhood.vtcs.begin(), keiborhood.vtcs.end(), a.begin());
			m_triKeiborhoods6.push_back(a);
			break;
		}
		default:  throw std::logic_error("not implemented");
		}
	}
}
TriKeiborhoods computeTriKeiborhoods(
	const std::vector<std::array<int, 3> > &tris,
	const std::vector<TriPair> &triPairs
	)
{
	std::vector<TriKeiborhood> keiborhoods;

	int numTris = (int)tris.size();

	//we first compute for every triangle in which tri pairs it is, then we just have to construct the keiborhoods correctly
	std::vector<std::vector<int> > trianglePairIdcs(numTris);
	for (int triPairIdx = 0; triPairIdx < triPairs.size(); triPairIdx++)
	{
		auto &triPair = triPairs[triPairIdx];
		for (int i = 0; i < triPair.tris.size(); i++)
		{
			int triIdx = triPair.tris[i];
			trianglePairIdcs[triIdx].push_back(triPairIdx);
		}
	}

	//check that we only have keiborhoods with 1, 2 or 3 neighborhood triangles
	for (int triIdx = 0; triIdx < numTris; triIdx++)
	{
		int nTriPairCount = trianglePairIdcs[triIdx].size();
		if (nTriPairCount != 1 && nTriPairCount != 2 && nTriPairCount != 3) throw std::invalid_argument("not supported mesh");
	}

//we handle for now only cases where every triangle either has 2 or 3 neighborhood triangles!
//
// case 1
// 
// x2
// | \
// x0--x1
// \   |
//  \  |
//    x3
//
// case 2: 
//
// x2--x4
// | \ |
// x0--x1
// \   |
//  \  |
//    x3
// case 3 is standard case
// x5--x2--x4
//  \  | \ |
//    x0--x1
//     \   |
//      \  |
//        x3

	keiborhoods.resize(numTris);
	for (int triIdx = 0; triIdx < numTris; triIdx++)
	{
		const std::array<int, 3> &tri = tris[triIdx];
		const std::vector<int> &trianglePairIdcsI = trianglePairIdcs[triIdx];
		int nTriPairCount = trianglePairIdcs[triIdx].size();

		if (nTriPairCount == 1) {
			int kd = 3;
		}

		std::array<int, 6> xIdcs;
		xIdcs[0] = tri[0];
		xIdcs[1] = tri[1];
		xIdcs[2] = tri[2];
		xIdcs[3] = -1;
		xIdcs[4] = -1;
		xIdcs[5] = -1;
		// we first always collect 6 idcs (we assume that we only have manifold edges)
		// and then we rotate/permutate the points in the case of ntriCount < 3 s.t. the last entries in xIdcs are -1

		auto getIdx = [&trianglePairIdcsI, &triPairs](int ev1, int ev2, int tri3) {
			for (int tpIdx : trianglePairIdcsI)
			{
				const TriPair &triPair = triPairs[tpIdx];
				if (ev1 == triPair.vtcs[1] && ev2 == triPair.vtcs[2]
					|| ev2 == triPair.vtcs[1] && ev1 == triPair.vtcs[2])
				{
					if (triPair.vtcs[0] == tri3)
					{
						return triPair.vtcs[3];
					}
					if (triPair.vtcs[3] == tri3)
					{
						return triPair.vtcs[0];
					}
				}
			}

			return -1;
		};

		xIdcs[3] = getIdx(xIdcs[0], xIdcs[1], xIdcs[2]);
		xIdcs[4] = getIdx(xIdcs[1], xIdcs[2], xIdcs[0]);
		xIdcs[5] = getIdx(xIdcs[2], xIdcs[0], xIdcs[1]);

		int notFoundCount = 0;
		for (int i = 3; i < 6; i++) if (xIdcs[i] == -1) notFoundCount += 1;
		if ( notFoundCount != (3 - nTriPairCount) )throw std::logic_error("bug or not manifold or sth");

		auto correctlyRotated = [](const std::array<int, 6> &xIdcs, int notFoundCount) {
			for (int i = 6 - notFoundCount; i < 6; i++)
			{
				if (xIdcs[i] != -1) return false;
			}
			return true;
		};
		auto rotate = [](const std::array<int, 6> &xIdcs) {
			std::array<int, 6> xIdcsNew = {
				xIdcs[1],
				xIdcs[2],
				xIdcs[0],
				xIdcs[4],
				xIdcs[5],
				xIdcs[3]
			};
			return xIdcsNew;
		};

		while (!correctlyRotated(xIdcs, notFoundCount)) xIdcs = rotate(xIdcs);

		int dkfjd = 3;

		keiborhoods[triIdx].vtcs.clear();
		for (int i = 0; i < 6 - notFoundCount; i++)
		{
			keiborhoods[triIdx].vtcs.push_back(xIdcs[i]);
		}
	}

	return TriKeiborhoods(std::move(keiborhoods));
}

}
