#include "pathfinder.hpp"

#include <math.h>

#include "searchspace.hpp"
#include "node.hpp"



namespace pathfinder {

int FindPath(const int nStartX, const int nStartY, const int nTargetX, const int nTargetY, const unsigned char* pMap, const int nMapWidth, const int nMapHeight, int* pOutBuffer, const int nOutBufferSize)
{
	SearchSpace finder(nMapWidth, nMapHeight, nOutBufferSize, pMap);

	if (!finder.insertInitialNodes(Vec2(nStartX, nStartY), Vec2(nTargetX, nTargetY)))
	{
		return -1;
	}
	else
	{
		bool solutionState = false;
		while (!finder.update(&solutionState))
		{
			finder.addNeighboringNodes();
		}
		return finder.getSolution(pOutBuffer);
	}
	return -1;
}

}  // namespace pathfinder
