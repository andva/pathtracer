#include "pathfinder.h"

#include "searchspace.h"
#include "node.h"

#include <math.h>

namespace pathfinder {

int FindPath(const int nStartX, const int nStartY, const int nTargetX, const int nTargetY, const unsigned char* pMap, const int nMapWidth, const int nMapHeight, int* pOutBuffer, const int nOutBufferSize)
{
	pathfinder::searchSpace finder(nMapWidth, nMapHeight, nOutBufferSize, &pMap);

	if (!finder.insertInitialNodes(vec2(nStartX, nStartY), vec2(nTargetX, nTargetY)))
	{
		return -1;
	}
	else
	{
		bool solutionState = false;
		while (!finder.update(solutionState))
		{
			finder.addNeighboringNodes();
		}
		return finder.getSolution(pOutBuffer);
	}
	return -1;
}

} // namespace pathfinder