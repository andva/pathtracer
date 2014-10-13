#include "pathfinder.h"

#include "searchspace.h"
#include "node.h"

#include <math.h>

namespace pathfinder {

unsigned int distNoSq(const vec2& sPosA, const vec2& sPosB)
{
	int dx = sPosA.x - sPosB.x;
	int dy = sPosA.y - sPosB.y;
	return dx * dx + dy * dy;
}

int FindPath(const int nStartX, const int nStartY, const int nTargetX, const int nTargetY, const unsigned char* pMap, const int nMapWidth, const int nMapHeight, int* pOutBuffer, const int nOutBufferSize)
{
	return 0;
}

int TestFunction(int i)
{
	return i;
}

} // namespace pathfinder