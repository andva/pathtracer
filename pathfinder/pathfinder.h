#pragma once
#include <vector>
#include "node.h"
#include "searchspace.h"


namespace pathfinder {

int FindPath(const int nStartX, const int nStartY, const int nTargetX, const int nTargetY, const unsigned char* pMap, const int nMapWidth, const int nMapHeight, int* pOutBuffer, const int nOutBufferSize);

} // namespace pathfinder