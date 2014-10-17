#include "pathfinder.h"

#include <math.h>

#include "searchspace.h"
#include "node.h"

namespace pathfinder {

int FindPath(
    const int nStartX, 
    const int nStartY,
    const int nTargetX,
    const int nTargetY,
    const unsigned char* pMap,
    const int nMapWidth,
    const int nMapHeight,
    int* pOutBuffer,
    const int nOutBufferSize) {

    SearchSpace finder(nMapWidth, nMapHeight, nOutBufferSize, pMap);
    return FindPath(nStartX, nStartY, nTargetX, nTargetY, pMap, nMapWidth, nMapHeight, &finder, pOutBuffer, nOutBufferSize);
    /*
    if (!finder.insertInitialNodes(Vec2(nStartX, nStartY), Vec2(nTargetX, nTargetY))) {
        return -1;
    } else {
        bool solutionState = false;
        while (!finder.update(&solutionState)) {
            finder.addNeighboringNodes();
        }
        return finder.getSolution(pOutBuffer);
    }
    return -1;*/
}

int FindPath(
    const int nStartX,
    const int nStartY,
    const int nTargetX,
    const int nTargetY,
    const unsigned char* pMap,
    const int nMapWidth,
    const int nMapHeight,
    SearchSpace* pInOutFinder,
    int* pOutBuffer,
    const int nOutBufferSize) {
    if (!pInOutFinder->insertInitialNodes(Vec2(nStartX, nStartY), Vec2(nTargetX, nTargetY))) {
        return -1;
    }
    else {
        bool solutionState = false;
        while (!(pInOutFinder->update(&solutionState))) {
            pInOutFinder->addNeighboringNodes();
        }
        return pInOutFinder->getSolution(pOutBuffer);
    }
    return -1;
}
}  // namespace pathfinder
