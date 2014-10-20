#include "pathfinder.h"

#include <math.h>

#include "searchspacepool.h"
#include "searchspace.h"
#include "node.h"

namespace pathfinder {
SearchSpacePool* PathFinder::s_objectPool;
std::once_flag PathFinder::singleton_flag;

inline unsigned int distManhattan(const Vec2& sPosA, const Vec2& sPosB) {
    int dx = abs(sPosA.x - sPosB.x);
    int dy = abs(sPosA.y - sPosB.y);
    return dx + dy;
}

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

    PathFinder finder(nMapWidth, nMapHeight, nOutBufferSize, pMap);

    return finder.findPath(Vec2(nStartX, nStartY), Vec2(nTargetX, nTargetY), nOutBufferSize, pOutBuffer);
}

SearchSpacePool* PathFinder::getSearchSpacePool() {
    std::call_once(singleton_flag, initObjectPool);
    return s_objectPool;
}

PathFinder::PathFinder(const int nMapWidth, const int nMapHeight,
    const unsigned int maxSteps, const unsigned char* const& pMap) :
    m_mapWidth(nMapWidth),
    m_mapHeight(nMapHeight),
    m_map(pMap) {
}

int PathFinder::findPath(const Vec2& start, const Vec2& target, const int nMaxSteps, int* pOutBuffer)
{
    SearchSpace searchSpace = getSearchSpacePool()->getResource(m_mapWidth, m_mapHeight, nMaxSteps, start, target, m_map);
    if (searchSpace.getNumVisitedNodes() == 0) {
        if (!insertInitialNodes(searchSpace.getStart(), searchSpace.getTarget(), &searchSpace)) {
            return -1;
        }
    }
    while (!searchSpace.updateActiveNode()) {
        addNeighboringNodes(&searchSpace);
    }
    PathFinder::getSearchSpacePool()->addResource(m_mapWidth, m_mapHeight, searchSpace, m_map);
    return searchSpace.getPathToTarget(pOutBuffer);
}

bool PathFinder::addNeighboringNodes(SearchSpace* pSearchSpace) {
    std::vector<Node> insertNodes;
    const int activeNodeId = pSearchSpace->getActiveNodeId();
    const Node* const activeNode = pSearchSpace->getActiveNode();
    const Vec2& pos = activeNode->pos;
    insertIfValid(activeNodeId, Vec2(pos.x + 1, pos.y), pSearchSpace, &insertNodes);
    insertIfValid(activeNodeId, Vec2(pos.x - 1, pos.y), pSearchSpace, &insertNodes);
    insertIfValid(activeNodeId, Vec2(pos.x, pos.y + 1), pSearchSpace, &insertNodes);
    insertIfValid(activeNodeId, Vec2(pos.x, pos.y - 1), pSearchSpace, &insertNodes);
    return pSearchSpace->insertNewNodes(insertNodes);
}

bool PathFinder::validateVec(const Vec2& nPos) const {
    if (!(nPos.x >= 0 && nPos.x < m_mapWidth && nPos.y >= 0 && nPos.y < m_mapHeight))
        return false;
    int i = calculateIndex(nPos);
    unsigned int val = m_map[i];
    return val == MapTileGround;
}

int PathFinder::calculateIndex(const Vec2& nPos) const {
    return m_mapWidth * nPos.y + nPos.x;
}

bool PathFinder::insertIfValid(const int nParentId, const Vec2& nPos, SearchSpace* pSearchSpace, std::vector<Node>* pInOutNodeList) {
    if (!validateVec(nPos)) {
        return false;
    }
    const Node* const parent = pSearchSpace->getActiveNode();
    unsigned int h = distManhattan(nPos, pSearchSpace->getTarget());
    unsigned int g = (parent != nullptr) ? parent->g + 1 : 1;

    if (pSearchSpace->isVisited(calculateIndex(nPos))) {
        return false;
    }

    if (h + g >= pSearchSpace->getMaxSteps()) {
        pSearchSpace->addOutOfRangeNode(Node(&nPos, h, g, pSearchSpace->getActiveNodeId()));
        return false;
    }

    for (auto it = pInOutNodeList->cbegin(); it != pInOutNodeList->cend(); ++it) {
        if (h + g <= it->h + it->g) {
            pInOutNodeList->insert(it, Node(&nPos, h, g, pSearchSpace->getActiveNodeId()));
            return true;
        }
    }
    pInOutNodeList->push_back(Node(&nPos, h, g, pSearchSpace->getActiveNodeId()));
    return true;
}

bool PathFinder::insertInitialNodes(const Vec2& nStartPos, const Vec2& nGoalPos, SearchSpace* pSearchSpace) {
    if (!validateVec(nStartPos) || !validateVec(nGoalPos)) {
        return false;
    }
    Node start(&nStartPos, 0, 0, -1);
    pSearchSpace->insert(start, nullptr);
    
    return true;
}

}  // namespace pathfinder
