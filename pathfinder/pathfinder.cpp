#include "pathfinder.h"

#include <math.h>

#include "objectpool.h"
#include "searchspace.h"
#include "node.h"

namespace pathfinder {
ObjectPool* PathFinder::s_instance;
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

    return finder.findPath(Vec2(nStartX, nStartY), Vec2(nTargetX, nTargetY), pOutBuffer);
}

ObjectPool* PathFinder::getInstance() {
    /*if (s_instance == nullptr) {
    ObjectPool* temp = new ObjectPool();

    if (InterlockedCompareExchangePointer(nullptr, static_cast<void*>(temp), static_cast<void*>(s_instance))) {
    delete temp;
    }
    }*/
    std::call_once(singleton_flag, init_singleton);
    return s_instance;
}

PathFinder::PathFinder(const int nMapWidth, const int nMapHeight,
    const unsigned int maxSteps, const unsigned char* const& pMap) :
    m_mapWidth(nMapWidth),
    m_mapHeight(nMapHeight),
    m_map(pMap),
    m_maxSteps(maxSteps) {
}

int PathFinder::findPath(const Vec2& start, const Vec2& target, int* pOutBuffer)
{
    SearchSpace searchSpace(m_mapWidth, m_mapHeight, start, target);
    int asd = findPath(&searchSpace, pOutBuffer);
    ObjectPool::getInstance()->addResource(m_mapWidth, m_mapHeight, searchSpace, m_map);
    return asd;
}

int PathFinder::findPath(SearchSpace* pSearchSpace, int* pOutBuffer) {
    if (!insertInitialNodes(pSearchSpace->getStart(), pSearchSpace->getTarget(), pSearchSpace)) {
        return -1;
    }
    else {
        while (!update(pSearchSpace)) {
            addNeighboringNodes(pSearchSpace);
        }
        ObjectPool::getInstance()->addResource(m_mapWidth, m_mapHeight, *pSearchSpace, m_map);
        return pSearchSpace->getPathToTarget(pOutBuffer);
    }
    return -1;
}

bool PathFinder::addNeighboringNodes(SearchSpace* pSearchSpace) {
    std::vector<Node> insertNodes;
    const Node& activeNode = pSearchSpace->getActiveNode();
    Vec2 pos = activeNode.pos;
    insertIfValid(&activeNode, Vec2(pos.x + 1, pos.y), pSearchSpace, &insertNodes);
    insertIfValid(&activeNode, Vec2(pos.x - 1, pos.y), pSearchSpace, &insertNodes);
    insertIfValid(&activeNode, Vec2(pos.x, pos.y + 1), pSearchSpace, &insertNodes);
    insertIfValid(&activeNode, Vec2(pos.x, pos.y - 1), pSearchSpace, &insertNodes);
    return pSearchSpace->insertNewNodes(insertNodes);
}

bool PathFinder::update(SearchSpace* pSearchSpace) {
    if (pSearchSpace->hasNoSolution()) {
        return true;
    }
    pSearchSpace->updateActiveNode();
    const Node& activeNode = pSearchSpace->getActiveNode();
    return activeNode.pos.x == m_goal.x && activeNode.pos.y == m_goal.y;
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

bool PathFinder::insertIfValid(const Node* const pParent, const Vec2& nPos, SearchSpace* searchSpace, std::vector<Node>* pInOutNodeList) {
    if (!validateVec(nPos)) {
        return false;
    }

    unsigned int h = distManhattan(nPos, m_goal);
    unsigned int g = (pParent != nullptr) ? pParent->g + 1 : 1;

    if (h + g >= m_maxSteps) {
        searchSpace->addOutOfRangeNode(Node(&nPos, h, g, pParent));
        return false;
    }
    if (searchSpace->isVisited(calculateIndex(nPos))) {
        return false;
    }

    
    for (Node::NodeVecIterC it = pInOutNodeList->cbegin(); it != pInOutNodeList->cend(); ++it) {
        if (h + g <= it->h + it->g) {
            pInOutNodeList->insert(it, Node(&nPos, h, g, pParent));
            return true;
        }
    }
    pInOutNodeList->push_back(Node(&nPos, h, g, pParent));
    return true;
}

bool PathFinder::insertInitialNodes(const Vec2& nStartPos, const Vec2& nGoalPos, SearchSpace* pSearchSpace) {
    if (!validateVec(nStartPos) || !validateVec(nGoalPos)) {
        return false;
    }

    m_start = nStartPos;
    Node start(&nStartPos, 0, 0, nullptr);
    pSearchSpace->insert(start, nullptr);
    m_goal = nGoalPos;
    return true;
}

}  // namespace pathfinder
