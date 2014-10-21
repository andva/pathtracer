#include "pathfinder.h"

#include <math.h>

#include "searchspacepool.h"
#include "searchspace.h"
#include "node.h"

namespace pathfinder {
SearchSpacePool* PathFinder::s_objectPool;
std::once_flag PathFinder::singleton_flag;

inline unsigned int PathFinder::distManhattan(const Vec2& posA, const Vec2& posB) {
    int dx = abs(posA.x - posB.x);
    int dy = abs(posA.y - posB.y);
    return dx + dy;
}

int FindPath(
        const int startX,
        const int startY,
        const int targetX,
        const int targetY,
        const unsigned char* map,
        const int mapWidth,
        const int mapHeight,
        int* outBuffer,
        const int outBufferSize) {

    PathFinder finder(mapWidth, mapHeight, outBufferSize, map);

    return finder.findPath(Vec2(startX, startY), Vec2(targetX, targetY), outBufferSize, outBuffer);
}

PathFinder::PathFinder(const int mapWidth, const int mapHeight,
        const unsigned int maxSteps, const unsigned char* const& map)
    : m_mapWidth(mapWidth)
    , m_mapHeight(mapHeight)
    , m_map(map)
{ }

SearchSpacePool* PathFinder::getSearchSpacePool() {
    std::call_once(singleton_flag, initObjectPool);
    return s_objectPool;
}

void PathFinder::initObjectPool()
{
    s_objectPool = new SearchSpacePool();
}

int PathFinder::findPath(const Vec2& startPos, const Vec2& targetPos, const int maxSteps, int* outBuffer)
{
    SearchSpace searchSpace = getSearchSpacePool()->getResource(m_mapWidth, m_mapHeight, maxSteps, startPos, targetPos, m_map);
    if (searchSpace.getNumVisitedNodes() == 0) {
        if (!insertInitialNodes(searchSpace.getStart(), searchSpace.getTarget(), &searchSpace)) {
            return SearchSpace::NoSolution;
        }
    }
    while (!searchSpace.updateActiveNode()) {
        addNeighboringNodes(&searchSpace);
    }
    int numSteps = searchSpace.getPathToTarget(outBuffer);
    if (numSteps == SearchSpace::NoSolution) getSearchSpacePool()->addResource(m_mapWidth, m_mapHeight, searchSpace, m_map);
    return numSteps;
}

bool PathFinder::addNeighboringNodes(SearchSpace* searchSpace) {
    std::vector<Node> insertNodes;
    const int activeNodeId = searchSpace->getActiveNodeId();
    const Node* const activeNode = searchSpace->getActiveNode();
    const Vec2& pos = activeNode->pos;
    insertIfValid(activeNodeId, Vec2(pos.x + 1, pos.y), searchSpace, &insertNodes);
    insertIfValid(activeNodeId, Vec2(pos.x - 1, pos.y), searchSpace, &insertNodes);
    insertIfValid(activeNodeId, Vec2(pos.x, pos.y + 1), searchSpace, &insertNodes);
    insertIfValid(activeNodeId, Vec2(pos.x, pos.y - 1), searchSpace, &insertNodes);
    return searchSpace->insertNewNodes(insertNodes);
}

bool PathFinder::validateVec(const Vec2& pos) const {
    if (!(pos.x >= 0 && pos.x < m_mapWidth && pos.y >= 0 && pos.y < m_mapHeight))
        return false;
    int i = calculateIndex(pos);
    unsigned int val = m_map[i];
    return val == MapTileGround;
}

int PathFinder::calculateIndex(const Vec2& pos) const {
    return m_mapWidth * pos.y + pos.x;
}

bool PathFinder::insertIfValid(const int parentId, const Vec2& pos, SearchSpace* searchSpace, std::vector<Node>* nodeVector) {
    if (!validateVec(pos)) {
        return false;
    }
    const Node* const parent = searchSpace->getActiveNode();
    unsigned int h = distManhattan(pos, searchSpace->getTarget());
    unsigned int g = (parent != nullptr) ? parent->g + 1 : 1;

    if (searchSpace->isVisited(calculateIndex(pos))) {
        return false;
    }

    if (h + g >= searchSpace->getMaxSteps()) {
        searchSpace->addOutOfRangeNode(Node(&pos, h, g, searchSpace->getActiveNodeId()));
        return false;
    }

    for (auto it = nodeVector->cbegin(); it != nodeVector->cend(); ++it) {
        if (h + g <= it->h + it->g) {
            nodeVector->insert(it, Node(&pos, h, g, searchSpace->getActiveNodeId()));
            return true;
        }
    }
    nodeVector->push_back(Node(&pos, h, g, searchSpace->getActiveNodeId()));
    return true;
}

bool PathFinder::insertInitialNodes(const Vec2& startPos, const Vec2& targetPos, SearchSpace* searchSpace) {
    if (!validateVec(startPos) || !validateVec(targetPos)) {
        return false;
    }
    Node start(&startPos, 0, 0, -1);
    searchSpace->insert(start, nullptr);
    
    return true;
}

}  // namespace pathfinder
