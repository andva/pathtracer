#pragma once
#include <vector>
#include "node.h"
#include "searchspace.h"
#include "searchspacepool.h"

namespace pathfinder {

inline unsigned int distManhattan(const Vec2& sPosA, const Vec2& sPosB);

int FindPath(const int nStartX, const int nStartY, const int nTargetX, 
    const int nTargetY, const unsigned char* pMap, const int nMapWidth, 
    const int nMapHeight, int* pOutBuffer, const int nOutBufferSize);
class PathFinder {
public:
    PathFinder(
        const int nMapWidth,
        const int nMapHeight,
        const unsigned int maxSteps,
        const unsigned char* const& pMap);

    // Adds neighboring nodes if positions are valid and they
    // are not already visited.
    // Assumes that all nodes are validated.
    // Returns false if insertNodes is empty else true
    bool addNeighboringNodes(SearchSpace* pSearchSpace);

    // Adds start and goal positions
    bool insertInitialNodes(const Vec2& nStartPos, const Vec2& nGoalPos, SearchSpace* pSearchSpace);

    // Returns -1 if no solution, otherwise fills pOutBuffer with values
    //int getSolution(int* pOutBuffer) const;

    int findPath(const Vec2& start, const Vec2& target, const int nMaxSteps, int* pOutBuffer);
    //int findPath(SearchSpace* pSearchSpace, int* pOutBuffer);
private:
    enum MapTile {
        MapTileGround = 0,
        MapTileWall = 1,
    };

    // Disallow default and copy constructor
    PathFinder();
    PathFinder(const PathFinder& ss);
    //

    // DO NOT ACCESS THIS DIRECTLY, WILL BREAK EVERYTHING
    // ONLY ACCESS USING getSearchSpacePool()
    static SearchSpacePool* s_objectPool;
    static std::once_flag singleton_flag;
    static void initObjectPool() {
        s_objectPool = new SearchSpacePool();
    }
    //

    const int m_mapWidth;
    const int m_mapHeight;
    //const unsigned int m_maxSteps;
    const unsigned char* const& m_map;

    SearchSpacePool* PathFinder::getSearchSpacePool();

    inline int calculateIndex(const Vec2& nPos) const;

    // Validates vector against map regions and already visited positions
    bool validateVec(const Vec2& nPos) const;

    // Add node to rNodeList if position is valid and is not already visited or added but not visited
    bool insertIfValid(const int nParentId, const Vec2& nPos, SearchSpace* pSearchSpace, std::vector<Node>* pInOutNodeList);

};
} // namespace pathfinder
