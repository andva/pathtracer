#pragma once
#include <vector>
#include "node.h"
#include "searchspace.h"
#include "searchspacepool.h"

namespace pathfinder {

int FindPath(const int startX, const int startY, const int targetX, 
    const int targetY, const unsigned char* map, const int mapWidth, 
    const int mapHeight, int* outBuffer, const int outBufferSize);

class PathFinder {
public:
    PathFinder(
        const int mapWidth,
        const int mapHeight,
        const unsigned int maxSteps,
        const unsigned char* const& map);

    // Adds neighboring nodes if positions are valid and they
    // are not already visited.
    // Assumes that all nodes are validated.
    // Returns false if insertNodes is empty else true
    bool addNeighboringNodes(SearchSpace* searchSpace);

    bool insertInitialNodes(const Vec2& startPos, const Vec2& targetPos, SearchSpace* searchSpace);

    int findPath(const Vec2& startPos, const Vec2& targetPos, const int maxSteps, int* outBuffer);
    
private:
    enum MapTile {
        MapTileGround = 0,
        MapTileWall = 1,
    };

    PathFinder() = delete;
    PathFinder(const PathFinder& ss) = delete;
    PathFinder& PathFinder::operator= (const PathFinder&) = delete;
    
    const int m_mapWidth;
    const int m_mapHeight;
    const unsigned char* const& m_map;

    inline int calculateIndex(const Vec2& pos) const;

    // Validates vector against map regions and already visited positions
    bool validateVec(const Vec2& pos) const;

    bool insertNodeIfValid(const int parentId, const Vec2& pos, SearchSpace* searchSpace, std::vector<Node>* nodeVector);

    inline unsigned int distManhattan(const Vec2& posA, const Vec2& posB);
};
} // namespace pathfinder
