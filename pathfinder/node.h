#pragma once

#include <vector>

#include "vec2.h"

namespace pathfinder {

struct Node {
    typedef std::vector<Node>::const_iterator NodeVecIterC;

    Vec2 pos;
    unsigned int h; //Estimated cost to travel to goal
    unsigned int g; //Cost to travel here
    int parent;

    Node(const Vec2* pPos, unsigned int nH, unsigned int nG, const int nParentId);
    Node();
    Node(const Node& other);
};

} // namespace pathfinder
