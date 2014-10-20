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

    Node(const Vec2* position, unsigned int h, unsigned int g, const int parentId);
    Node();
    Node(const Node& other);
};

} // namespace pathfinder
