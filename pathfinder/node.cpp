#include "node.h"
namespace pathfinder {

Node::Node(const Vec2* pPos, unsigned int nH, unsigned int nG, const int nParentId) :
pos(pPos->x, pPos->y),
h(nH),
g(nG),
parent(nParentId) {}

Node::Node() : parent(-1) {
    h = 0;
    g = 0;
    pos = Vec2(0, 0);
}

Node::Node(const Node& rCopy) :
pos(rCopy.pos.x, rCopy.pos.y),
h(rCopy.h),
g(rCopy.g),
parent(rCopy.parent) {
}
}  //  namespace pathfinder
