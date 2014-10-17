#include "node.hpp"
namespace pathfinder {
Node::Node(const Vec2* pPos, unsigned int nH, unsigned int nG, const Node* pParent) :
        pos(pPos->x, pPos->y),
        h(nH),
        g(nG),
        parent(pParent) {
}
Node::Node() :
        parent(nullptr) {
	h = 0;
	g = 0;
	pos = Vec2(0, 0);
}
Node::Node(const Node& other) :
        pos(other.pos.x, other.pos.y),
        h(other.h),
        g(other.g),
        parent(other.parent) {

}
}  //  namespace pathfinder
