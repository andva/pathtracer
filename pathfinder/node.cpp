#include "node.h"
namespace pathfinder {

Node::Node(const Vec2* pPos, unsigned int costToTarget, unsigned int costFromStart, const int parentId)
    : pos(pPos->x, pPos->y)
    , h(costToTarget)
    , g(costFromStart)
    , parent(parentId)
{ }

Node::Node()
    : parent(-1)
    , h(0)
    , g(0)
    , pos()
{ }

Node::Node(const Node& rCopy)
    : pos(rCopy.pos.x, rCopy.pos.y)
    , h(rCopy.h)
    , g(rCopy.g)
    , parent(rCopy.parent)
{ }
}  //  namespace pathfinder
