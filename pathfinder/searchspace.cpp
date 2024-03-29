#include "searchspace.h"

#include <algorithm>
#include <vector>

#include "node.h"
#include "vec2.h"

namespace pathfinder {

SearchSpace::SearchSpace(
        const int mapWidth,
        const int mapHeight,
        const unsigned int maxSteps,
        const Vec2& startPos,
        const Vec2& targetPos)
    : m_mapWidth(mapWidth)
    , m_mapHeight(mapHeight)
    , m_maxSteps(maxSteps)
    , m_start(startPos)
    , m_target(targetPos)
    , m_foundNodes(new std::map<int, Node>())
    , m_activeNodeId(-1)
{

    m_approvedNotVisitedNodes.reserve(mapWidth + mapHeight);
}

SearchSpace::SearchSpace(const SearchSpace& other)
    : m_mapWidth(other.m_mapWidth)
    , m_mapHeight(other.m_mapHeight)
    , m_maxSteps(other.m_maxSteps)
    , m_start(other.m_start)
    , m_target(other.m_target)
    , m_activeNodeId(other.m_activeNodeId)
    , m_approvedNotVisitedNodes(other.m_approvedNotVisitedNodes)
    , m_foundNodes(other.m_foundNodes)
    , m_outOfRangeNodes(other.m_outOfRangeNodes)
{

}

bool SearchSpace::insertAtPosition(const Node& node, const Node::NodeVecIterC* itr) {
    int mapIndex = node.pos.x + node.pos.y * m_mapWidth;
    (*m_foundNodes)[mapIndex] = node;
    if (itr == nullptr)  {
        m_approvedNotVisitedNodes.push_back(node);
    } else {
        m_approvedNotVisitedNodes.insert(*itr, node);
    }
    return true;
}

void SearchSpace::addOutOfRangeNode(const Node& node) {
    m_outOfRangeNodes.push_back(node);
    int mapIndex = node.pos.x + node.pos.y * m_mapWidth;
    (*m_foundNodes)[mapIndex] = node;
}

bool SearchSpace::isVisited(const int nMapIndex) const {
    return m_foundNodes->find(nMapIndex) != m_foundNodes->end();
}

bool SearchSpace::insertNewNodes(const std::vector<Node>& insertNodes) {
    if (insertNodes.size() == 0) return false;
    if (m_approvedNotVisitedNodes.size() == 0) {
        auto it = insertNodes.cend();
        auto itBeg = insertNodes.cbegin();
        while (it != itBeg) {
            --it;
            insertAtPosition(*it, nullptr);
        }
        return true;
    }

    Node::NodeVecIterC currInNode = insertNodes.cbegin();
    Node::NodeVecIterC inNodeEnd = insertNodes.cend();
    Node::NodeVecIterC it = m_approvedNotVisitedNodes.cend();
    Node::NodeVecIterC nvBegin = m_approvedNotVisitedNodes.cbegin();
    while (it != nvBegin) {
        --it;
        unsigned int heuristic = it->h + it->g;
        while (heuristic >= currInNode->h + currInNode->g) {
            insertAtPosition(*currInNode, &(it + 1));
            ++currInNode;
            if (currInNode == inNodeEnd) {
                return true;
            }
        }
    }
    return true;
}

bool SearchSpace::updateActiveNode() {
    if (hasNoSolution()) {
        return true;
    }
    Node& n = m_approvedNotVisitedNodes.back();
    int i = n.pos.x + n.pos.y * m_mapWidth;
    m_approvedNotVisitedNodes.pop_back();
    m_activeNodeId = i;
    Vec2& activeNodePos = (*m_foundNodes)[i].pos;
    return activeNodePos.x == getTarget().x && activeNodePos.y == getTarget().y;
}

bool sortFunc(const Node& a, const Node& b) {
    return a.g + a.h > b.g + b.h;
}

void SearchSpace::updateForNewMaxSteps(const unsigned int maxSteps) {
    m_maxSteps = maxSteps;
    if (m_outOfRangeNodes.empty()) return;
    std::sort(m_outOfRangeNodes.begin(), m_outOfRangeNodes.end(), sortFunc);
    std::vector<Node> validNodes;
    while (m_outOfRangeNodes.back().g + m_outOfRangeNodes.back().h < maxSteps) {
        insertAtPosition(m_outOfRangeNodes.back(), nullptr);
        m_outOfRangeNodes.pop_back();
        if (m_outOfRangeNodes.empty()) break;
    }
}

void SearchSpace::getParentValue(int nParentValue, const int nIndex, const int nDepth, int* outBuffer) const {
    if (nParentValue == -1) {
        return;
    }
    const Node& node = m_foundNodes->find(nParentValue)->second;
    getParentValue(node.parent, nIndex + 1, nDepth, outBuffer);
    outBuffer[nDepth - nIndex] = node.pos.x + node.pos.y * m_mapWidth;
}

int SearchSpace::getPathToTarget(int* outBuffer) const {
    if (m_activeNodeId == -1) return NoSolution;
    const Node& activeNode = m_foundNodes->find(m_activeNodeId)->second;
    if (!activeNode.pos.equal(m_target)) return NoSolution;
    getParentValue(m_activeNodeId, 0, activeNode.g, outBuffer);
    return activeNode.g + 1;
}

bool SearchSpace::hasNoSolution() const {
    return m_approvedNotVisitedNodes.size() == 0;
}

const int SearchSpace::getActiveNodeId() const
{
    return m_activeNodeId;
}

unsigned int SearchSpace::getNumVisitedNodes() const {
    return m_foundNodes->size();
}

unsigned int SearchSpace::getNumActiveNodes() const {
    return m_approvedNotVisitedNodes.size();
}


const Node* const SearchSpace::getActiveNode() const {
    return &(m_foundNodes->find(m_activeNodeId))->second;
}

const Vec2& SearchSpace::getStart() const {
    return m_start;
}
const Vec2& SearchSpace::getTarget() const {
    return m_target;
}

const unsigned int SearchSpace::getMaxSteps() const {
    return m_maxSteps;
}

void SearchSpace::setMaxSteps(const int maxSteps) {
    m_maxSteps = maxSteps;
}

}  // namespace pathfinder
