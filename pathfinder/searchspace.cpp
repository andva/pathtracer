#include "searchspace.h"

#include <algorithm>
#include <vector>

namespace pathfinder {

SearchSpace::SearchSpace(const int nMapWidth, const int nMapHeight, const Vec2& start, const Vec2& target) : 
m_mapWidth(nMapWidth),
m_mapHeight(nMapHeight),
m_start(start),
m_target(target),
m_activeNode(nullptr) {
    m_approvedNotVisitedNodes.reserve(nMapWidth + nMapHeight);
}

bool SearchSpace::insert(const Node& node, const Node::NodeVecIterC* itr) {
    int mapIndex = node.pos.x + node.pos.y * m_mapWidth;
    m_foundNodes[mapIndex] = node;
    if (itr == nullptr)  {
        m_approvedNotVisitedNodes.push_back(node);
    } else {
        m_approvedNotVisitedNodes.insert(*itr, node);
    }
    return true;
}

void SearchSpace::addOutOfRangeNode(const Node& node) {
    m_outOfRangeNodes.push_back(node);
}

bool SearchSpace::isVisited(const int nMapIndex) const {
    return m_foundNodes.find(nMapIndex) != m_foundNodes.end();
}

bool SearchSpace::insertNewNodes(const std::vector<Node>& insertNodes) {
    if (insertNodes.size() == 0) return false;
    if (m_approvedNotVisitedNodes.size() == 0) {
        Node::NodeVecIterC it = insertNodes.cend();
        Node::NodeVecIterC itBeg = insertNodes.cbegin();
        while (it != itBeg) {
            --it;
            insert(*it, nullptr);
        }
        return true;
    }

    Node::NodeVecIterC currInNode = insertNodes.cbegin();
    Node::NodeVecIterC inNodeEnd = insertNodes.cend();
    Node::NodeVecIterC it = m_approvedNotVisitedNodes.cend();
    Node::NodeVecIterC nvBegin = m_approvedNotVisitedNodes.cbegin();
    while (it != nvBegin) {
        --it;
        while (it->h + it->g >= currInNode->h + currInNode->g) {
            insert(*currInNode, &(it + 1));
            ++currInNode;
            if (currInNode == inNodeEnd) {
                return true;
            }
        }
    }
    return true;
}

void SearchSpace::updateActiveNode() {
    Node& n = m_approvedNotVisitedNodes.back();
    int i = n.pos.x + n.pos.y * m_mapWidth;
    m_approvedNotVisitedNodes.pop_back();
    m_activeNode = &m_foundNodes[i];
}

void SearchSpace::getParentValue(const Node* pNode, const int nIndex, const int depth, int* pOutBuffer) const {
    if (pNode->parent != nullptr) getParentValue(pNode->parent, nIndex + 1, depth, pOutBuffer);
    pOutBuffer[depth - nIndex] = pNode->pos.x + pNode->pos.y * m_mapWidth;
}

int SearchSpace::getPathToTarget(int* pOutBuffer) const {
    const int NO_SOLUTION = -1;
    if (m_activeNode == nullptr) return NO_SOLUTION;
    if (m_activeNode->pos.x != m_target.x || m_activeNode->pos.y != m_target.y) return NO_SOLUTION;
    const Node* nodeItr = m_activeNode;
    getParentValue(nodeItr, 0, nodeItr->g, pOutBuffer);
    return nodeItr->g + 1;
}
}  // namespace pathfinder
