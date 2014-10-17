#include "searchspace.h"

#include <algorithm>
#include <vector>

namespace pathfinder {
inline unsigned int distManhattan(const Vec2& sPosA, const Vec2& sPosB) {
    int dx = abs(sPosA.x - sPosB.x);
    int dy = abs(sPosA.y - sPosB.y);
    return dx + dy;
}

SearchSpace::SearchSpace(const int nMapWidth, const int nMapHeight,
    const unsigned int maxSteps, const unsigned char* const& pMap) :
        m_mapWidth(nMapWidth),
        m_mapHeight(nMapHeight),
        m_map(pMap),
        m_maxSteps(maxSteps) {
    m_nodeVector.reserve(static_cast<int>(nMapWidth + nMapHeight));
}

bool SearchSpace::insertNode(const Node& node, const Node::nodePtr* itr) {
    int i = node.pos.x + node.pos.y * m_mapWidth;
    m_visitedNodeMap[i] = node;
    if (itr == nullptr)  {
        m_nodeVector.push_back(node);
    } else {
        m_nodeVector.insert(*itr, node);
    }
    return true;
}

bool SearchSpace::insertOrdered(const std::vector<Node>& insertNodes) {
    if (insertNodes.size() == 0) return false;
    if (m_nodeVector.size() == 0) {
        Node::nodePtr it = insertNodes.cend();
        Node::nodePtr itBeg = insertNodes.cbegin();
        while (it != itBeg) {
            --it;
            insertNode(*it, nullptr);
        }
        return true;
    }

    Node::nodePtr currInNode = insertNodes.cbegin();
    Node::nodePtr inNodeEnd = insertNodes.cend();
    Node::nodePtr it = m_nodeVector.cend();
    Node::nodePtr nvBegin = m_nodeVector.cbegin();
    while (it != nvBegin) {
        --it;
        while (it->h + it->g >= currInNode->h + currInNode->g) {
            insertNode(*currInNode, &(it + 1));
            ++currInNode;
            if (currInNode == inNodeEnd) {
                return true;
            }
        }
    }
    return true;
}

bool SearchSpace::addNeighboringNodes() {
    // Create nodes
    std::vector<Node> insertNodes;
    Vec2 pos = m_activeNode->pos;
    insertIfValid(m_activeNode, Vec2(pos.x + 1, pos.y), &insertNodes);
    insertIfValid(m_activeNode, Vec2(pos.x - 1, pos.y), &insertNodes);
    insertIfValid(m_activeNode, Vec2(pos.x, pos.y + 1), &insertNodes);
    insertIfValid(m_activeNode, Vec2(pos.x, pos.y - 1), &insertNodes);
    return insertOrdered(insertNodes);
}

bool SearchSpace::update(bool* solutionState) {
    bool noSolution = m_nodeVector.size() == 0;
    if (noSolution) {
        solutionState = false;
        return true;
    }
    Node& n = m_nodeVector.back();
    int i = n.pos.x + n.pos.y * m_mapWidth;
    m_nodeVector.pop_back();

    m_activeNode = &m_visitedNodeMap[i];
    bool foundPath = m_activeNode->pos.x == m_goal.x && m_activeNode->pos.y == m_goal.y;
    *solutionState = foundPath;

    return foundPath;
}

bool SearchSpace::validateVec(const Vec2& nPos) const {
    if (!(nPos.x >= 0 && nPos.x < m_mapWidth && nPos.y >= 0 && nPos.y < m_mapHeight))
        return false;
    int i = calculateIndex(nPos);
    unsigned int val = m_map[i];
    return val == MapTileGround;
}

int SearchSpace::calculateIndex(const Vec2& nPos) const {
    return m_mapWidth * nPos.y + nPos.x;
}

bool SearchSpace::insertIfValid(const Node* pParent, const Vec2& nPos, std::vector<Node>* pInOutNodeList) {
    if (!validateVec(nPos)) {
        return false;
    }

    unsigned int h = distManhattan(nPos, m_goal);
    unsigned int g = (pParent != nullptr) ? pParent->g + 1 : 1;

    if (h + g >= m_maxSteps) {
        return false;
    }
    if (m_visitedNodeMap.find(nPos.x + nPos.y * m_mapWidth) != m_visitedNodeMap.end()) {
        return false;
    }

    Node n = Node(&nPos, h, g, pParent);
    for (Node::nodePtr it = pInOutNodeList->cbegin(); it != pInOutNodeList->cend(); ++it) {
        if (h + g <= it->h + it->g) {
            pInOutNodeList->insert(it, n);
            return true;
        }
    }
    pInOutNodeList->push_back(n);
    return true;
}

bool SearchSpace::insertInitialNodes(const Vec2& nStartPos, const Vec2& nGoalPos) {
    if (!validateVec(nStartPos) || !validateVec(nGoalPos)) {
        return false;
    }

    m_start = nStartPos;
    Node start(&nStartPos, 0, 0, nullptr);
    insertNode(start, nullptr);
    m_activeNode = &start;
    m_goal = nGoalPos;
    return true;
}

void SearchSpace::getParentValue(const Node* n, const int i, const int depth, int* pOutBuffer) const {
    if (n->parent != nullptr) getParentValue(n->parent, i + 1, depth, pOutBuffer);
    pOutBuffer[depth - i] = n->pos.x + n->pos.y * m_mapWidth;
}

int SearchSpace::getSolution(int* pOutBuffer) const {
    if (m_activeNode->pos.x == m_goal.x && m_activeNode->pos.y == m_goal.y) {
        const Node* nodeItr = m_activeNode;
        getParentValue(nodeItr, 0, nodeItr->g, pOutBuffer);
        return nodeItr->g + 1;
    } else {
        return -1;
    }
}

}  // namespace pathfinder
