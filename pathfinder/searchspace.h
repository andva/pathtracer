#pragma once

#include <vector>
#include <map>
#include <iostream>

#include "node.h"

namespace pathfinder {



class SearchSpace {
 public:
     SearchSpace(const int nMapWidth, const int nMapHeight, const Vec2& start, const Vec2& target);

     bool insertNewNodes(const std::vector<Node>& insertNodes);
     bool insert(const Node& node, const Node::NodeVecIterC* itr);
     void addOutOfRangeNode(const Node& n);
     bool isVisited(const int nMapIndex) const;
     bool hasNoSolution() const { return m_approvedNotVisitedNodes.size() == 0; }
     void updateActiveNode();

     const Node& getActiveNode() const { return *m_activeNode; }
     
     int getPathToTarget(int* pOutBuffer) const;
     unsigned int getNumVisitedNodes() const { return m_foundNodes.size(); }
     const Vec2& getStart() const { return m_start; }
     const Vec2& getTarget() const { return m_target; }
 private:
    const int m_mapWidth;
    const int m_mapHeight;

    const Vec2 m_start;
    const Vec2 m_target;

    std::vector<Node> m_approvedNotVisitedNodes;
    std::map<int, Node> m_foundNodes;
    
    std::vector<Node> m_outOfRangeNodes;
    Node* m_activeNode;

    void getParentValue(const Node* pNode, const int nIndex, const int depth, int* pOutBuffer) const;
};

}  // namespace pathfinder
