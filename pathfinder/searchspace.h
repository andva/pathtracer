#pragma once

#include <vector>
#include <map>
#include <iostream>

#include "node.h"
#include "vec2.h"

namespace pathfinder {



class SearchSpace {
 public:
     enum SolutionStatus {
         NoSolution = -1,
     };

     SearchSpace(const int mapWidth, const int mapHeight, const unsigned int maxSteps, const Vec2& startPos, const Vec2& targetPos);
     SearchSpace(const SearchSpace& other);

     bool insertNewNodes(const std::vector<Node>& insertNodes);
     bool insert(const Node& node, const Node::NodeVecIterC* itr);
     void addOutOfRangeNode(const Node& node);
     bool isVisited(const int mapIndex) const;
     bool hasNoSolution() const;
     bool updateActiveNode();

     void updateForNewMaxSteps(const unsigned int maxSteps);

     const int getActiveNodeId() const;
     int getPathToTarget(int* outBuffer) const;
     unsigned int getNumVisitedNodes() const;
     unsigned int getNumActiveNodes() const;
     const Node* const getActiveNode() const;
     const unsigned int getMaxSteps() const;
     const Vec2& getStart() const;
     const Vec2& getTarget() const;
     
     void setMaxSteps(const int maxSteps);
 private:
    
    const int m_mapWidth;
    const int m_mapHeight;
    unsigned int m_maxSteps;

    const Vec2 m_start;
    const Vec2 m_target;

    std::vector<Node> m_approvedNotVisitedNodes;
    std::map<int, Node> m_foundNodes;
    
    std::vector<Node> m_outOfRangeNodes;
    unsigned int m_activeNodeId;

    void getParentValue(int parentValue, const int currentDepth, const int totalDepth, int* outBuffer) const;
};

}  // namespace pathfinder
