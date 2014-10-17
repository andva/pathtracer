#pragma once

#include <vector>
#include <map>

#include "node.hpp"

namespace pathfinder
{

inline unsigned int distManhattan(const Vec2& sPosA, const Vec2& sPosB);

namespace
{
	enum MapTile
	{
		MapTileGround = 0,
		MapTileWall = 1,
	};
}

class SearchSpace
{
public:
	SearchSpace(const int nMapWidth, const int nMapHeight, const unsigned int maxSteps, const unsigned char* const& pMap);

	// Adds neighboring nodes if positions are valid and they
	// are not already visited.
	// Assumes that all nodes are validated already
	//
	// Returns false if insertNodes is empty else true
	bool addNeighboringNodes();

	// Adds start and goal positions
	bool insertInitialNodes(const Vec2& nStartPos, const Vec2& nGoalPos);
	
	// Returns true if finished with searching
	// Sets solutionState to true if goal was found to false if no solution was found
	bool update(bool* solutionState);
	
	// Returns -1 if no solution, otherwise fills pOutBuffer with values
	int getSolution(int* pOutBuffer) const;

private:
	// Disallow default and copy constructor
	SearchSpace();
	SearchSpace(const SearchSpace& ss);
	//

	const int m_mapWidth;
	const int m_mapHeight;
	const unsigned int m_maxSteps;
	const unsigned char* const& m_map;
	Vec2 m_start;
	Vec2 m_goal;
	const Node* m_activeNode;

	std::vector<Node> m_nodeVector;
	std::map<int, Node> m_visitedNodeMap;

	bool insertNode(const Node& node, const Node::nodePtr* itr);
	inline int calculateIndex(const Vec2& nPos) const;

	// Insert list of nodes in nodeVector
	bool insertOrdered(const std::vector<Node>& insertNodes);
	
	// Validates vector against map regions and already visited positions
	bool validateVec(const Vec2& nPos) const;
	
	// Add node to rNodeList if position is valid and is not already visited or added but not visited
	bool insertIfValid(const Node* pParent, const Vec2& nPos, std::vector<Node>* pInOutNodeList);
	
	//
	void getParentValue(const Node* n, const int i, const int depth, int* pOutBuffer) const;
};

} //namespace pathfinder