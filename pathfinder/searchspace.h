#pragma once
#include <vector>
#include <map>
#include "node.h"

namespace pathfinder
{

unsigned int distManhattan(const vec2& sPosA, const vec2& sPosB);

namespace
{
	enum MapTile
	{
		MapTileGround = 0,
		MapTileWall = 1,
	};
}

class searchSpace
{
public:
	searchSpace(const int nMapWidth, const int nMapHeight, const int maxSteps, const unsigned char* const * pMap) :
		m_nMapWidth(nMapWidth), m_nMapHeight(nMapHeight), m_pMap(pMap), m_maxSteps(maxSteps)
	{
		m_vNodeVector.reserve(int(nMapWidth * nMapHeight / 9));
	}
	// Adds neighboring nodes if positions are valid and they
	// are not already visited.
	// Assumes that all nodes are validated already
	//
	// Returns false if insertNodes is empty else true
	bool addNeighboringNodes();

	// Adds start and goal positions
	bool insertInitialNodes(const vec2& nStartPos, const vec2& nGoalPos);
	
	// Returns true if finished with searching
	// Sets solutionState to true if goal was found to false if no solution was found
	bool update(bool& solutionState);
	
	// Returns -1 if no solution, otherwise fills pOutBuffer with values
	int getSolution(int* pOutBuffer) const;

	// Returns number of nodes yet to be visited
	int getSearchSpaceSize() const { return m_vNodeVector.size(); }
private:
	const int m_nMapWidth;
	const int m_nMapHeight;
	const int m_maxSteps;
	const unsigned char* const * m_pMap;
	vec2 m_start;
	vec2 m_goal;
	const node* m_activeNode;

	std::vector<node> m_vNodeVector;
	std::map<int, node> m_visitedNodeMap;

	bool insertNode(const node& node, const node::nodePtr* pos);
	inline int calculateIndex(const vec2& nPos) const;
	bool insertOrdered(const std::vector<node>& insertNodes);
	
	// Validates vector against map regions and already visited positions
	bool validateVec(const vec2& nPos) const;
	
	// Add node to rNodeList if position is valid and is not already visited or added but not visited
	bool insertIfValid(const node* pParent, const vec2& nPos, std::vector<node>& rNodeList);


};

} //namespace pathfinder