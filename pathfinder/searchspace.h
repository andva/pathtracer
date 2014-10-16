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
	searchSpace(const int nMapWidth, const int nMapHeight, const unsigned char* const * pMap) :
		m_nMapWidth(nMapWidth), m_nMapHeight(nMapHeight), m_pMap(pMap)
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
	
	bool update(bool& solutionState);
	std::vector<node> m_vNodeVector;
	std::map<int, node> m_visitedNodeMap;
private:
	const int m_nMapWidth;
	const int m_nMapHeight;
	const unsigned char* const * m_pMap;
	vec2 m_start;
	vec2 m_goal;
	node* m_activeNode;
	bool insertNode(const node& node, const node::nodePtr* pos);
	inline int calculateIndex(const vec2& nPos) const;
	bool insertOrdered(const std::vector<node>& insertNodes);
	bool validateVec(const vec2& nPos) const;
	// Add node to rNodeList if position is valid and is not already visited or added but not visited
	bool insertIfValid(const node* pParent, const vec2& nPos, std::vector<node>& rNodeList);
};

} //namespace pathfinder