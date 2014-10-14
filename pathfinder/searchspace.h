#pragma once
#include <vector>

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
	searchSpace(const int nMapWidth, const int nMapHeight, unsigned int* const * pMap) :
		m_nMapWidth(nMapWidth), m_nMapHeight(nMapHeight), m_pMap(pMap)
	{
		m_vNodeVector.reserve(int(nMapWidth * nMapHeight / 9));
		m_visitedNodes.reserve(int(nMapWidth * nMapHeight / 9));
	}
	bool addNeighbouringNodes();
	bool insertInitialNodes(const vec2& nStartPos, const vec2& nGoalPos);
	int numDuplicates() const;
	bool update(bool& solutionState);
	std::vector<node> m_vNodeVector;
	std::vector<node> m_visitedNodes;

private:
	const int m_nMapWidth;
	const int m_nMapHeight;
	unsigned int* const * m_pMap;
	vec2 m_start;
	vec2 m_goal;
	node::nodePtr m_activeNode;
	inline int calculateIndex(const vec2& nPos) const;
	bool insertOrdered(const std::vector<pathfinder::node>& insertNodes, std::vector<pathfinder::node>& nodeList) const;
	bool validateVec(const vec2& nPos) const;
	bool insertIfValid(const node::nodePtr* pParent, const vec2& nPos, std::vector<pathfinder::node>& rNodeList);
};

} //namespace pathfinder