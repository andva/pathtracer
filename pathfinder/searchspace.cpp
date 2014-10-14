#include "searchspace.h"
#include <iostream>
#include <algorithm>
#include <set>
#include <list>
#include <cassert>

using namespace pathfinder;

unsigned int pathfinder::distManhattan(const vec2& sPosA, const vec2& sPosB)
{
	int dx = abs(sPosA.x - sPosB.x);
	int dy = abs(sPosA.y - sPosB.y);
	return dx + dy;
}

bool searchSpace::insertOrdered(const std::vector<pathfinder::node>& insertNodes, std::vector<pathfinder::node>& nodeList) const
{
	if (insertNodes.size() == 0) return false;
	if (nodeList.size() == 0)
	{
		node::nodePtr it = insertNodes.cend();
		do
		{
			--it;
			nodeList.push_back(*it);
		} while (it != insertNodes.cbegin());
		return true;
	}
	std::vector<node>::const_iterator currInNode = insertNodes.cbegin();
	node::nodePtr inNodeEnd = insertNodes.end();
	node::nodePtr it = nodeList.end();
	// Add all nodes with lower cost compared to the current lowest
	while (currInNode != inNodeEnd)
	{
		while (it != nodeList.cbegin())
		{
			
			--it;
			if (currInNode->pos.x == it->pos.x && currInNode->pos.y == it->pos.y)
			{
				++currInNode;
				if (currInNode == inNodeEnd) return true;
				else break;
			}
		}
		it = nodeList.end() - 1;
		if (currInNode->h + currInNode->g > it->h + it->g) break;
		nodeList.push_back(*currInNode);
		
		++currInNode;
	}
	// If all nodes had lower cost
	if (currInNode == inNodeEnd) return true;

	// Find correct positions for all other nodes that are supposed to be added
	int n = 1;
	it = nodeList.end() - n;
	do
	{
		while (it->h + it->g >= currInNode->h + currInNode->g && currInNode != inNodeEnd)
		{
			nodeList.insert(it, *currInNode);
			++currInNode;
			it = nodeList.end() - n - 1;
			if (currInNode == inNodeEnd)
			{
				return true;
			}
		}
		if (it != nodeList.cbegin())
		{
			--it;
			++n;
		}
		else
			break;
	} while (true);
	return true;
}

bool searchSpace::addNeighbouringNodes()
{
	// Create nodes
	std::vector<pathfinder::node> insertNodes;
	insertIfValid(&m_activeNode, vec2(m_activeNode->pos.x + 1, m_activeNode->pos.y), insertNodes);
	insertIfValid(&m_activeNode, vec2(m_activeNode->pos.x - 1, m_activeNode->pos.y), insertNodes);
	insertIfValid(&m_activeNode, vec2(m_activeNode->pos.x, m_activeNode->pos.y + 1), insertNodes);
	insertIfValid(&m_activeNode, vec2(m_activeNode->pos.x, m_activeNode->pos.y - 1), insertNodes);
	return insertOrdered(insertNodes, m_vNodeVector);
}

bool searchSpace::update(bool& solutionState)
{
	bool noSolution = m_vNodeVector.size() == 0;
	if (noSolution)
	{
		solutionState = false;
		return true;
	}
	pathfinder::node n = m_vNodeVector.back();
	m_vNodeVector.pop_back();
	m_visitedNodes.push_back(n);

	m_activeNode = m_visitedNodes.cend() - 1;
	bool foundPath = m_activeNode->pos.x == m_goal.x && m_activeNode->pos.y == m_goal.y;
	solutionState = foundPath;
	return foundPath;
}

bool searchSpace::validateVec(const vec2& nPos) const
{
	if (!(nPos.x >= 0 && nPos.x < m_nMapWidth && nPos.y >= 0 && nPos.y < m_nMapHeight))
		return false;
	int i = calculateIndex(nPos);
	unsigned int val = (*m_pMap)[i];
	return val == MapTileGround;
}

int searchSpace::calculateIndex(const vec2& nPos) const
{
	return m_nMapWidth * nPos.y + nPos.x;
}

// Add node to rNodeList if position is valid and is not already visited or added but not visited
bool searchSpace::insertIfValid(const node::nodePtr* pParent, const vec2& nPos, std::vector<pathfinder::node>& rNodeList)
{
	if (!validateVec(nPos))
	{
		return false;
	}

	unsigned int h = distManhattan(nPos, m_goal);
	unsigned int g = 1;

	if (pParent != nullptr)
	{
		g = (**pParent).g + 1;
	}	
	if (m_visitedNodes.size() > 0) 
	{
		node::nodePtr vnEnd = m_visitedNodes.cbegin();
		for (node::nodePtr it = m_visitedNodes.cend(); true;)
		{
			--it;
			vec2 p = it->pos;
			if (p.x == nPos.x && p.y == nPos.y)
			{
				return false;
			}
			if (it == m_visitedNodes.cbegin() || it->g + it->h < h + g) break;
		}
	}

	node n = node(&nPos, h, g, pParent);

	for (node::nodePtr it = rNodeList.cbegin(); it != rNodeList.cend(); ++it)
	{
		if (h + g <= it->h + it->g)
		{
			rNodeList.insert(it, n);
			return true;
		}
	}
	rNodeList.push_back(n);
	return true;
}

bool searchSpace::insertInitialNodes(const vec2& nStartPos, const vec2& nGoalPos)
{
	if (!validateVec(nStartPos) || !validateVec(nGoalPos))
	{
		return false;
	}

	m_start = nStartPos;
	pathfinder::node start(&nStartPos, 0, 0, nullptr);
	m_vNodeVector.push_back(start);
	m_goal = nGoalPos;

	return true;
}

struct less_than_key
{
	inline bool operator() (const pathfinder::node& a, const pathfinder::node& b)
	{
		if (a.pos.x < b.pos.x) return true;
		else if (a.pos.x == b.pos.x && a.pos.y < b.pos.y) return true;
		else return false;
	}
};

int searchSpace::numDuplicates() const
{
	std::vector<pathfinder::node> tmp(m_visitedNodes.size());
	std::copy(m_visitedNodes.begin(), m_visitedNodes.end(), tmp.begin());
	tmp.insert(tmp.end(), m_vNodeVector.begin(), m_vNodeVector.end());
	std::sort(tmp.begin(), tmp.end(), less_than_key());
	node::nodePtr it = tmp.cbegin();
	node::nodePtr nextIt = tmp.cbegin() + 1;
	int duplicates = 0;
	for (; it != tmp.cend() - 1; ++it, ++nextIt)
	{
		if (it->pos.x == nextIt->pos.x && it->pos.y == nextIt->pos.y) {
			++duplicates;
		}
	}
	return duplicates;
}