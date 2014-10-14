#include "searchspace.h"
#include <iostream>

using namespace pathfinder;

unsigned int pathfinder::distManhattan(const vec2& sPosA, const vec2& sPosB)
{
	int dx = abs(sPosA.x - sPosB.x);
	int dy = abs(sPosA.y - sPosB.y);
	return dx + dy;
}

bool insertOrdered(const std::vector<pathfinder::node>& insertNodes, std::vector<pathfinder::node>& nodeList)
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
	node::nodePtr it = nodeList.end() - 1;
	// Add all nodes with lower cost compared to the current lowest
	while (currInNode != inNodeEnd)
	{
		if (currInNode->h + currInNode->g > it->h + it->g) break;
		nodeList.push_back(*currInNode);
		it = nodeList.end() - 1;
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
				return true;
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
	/*if (m_vNodeVector.size() == 0)
	{
		if (insertNodes.size() != 0)
		{
			node::nodePtr it = insertNodes.end();
			do
			{
				--it;
				m_vNodeVector.push_back(*it);
			} while (it != insertNodes.begin());
			return true;
		}
		return false;
	}

	std::vector<node>::iterator currInNode = insertNodes.begin();
	node::nodePtr inNodeEnd = insertNodes.end();
	node::nodePtr it = m_vNodeVector.end() - 1;
	// Add all nodes with lower cost compared to the current lowest
	while (currInNode != inNodeEnd)
	{
		if (currInNode->h + currInNode->g > it->h + it->g) break;
		m_vNodeVector.push_back(*currInNode);
		it = m_vNodeVector.end() - 1;
		++currInNode;
	}
	// If all nodes had lower cost
	if (currInNode == inNodeEnd) return true;
	
	// Find correct positions for all other nodes that are supposed to be added
	int n = 1;
	it = m_vNodeVector.end() - n;
	do
	{
		while (it->h + it->g >= currInNode->h + currInNode->g && currInNode != inNodeEnd)
		{
			m_vNodeVector.insert(it, *currInNode);
			++currInNode;
			it = m_vNodeVector.end() - n - 1;
			if (currInNode == inNodeEnd)
				return true;
		}
		if (it != m_vNodeVector.cbegin())
		{
			--it;
			++n;
		}
		else
			break;
	} while (true);

	return true;*/
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
	node::nodePtr vnEnd = m_visitedNodes.cend();
	for (node::nodePtr it = m_visitedNodes.cbegin(); it != vnEnd; ++it)
	{
		if (it->pos.x == nPos.x && it->pos.y == nPos.y)
		{
			return false;
		}
	}

	
	unsigned int h = distManhattan(nPos, m_goal);
	unsigned int g = 1;

	if (pParent != nullptr)
	{
		g = (**pParent).g + 1;
	}
	/*if (m_vNodeVector.size() > 0)
	{
		for (node::nodePtr it = m_vNodeVector.cbegin(); it != m_vNodeVector.cend(); ++it)
		{
			if (it->pos.x == nPos.x && it->pos.y == nPos.y)
			{
				//std::cout << "HHEJEE REMOVE " << __FUNCTION__ << std::endl;
				return false;
			}
		}
	}*/
	

	node n = node(&nPos, h, g, pParent);
	int offset = rNodeList.size() == 0 ? 0 : 1;

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