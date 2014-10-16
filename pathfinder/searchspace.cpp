#include "searchspace.h"
#include <iostream>
#include <algorithm>
#include <functional>
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

bool searchSpace::insertNode(const pathfinder::node& node, const node::nodePtr* pos)
{
	int i = node.pos.x + node.pos.y * m_nMapWidth;
	//if (m_visitedNodeMap.find(i) != m_visitedNodeMap.end()) return false;

	m_visitedNodeMap[i] = node;
	if (pos == nullptr)
	{
		m_vNodeVector.push_back(node);
		
	}
	else
	{
		m_vNodeVector.insert(*pos, node);
	}
	return true;
}

bool searchSpace::insertOrdered(const std::vector<pathfinder::node>& insertNodes)
{
	if (insertNodes.size() == 0) return false;
	if (m_vNodeVector.size() == 0)
	{
		node::nodePtr it = insertNodes.cend();
		node::nodePtr itBeg = insertNodes.cbegin();
		do
		{
			--it;
			insertNode(*it, nullptr);
		} while (it != itBeg);
		return true;
	}
	std::vector<node>::const_iterator currInNode = insertNodes.cbegin();
	node::nodePtr inNodeEnd = insertNodes.end();
	int n = 1;
	node::nodePtr it = m_vNodeVector.end() - n;
	node::nodePtr nvBegin = m_vNodeVector.cbegin();
	do
	{
		while (it->h + it->g >= currInNode->h + currInNode->g && currInNode != inNodeEnd)
		{
			insertNode(*currInNode, &(it + 1));
			++currInNode;
			it = m_vNodeVector.end() - n - 1;
			if (currInNode == inNodeEnd)
			{
				return true;
			}
		}
		if (it != nvBegin)
		{
			--it;
			++n;
		}
		else
			break;
	} while (true);
	return true;
}

bool searchSpace::addNeighboringNodes()
{
	// Create nodes
	std::vector<pathfinder::node> insertNodes;
	insertIfValid(m_activeNode, vec2(m_activeNode->pos.x + 1, m_activeNode->pos.y), insertNodes);
	insertIfValid(m_activeNode, vec2(m_activeNode->pos.x - 1, m_activeNode->pos.y), insertNodes);
	insertIfValid(m_activeNode, vec2(m_activeNode->pos.x, m_activeNode->pos.y + 1), insertNodes);
	insertIfValid(m_activeNode, vec2(m_activeNode->pos.x, m_activeNode->pos.y - 1), insertNodes);
	return insertOrdered(insertNodes);
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
	int i = n.pos.x + n.pos.y * m_nMapWidth;
	m_activeNode = &m_visitedNodeMap[i]; // All nodes are already added
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

bool searchSpace::insertIfValid(const node* pParent, const vec2& nPos, std::vector<pathfinder::node>& rNodeList)
{
	if (!validateVec(nPos))
	{
		return false;
	}

	unsigned int h = distManhattan(nPos, m_goal);
	unsigned int g = 1;

	if (pParent != nullptr)
	{
		g = (*pParent).g + 1;
	}
	if (h + g >= m_maxSteps)
	{
		return false;
	}
	if (m_visitedNodeMap.find(nPos.x + nPos.y * m_nMapWidth) != m_visitedNodeMap.end())
	{
		return false;
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
	insertNode(start, nullptr);
	m_activeNode = &start;
	m_goal = nGoalPos;

	return true;
}

void getParentValue(const node* n, const int nMapWidth, vec2 start, const int i, const int depth, int* pOutBuffer)
{
	if (n->parent != nullptr) getParentValue(n->parent, nMapWidth, start, i + 1, depth, pOutBuffer);
	pOutBuffer[depth - i] = n->pos.x + n->pos.y * nMapWidth;
}

int searchSpace::getSolution(int* pOutBuffer) const
{
	if (m_activeNode->pos.x == m_goal.x && m_activeNode->pos.y == m_goal.y)
	{
		const node* nodeItr = m_activeNode;
		getParentValue(nodeItr, m_nMapWidth, m_start, 0, nodeItr->g, pOutBuffer);
		return nodeItr->g + 1;
	}
	else {
		return -1;
	}
}
