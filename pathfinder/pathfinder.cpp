#include "pathfinder.h"
#include <math.h>

namespace pathfinder {

bool searchSpace::addNeighbouringNodes()
{
	// Create nodes
	std::vector<pathfinder::node> insertNodes;
	insertIfValid(&m_activeNode, vec2(m_activeNode->pos.x + 1, m_activeNode->pos.y), insertNodes);
	insertIfValid(&m_activeNode, vec2(m_activeNode->pos.x - 1, m_activeNode->pos.y), insertNodes);
	insertIfValid(&m_activeNode, vec2(m_activeNode->pos.x, m_activeNode->pos.y + 1), insertNodes);
	insertIfValid(&m_activeNode, vec2(m_activeNode->pos.x, m_activeNode->pos.y - 1), insertNodes);

	if (m_vNodeVector.size() == 0)
	{
		
		node::nodePtr it = insertNodes.end();
		do
		{
			--it;
			m_vNodeVector.push_back(*it);
		} while(it != insertNodes.begin());

		//m_vNodeVector = insertNodes;
		
		/*for (node::nodePtr it = insertNodes.cbegin(); it != insertNodes.cend(); ++it)
		{
			m_vNodeVector.push_back(*it);
		}*/
		return true;
	}

	std::vector<node>::iterator currInNode = insertNodes.begin();
	node::nodePtr inNodeEnd = insertNodes.end();
	node::nodePtr it = m_vNodeVector.cend() - 1;
	while(currInNode->h + currInNode->g <= it->h + it->g && currInNode != inNodeEnd)
	{
		m_vNodeVector.push_back(*currInNode);
		++currInNode;
	}
	++it;
	do
	{
		--it;
		while (currInNode->h + currInNode->g <= it->h + it->g && currInNode != inNodeEnd)
		{
			node::nodePtr tmp = it + 1;
			m_vNodeVector.insert(it, *currInNode);
			++currInNode;
			it = tmp;
			if (currInNode == inNodeEnd)
				return true;
		}
	} while (it != m_vNodeVector.cbegin());

	return true;
}

bool searchSpace::update()
{
	pathfinder::node n = m_vNodeVector.back();
	m_vNodeVector.pop_back();
	m_visitedNodes.push_back(n);
	m_activeNode = m_visitedNodes.cend() - 1;
	return m_activeNode->pos.x == m_goal.x && m_activeNode->pos.y == m_goal.y;
}

bool searchSpace::validateVec(const vec2& nPos)
{
	return (nPos.x >= 0 && nPos.x < m_nMapWidth && nPos.y >= 0 && nPos.y < m_nMapHeight);
}

bool searchSpace::insertIfValid(const node::nodePtr* pParent, const vec2& nPos, std::vector<pathfinder::node>& rNodeList)
{
	if (!validateVec(nPos))
	{
		return false;
	}
	
	for (node::nodePtr it = m_visitedNodes.cbegin(); it != m_visitedNodes.cend(); ++it)
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

unsigned int distManhattan(const vec2& sPosA, const vec2& sPosB)
{
	int dx = abs(sPosA.x - sPosB.x);
	int dy = abs(sPosA.y - sPosB.y);
	return dx + dy;
}

unsigned int distNoSq(const vec2& sPosA, const vec2& sPosB)
{
	int dx = sPosA.x - sPosB.x;
	int dy = sPosA.y - sPosB.y;
	return dx * dx + dy * dy;
}

int FindPath(const int nStartX, const int nStartY, const int nTargetX, const int nTargetY, const unsigned char* pMap, const int nMapWidth, const int nMapHeight, int* pOutBuffer, const int nOutBufferSize)
{
	return 0;
}

int TestFunction(int i)
{
	return i;
}

} // namespace pathfinder