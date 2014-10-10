#pragma once
#include <vector>

namespace pathfinder {
struct vec2
{
	int x;
	int y;
	vec2(int x, int y) {
		this->x = x;
		this->y = y;
	}
	vec2() : x(0), y(0) {
	}
};

struct node
{
	typedef std::vector<node>::const_iterator nodePtr;
	vec2 pos;
	unsigned int h; //Estimated cost to travel to goal
	unsigned int g; //Cost to travel here
	const nodePtr* parent;
	node(const vec2* pPos, unsigned int nH, unsigned int nG, const nodePtr* pParent) : 
		pos(pPos->x, pPos->y), h(nH), g(nG), parent(pParent) 
	{}
};

class searchSpace
{
public:
	searchSpace(const int nMapWidth, const int nMapHeight, const unsigned int* pMap) :
		m_nMapWidth(nMapWidth), m_nMapHeight(nMapHeight), m_pMap(pMap) 
	{}
	bool addNeighbouringNodes();
	bool insertInitialNodes(const vec2& nStartPos, const vec2& nGoalPos);
	bool update();
	std::vector<node> m_vNodeVector;
	std::vector<node> m_visitedNodes;

private:
	const int m_nMapWidth;
	const int m_nMapHeight;
	const unsigned int* m_pMap;
	vec2 m_start;
	vec2 m_goal;
	node::nodePtr m_activeNode;
	
	bool validateVec(const vec2& nPos);
	bool insertIfValid(const node::nodePtr* pParent, const vec2& nPos, std::vector<pathfinder::node>& rNodeList);
};


unsigned int distNoSq(const vec2& sPosA, const vec2& sPosB);

unsigned int distManhattan(const vec2& sPosA, const vec2& sPosB);

int TestFunction(int i);

int FindPath(const int nStartX, const int nStartY, const int nTargetX, const int nTargetY, const unsigned char* pMap, const int nMapWidth, const int nMapHeight, int* pOutBuffer, const int nOutBufferSize);

} // namespace pathfinder