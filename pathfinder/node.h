#pragma once

#include <vector>

namespace pathfinder
{
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
	node()
	{
		h = 0;
		g = 0;
		pos = vec2(0, 0);
	}
};
} // namespace pathfinder

