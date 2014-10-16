#include <pathfinder/pathfinder.h>
#include <cassert>

int main(int argc, char *argv[])
{
	int nMapWidth = 500;
	int nMapHeight = 500;
	unsigned int mapSize = nMapWidth * nMapHeight;
	unsigned char* pMap = new unsigned char[mapSize];
	for (int y = 0; y < nMapHeight; ++y)
	for (int x = 0; x < nMapWidth; ++x)
	{
		pMap[x + y * nMapWidth] = (x == nMapWidth - 3) ? 1 : 0;
	}
	pathfinder::searchSpace pSearchSpace(nMapWidth, nMapHeight, &pMap);
	
	bool solutionState = false;
	pSearchSpace.insertInitialNodes(pathfinder::vec2(0, nMapHeight / 2), pathfinder::vec2(nMapWidth - 1, nMapHeight / 2));
	while (pSearchSpace.m_vNodeVector.size() > 0) {
		if (pSearchSpace.update(solutionState))
			assert(!solutionState);
		pSearchSpace.addNeighboringNodes();
	}
	return 0;
}