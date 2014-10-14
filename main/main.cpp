#include <pathfinder/pathfinder.h>
#include <cassert>

int main(int argc, char *argv[])
{
	int nMapWidth = 30;
	int nMapHeight = 30;
	unsigned int mapSize = nMapWidth * nMapHeight;
	unsigned int* pMap = new unsigned int[mapSize];
	for (int y = 0; y < nMapWidth; ++y)
	for (int x = 0; x < nMapHeight; ++x)
	{
		pMap[x + y * nMapWidth] = (x == nMapWidth - 3) ? 1 : 0;
	}
	pathfinder::searchSpace* pSearchSpace = new pathfinder::searchSpace(nMapWidth, nMapHeight, &pMap);
	
	bool solutionState = false;
	pSearchSpace->insertInitialNodes(pathfinder::vec2(0, nMapHeight / 2), pathfinder::vec2(nMapWidth - 1, nMapHeight / 2));
	while (pSearchSpace->m_vNodeVector.size() > 0) {
		if (pSearchSpace->update(solutionState))
			assert(!solutionState);
		pSearchSpace->addNeighbouringNodes();
	}
	delete pSearchSpace;
	return 0;
}