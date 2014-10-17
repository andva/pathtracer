#include <cassert>

#include <pathfinder/pathfinder.h>

int main(int argc, char *argv[]) {
	int nMapWidth = 500;
	int nMapHeight = 500;
	unsigned int mapSize = nMapWidth * nMapHeight;
	unsigned char* pMap = new unsigned char[mapSize];
	const unsigned int maxSteps = 900;

	for (int y = 0; y < nMapHeight; ++y)
	for (int x = 0; x < nMapWidth; ++x) {
		pMap[x + y * nMapWidth] = (x == nMapWidth - 3) ? 1 : 0;
	}
	
    pathfinder::Vec2 start(0, nMapHeight / 2);
	pathfinder::Vec2 goal(nMapWidth - 1, nMapHeight / 2);
	int* outBuffer = new int[maxSteps];
	int pathLength = pathfinder::FindPath(start.x, start.y, goal.x, goal.y, pMap, nMapWidth, nMapHeight, outBuffer, maxSteps);
	assert(pathLength == -1);
	return 0;
}
