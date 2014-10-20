#include <cassert>

#include <pathfinder/pathfinder.h>

int main(int argc, char *argv[]) {
	int mapWidth = 500;
	int mapHeight = 500;
	unsigned int mapSize = mapWidth * mapHeight;
	unsigned char* map = new unsigned char[mapSize];
	const unsigned int maxSteps = 900;

	for (int y = 0; y < mapHeight; ++y)
	for (int x = 0; x < mapWidth; ++x) {
		map[x + y * mapWidth] = (x == mapWidth - 3) ? 1 : 0;
	}
	
    pathfinder::Vec2 start(0, mapHeight / 2);
	pathfinder::Vec2 goal(mapWidth - 1, mapHeight / 2);
	int* outBuffer = new int[maxSteps];
	int pathLength = pathfinder::FindPath(start.x, start.y, goal.x, goal.y, map, mapWidth, mapHeight, outBuffer, maxSteps);
	assert(pathLength == -1);

    pathLength = pathfinder::FindPath(start.x, start.y, goal.x, goal.y, map, mapWidth, mapHeight, outBuffer, maxSteps);
    assert(pathLength == -1);
	return 0;
}
