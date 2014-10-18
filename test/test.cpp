#include <gtest/gtest.h>
#include <pathfinder/pathfinder.h>
#include <pathfinder/searchspace.h>
#include <memory>

class PathfinderTester : public ::testing::Test  {
 public:
	enum MapType {
		MapTypeNone,
		MapTypeSimple,
		MapTypeSmallImpossible,
		MapTypeLargeImpossible,
		MapTypeSmallDiagonal,
		MapTypeSingleLane,
	};

	void createMap(const MapType mapType, const unsigned int maxSteps) {
		delete[] m_map;
		delete[] m_outBuffer;
		m_outBufferSize = maxSteps;
		m_outBuffer = new int[m_outBufferSize];
		if (mapType == MapTypeNone) {
			m_mapWidth = 0;
			m_mapHeight = 0;
			m_map = new unsigned char[0];
		} else if (mapType == MapTypeSimple) {
			m_mapWidth = 5;
			m_mapHeight = 5;
			unsigned int mapSize = m_mapWidth * m_mapHeight;
			m_map = new unsigned char[mapSize];
			for (unsigned int i = 0; i < mapSize; ++i) {
				m_map[i] = 0;
			}
		} else if (mapType == MapTypeSmallImpossible) {
			m_mapWidth = 5;
			m_mapHeight = 5;
			unsigned int mapSize = m_mapWidth * m_mapHeight;
			m_map = new unsigned char[mapSize];
			for (int y = 0; y < m_mapHeight; ++y)
			for (int x = 0; x < m_mapWidth; ++x) {
				m_map[x + y * m_mapWidth] = (x == 2) ? 1 : 0;
			}
		} else if (mapType == MapTypeLargeImpossible) {
			m_mapWidth = 500;
			m_mapHeight = 500;
			unsigned int mapSize = m_mapWidth * m_mapHeight;
			m_map = new unsigned char[mapSize];
			for (int y = 0; y < m_mapHeight; ++y)
			for (int x = 0; x < m_mapWidth; ++x) {
				m_map[x + y * m_mapWidth] = (x == m_mapWidth - 3) ? 1 : 0;
			}
		} else if (mapType == MapTypeSmallDiagonal) {
			m_mapWidth = 10;
			m_mapHeight = 10;
			unsigned int mapSize = m_mapWidth * m_mapHeight;
			m_map = new unsigned char[mapSize];
			for (int y = 0; y < m_mapHeight; ++y)
			for (int x = 0; x < m_mapWidth; ++x) {
				m_map[x + y * m_mapWidth] = (x == y) ? 1 : 0;
			}
		}
		if (mapType == MapTypeSingleLane) {
			m_mapWidth = 10;
			m_mapHeight = 1;
			unsigned int mapSize = m_mapWidth * m_mapHeight;
			m_map = new unsigned char[mapSize];
			for (unsigned int i = 0; i < mapSize; ++i) {
				m_map[i] = 0;
			}
		}
        m_pathFinder.reset(new pathfinder::PathFinder(m_mapWidth, m_mapHeight, m_outBufferSize, m_map));
	}

 protected:
	virtual void SetUp() {
		m_outBuffer = new int[0];
		m_mapWidth = 0;
		m_mapHeight = 0;
		m_map = new unsigned char[0];
		createMap(MapTypeNone, 0);
	}

	virtual void TearDown() {
		delete[] m_map;
	}
	unsigned char* m_map;
	pathfinder::Vec2 m_start;
	pathfinder::Vec2 m_goal;
	int m_mapWidth;
	int m_mapHeight;
    std::unique_ptr<pathfinder::PathFinder> m_pathFinder;
	int* m_outBuffer;
	int m_outBufferSize;
};

TEST_F(PathfinderTester, ManhattanDistance) {
	EXPECT_EQ(0, pathfinder::distManhattan(m_start, m_goal));
	EXPECT_EQ(2, pathfinder::distManhattan(pathfinder::Vec2(), pathfinder::Vec2(1,1)));
}

TEST_F(PathfinderTester, InsertInitialNodes) {
    unsigned int maxSteps = 2;
	createMap(MapTypeSimple, maxSteps);
	pathfinder::Vec2 start(1, 1);
	pathfinder::Vec2 goal(2, 0);
    pathfinder::PathFinder finder(m_mapWidth, m_mapHeight, maxSteps, m_map);
    pathfinder::SearchSpace searchSpace(m_mapWidth, m_mapHeight, start, goal);
    EXPECT_TRUE(finder.insertInitialNodes(start, goal, &searchSpace));
    EXPECT_FALSE(finder.update(&searchSpace));
}

TEST_F(PathfinderTester, StartAndGoalTests) {
    unsigned int maxSteps = 2;
    createMap(MapTypeSimple, maxSteps);
	pathfinder::Vec2 p1(1, 1);
	pathfinder::Vec2 p2(-1, 1);
	pathfinder::Vec2 p3(m_mapWidth, 1);
	pathfinder::Vec2 p4(1, m_mapHeight);
    
    pathfinder::PathFinder finder(m_mapWidth, m_mapHeight, maxSteps, m_map);
    {
        pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, p1, p2);
        EXPECT_FALSE(finder.insertInitialNodes(p1, p2, &sSpace));
        EXPECT_TRUE(finder.update(&sSpace));
        EXPECT_EQ(-1, sSpace.getPathToTarget(m_outBuffer));
    }
    {
        pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, p3, p2);
        EXPECT_FALSE(finder.insertInitialNodes(p3, p2, &sSpace));
        EXPECT_TRUE(finder.update(&sSpace));
        EXPECT_EQ(-1, sSpace.getPathToTarget(m_outBuffer));
    }
    {
        pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, p4, p2);
        EXPECT_FALSE(finder.insertInitialNodes(p4, p2, &sSpace));
        EXPECT_TRUE(finder.update(&sSpace));
        EXPECT_EQ(-1, sSpace.getPathToTarget(m_outBuffer));
    }
    {
        pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, p4, p2);
        EXPECT_FALSE(finder.insertInitialNodes(p2, p2, &sSpace));
        EXPECT_TRUE(finder.update(&sSpace));
        EXPECT_EQ(-1, sSpace.getPathToTarget(m_outBuffer));
    }
    {
        pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, p1, p1);
        EXPECT_TRUE(finder.insertInitialNodes(p1, p1, &sSpace));
        EXPECT_TRUE(finder.update(&sSpace));
        EXPECT_NE(-1, sSpace.getPathToTarget(m_outBuffer));
    }
    
}

TEST_F(PathfinderTester, AddNeighboringNodes) {
	const int outBufferSize = 4;
	createMap(MapTypeSimple, outBufferSize);
	pathfinder::Vec2 start(1, 1);
	pathfinder::Vec2 goal(2, 0);
    pathfinder::PathFinder finder(m_mapWidth, m_mapHeight, outBufferSize, m_map);
    pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, start, goal);
    EXPECT_TRUE(finder.insertInitialNodes(start, goal, &sSpace));
    EXPECT_FALSE(finder.update(&sSpace));
    finder.addNeighboringNodes(&sSpace);
	
    EXPECT_FALSE(finder.update(&sSpace));
    finder.addNeighboringNodes(&sSpace);

    EXPECT_TRUE(finder.update(&sSpace));
    int steps = sSpace.getPathToTarget(m_outBuffer);
	EXPECT_NE(-1, steps);
    EXPECT_TRUE(sSpace.getNumVisitedNodes() > 1);
}

TEST_F(PathfinderTester, NoSolutionSmall) {
	createMap(MapTypeSmallImpossible, 10);
    
    pathfinder::Vec2 start = pathfinder::Vec2(0, 2);
    pathfinder::Vec2 target = pathfinder::Vec2(m_mapWidth - 1, 2);

    pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, start, target);
    int numSteps = m_pathFinder->findPath(&sSpace, m_outBuffer);
	EXPECT_EQ(-1, numSteps);
    EXPECT_TRUE(sSpace.getNumVisitedNodes() > 1);
}

TEST_F(PathfinderTester, NoSolutionDiagonal) {
	createMap(MapTypeSmallDiagonal, 10);
	pathfinder::Vec2 start(0, m_mapHeight / 2);
	pathfinder::Vec2 target(m_mapWidth - 1, m_mapHeight / 2);
    pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, start, target);
    int numSteps = m_pathFinder->findPath(&sSpace, m_outBuffer);
	EXPECT_EQ(-1, numSteps);
    EXPECT_TRUE(sSpace.getNumVisitedNodes() > 1);
}

TEST_F(PathfinderTester, NoSolutionLarge) {
	createMap(MapTypeLargeImpossible, 900);
	pathfinder::Vec2 start(0, m_mapHeight / 2);
	pathfinder::Vec2 target(m_mapWidth - 1, m_mapHeight / 2);
    pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, start, target);
    int numSteps = m_pathFinder->findPath(&sSpace, m_outBuffer);
	EXPECT_EQ(-1, numSteps);
    EXPECT_TRUE(sSpace.getNumVisitedNodes() > 1);
}

TEST_F(PathfinderTester, SingleLaneMap) {
	createMap(MapTypeSingleLane, 10);
    pathfinder::Vec2 start(0, 0);
    pathfinder::Vec2 target(m_mapWidth - 1, 0);
    pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, start, target);
    int numSteps = m_pathFinder->findPath(&sSpace, m_outBuffer);
	EXPECT_NE(-1, numSteps);
    EXPECT_TRUE(sSpace.getNumVisitedNodes() > 1);
}

TEST_F(PathfinderTester, LoopAllowedLength) {
    createMap(MapTypeSimple, 0);
	pathfinder::Vec2 start(0, m_mapHeight / 2);
    pathfinder::Vec2 target(m_mapWidth - 2, m_mapHeight / 2);
	for (int i = 0; i < m_mapWidth; ++i) {
		createMap(MapTypeSimple, i);
        pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, start, target);
        int numSteps = m_pathFinder->findPath(&sSpace, m_outBuffer);
        bool canFindSolution = target.x - start.x < m_outBufferSize;
        EXPECT_EQ(canFindSolution, numSteps != -1);
        EXPECT_TRUE(canFindSolution ? sSpace.getNumVisitedNodes() > 1 : sSpace.getNumVisitedNodes() == 1);
	}
}
