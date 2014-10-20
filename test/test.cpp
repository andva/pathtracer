#include <gtest/gtest.h>
#include <pathfinder/pathfinder.h>
#include <pathfinder/searchspace.h>

#include <memory>
#include <thread>

class PathfinderTester : public ::testing::Test  {
 public:
    PathfinderTester() {};
	enum MapType {
		MapTypeNone,
		MapTypeSimple,
		MapTypeSmallImpossible,
		MapTypeLargeImpossible,
		MapTypeSmallDiagonal,
		MapTypeSingleLane,
        MapTypeMiddleWall,
	};
    void findPath(const pathfinder::Vec2& start, const pathfinder::Vec2& target, const int nMaxSteps, int* pOutBuffer) {

        int steps = m_pathFinder->findPath(start, target, nMaxSteps, pOutBuffer);
        EXPECT_NE(-1, steps);
        testSolution(nMaxSteps, steps, start, target, pOutBuffer);
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
		} else if (mapType == MapTypeSingleLane) {
			m_mapWidth = 10;
			m_mapHeight = 1;
			unsigned int mapSize = m_mapWidth * m_mapHeight;
			m_map = new unsigned char[mapSize];
			for (unsigned int i = 0; i < mapSize; ++i) {
				m_map[i] = 0;
			}
        } else if (mapType == MapTypeMiddleWall) {
            m_mapWidth = 3;
            m_mapHeight = 3;
            unsigned int mapSize = m_mapWidth * m_mapHeight;
            m_map = new unsigned char[mapSize];
            for (int y = 0; y < m_mapHeight; ++y)
            for (int x = 0; x < m_mapWidth; ++x) {
                m_map[x + y * m_mapWidth] = (x == m_mapWidth / 2 && (y != 0 || y != m_mapHeight - 1)) ? 1 : 0;
            }
        }

        m_pathFinder.reset(new pathfinder::PathFinder(m_mapWidth, m_mapHeight, m_outBufferSize, m_map));
	}

    void testSolution(const int nMaxSteps, const int nSolutionSize, const pathfinder::Vec2& start, const pathfinder::Vec2& target, const int* const pBuffer) {
        if (nSolutionSize > 0) {
            EXPECT_EQ(start.x + start.y * m_mapWidth, pBuffer[0]);
            EXPECT_EQ(target.x + target.y * m_mapWidth, pBuffer[nSolutionSize - 1]);
        }
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
    pathfinder::SearchSpace searchSpace(m_mapWidth, m_mapHeight, maxSteps, start, goal);
    EXPECT_TRUE(finder.insertInitialNodes(start, goal, &searchSpace));
    EXPECT_FALSE(searchSpace.updateActiveNode());
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
        pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, maxSteps, p1, p2);
        EXPECT_FALSE(finder.insertInitialNodes(p1, p2, &sSpace));
        EXPECT_TRUE(sSpace.updateActiveNode());
        EXPECT_EQ(-1, sSpace.getPathToTarget(m_outBuffer));
    }
    {
        pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, maxSteps, p3, p2);
        EXPECT_FALSE(finder.insertInitialNodes(p3, p2, &sSpace));
        EXPECT_TRUE(sSpace.updateActiveNode());
        EXPECT_EQ(-1, sSpace.getPathToTarget(m_outBuffer));
    }
    {
        pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, maxSteps, p4, p2);
        EXPECT_FALSE(finder.insertInitialNodes(p4, p2, &sSpace));
        EXPECT_TRUE(sSpace.updateActiveNode());
        EXPECT_EQ(-1, sSpace.getPathToTarget(m_outBuffer));
    }
    {
        pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, maxSteps, p4, p2);
        EXPECT_FALSE(finder.insertInitialNodes(p2, p2, &sSpace));
        EXPECT_TRUE(sSpace.updateActiveNode());
        EXPECT_EQ(-1, sSpace.getPathToTarget(m_outBuffer));
    }
    {
        pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, maxSteps, p1, p1);
        EXPECT_TRUE(finder.insertInitialNodes(p1, p1, &sSpace));
        EXPECT_TRUE(sSpace.updateActiveNode());
        EXPECT_NE(-1, sSpace.getPathToTarget(m_outBuffer));
    }
    
}

TEST_F(PathfinderTester, AddNeighboringNodes) {
	const unsigned int outBufferSize = 4;
	createMap(MapTypeSimple, outBufferSize);
	pathfinder::Vec2 start(1, 1);
	pathfinder::Vec2 target(2, 0);
    pathfinder::PathFinder finder(m_mapWidth, m_mapHeight, outBufferSize, m_map);
    pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, outBufferSize, start, target);
    EXPECT_TRUE(finder.insertInitialNodes(start, target, &sSpace));
    EXPECT_FALSE(sSpace.updateActiveNode());
    finder.addNeighboringNodes(&sSpace);
	
    EXPECT_FALSE(sSpace.updateActiveNode());
    finder.addNeighboringNodes(&sSpace);

    EXPECT_TRUE(sSpace.updateActiveNode());
    int steps = sSpace.getPathToTarget(m_outBuffer);
	EXPECT_NE(-1, steps);
    testSolution(outBufferSize, steps, start, target, m_outBuffer);
    EXPECT_TRUE(sSpace.getNumVisitedNodes() > 1);
}

TEST_F(PathfinderTester, NoSolutionSmall) {
    const unsigned int outBufferSize = 10;
    createMap(MapTypeSmallImpossible, outBufferSize);
    
    pathfinder::Vec2 start = pathfinder::Vec2(0, 2);
    pathfinder::Vec2 target = pathfinder::Vec2(m_mapWidth - 1, 2);
    int numSteps = m_pathFinder->findPath(start, target, outBufferSize, m_outBuffer);
	EXPECT_EQ(-1, numSteps);
}

TEST_F(PathfinderTester, NoSolutionDiagonal) {
    const unsigned int outBufferSize = 10;
    createMap(MapTypeSmallDiagonal, outBufferSize);
	pathfinder::Vec2 start(0, m_mapHeight / 2);
	pathfinder::Vec2 target(m_mapWidth - 1, m_mapHeight / 2);
    int numSteps = m_pathFinder->findPath(start, target, outBufferSize, m_outBuffer);
	EXPECT_EQ(-1, numSteps);
}

TEST_F(PathfinderTester, NoSolutionLarge) {
    const unsigned int outBufferSize = 10;
    createMap(MapTypeLargeImpossible, outBufferSize);
	pathfinder::Vec2 start(0, m_mapHeight / 2);
	pathfinder::Vec2 target(m_mapWidth - 1, m_mapHeight / 2);
    int numSteps = m_pathFinder->findPath(start, target, outBufferSize, m_outBuffer);
	EXPECT_EQ(-1, numSteps);
}

TEST_F(PathfinderTester, SingleLaneMap) {
    const unsigned int outBufferSize = 10;
    createMap(MapTypeSingleLane, outBufferSize);
    pathfinder::Vec2 start(0, 0);
    pathfinder::Vec2 target(m_mapWidth - 1, 0);
    int numSteps = m_pathFinder->findPath(start, target, outBufferSize, m_outBuffer);
	EXPECT_NE(-1, numSteps);
    testSolution(outBufferSize, numSteps, start, target, m_outBuffer);
}

TEST_F(PathfinderTester, LoopAllowedLength) {
    createMap(MapTypeSingleLane, 0);
	pathfinder::Vec2 start(0, m_mapHeight / 2);
    pathfinder::Vec2 target(m_mapWidth - 2, m_mapHeight / 2);
	for (int i = 0; i < m_mapWidth; ++i) {
        createMap(MapTypeSingleLane, i);
        int numSteps = m_pathFinder->findPath(start, target, static_cast<unsigned int>(i), m_outBuffer);
        bool canFindSolution = target.x - start.x < i;
        EXPECT_EQ(canFindSolution, numSteps != -1);
        testSolution(i, numSteps, start, target, m_outBuffer);
	}
}

TEST_F(PathfinderTester, CacheUsed) {
    createMap(MapTypeMiddleWall, 0);
    pathfinder::Vec2 start(0, m_mapHeight / 2);
    pathfinder::Vec2 target(m_mapWidth - 1, m_mapHeight / 2);
    for (int i = m_mapWidth + 1; i < m_mapWidth * m_mapHeight; ++i) {
        createMap(MapTypeMiddleWall, i);
        int numSteps = m_pathFinder->findPath(start, target, static_cast<unsigned int>(i), m_outBuffer);
        EXPECT_EQ(numSteps == -1, i >= 5);
        testSolution(i, numSteps, start, target, m_outBuffer);
    }
}

TEST_F(PathfinderTester, MultiThreads) {
    const unsigned int outBufferSize = 10;
    createMap(MapTypeSimple, outBufferSize);
    int* t1OutBuffer = new int[outBufferSize];
    std::thread t1(
        &PathfinderTester::findPath,
        this,
        pathfinder::Vec2(0, 0),
        pathfinder::Vec2(4, 0),
        outBufferSize,
        t1OutBuffer
        );
    t1.join();
}