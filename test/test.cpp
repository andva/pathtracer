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

    void findPath(const pathfinder::Vec2& startPos, const pathfinder::Vec2& targetPos, const int maxSteps, int* outBuffer) {
        int steps = pathfinder::FindPath(startPos.x, startPos.y, targetPos.x, targetPos.y, m_map, m_mapWidth, m_mapHeight, outBuffer, maxSteps);
        EXPECT_EQ(m_solveable, steps != pathfinder::SearchSpace::NoSolution);
        testSolution(maxSteps, steps, startPos, targetPos, outBuffer);
    };
	void createMap(const MapType eMapType, const unsigned int maxSteps) {
		delete[] m_map;
		delete[] m_outBuffer;
		m_outBufferSize = maxSteps;
		m_outBuffer = new int[m_outBufferSize];
		if (eMapType == MapTypeNone) {
			m_mapWidth = 0;
			m_mapHeight = 0;
			m_map = new unsigned char[0];
            m_solveable = false;
		} else if (eMapType == MapTypeSimple) {
			m_mapWidth = 5;
            m_mapHeight = 5;
            m_start = pathfinder::Vec2(0, m_mapHeight / 2);
            m_target = pathfinder::Vec2(m_mapWidth - 1, m_mapHeight / 2);
			unsigned int mapSize = m_mapWidth * m_mapHeight;
			m_map = new unsigned char[mapSize];
			for (unsigned int i = 0; i < mapSize; ++i) {
				m_map[i] = 0;
			}
            m_solveable = true;
		} else if (eMapType == MapTypeSmallImpossible) {
			m_mapWidth = 5;
			m_mapHeight = 5;
            m_start = pathfinder::Vec2(0, 2);
            m_target = pathfinder::Vec2(m_mapWidth - 1, 2);
			unsigned int mapSize = m_mapWidth * m_mapHeight;
			m_map = new unsigned char[mapSize];
			for (int y = 0; y < m_mapHeight; ++y)
			for (int x = 0; x < m_mapWidth; ++x) {
				m_map[x + y * m_mapWidth] = (x == 2) ? 1 : 0;
			}
            m_solveable = false;
		} else if (eMapType == MapTypeLargeImpossible) {
			m_mapWidth = 500;
			m_mapHeight = 500;
            m_start = pathfinder::Vec2(0, m_mapHeight / 2);
            m_target = pathfinder::Vec2(m_mapWidth - 1, m_mapHeight / 2);
			unsigned int mapSize = m_mapWidth * m_mapHeight;
			m_map = new unsigned char[mapSize];
			for (int y = 0; y < m_mapHeight; ++y)
			for (int x = 0; x < m_mapWidth; ++x) {
				m_map[x + y * m_mapWidth] = (x == m_mapWidth - 3) ? 1 : 0;
			}
            m_solveable = false;
		} else if (eMapType == MapTypeSmallDiagonal) {
			m_mapWidth = 10;
			m_mapHeight = 10;
            m_start = pathfinder::Vec2(0, m_mapHeight / 2);
            m_target = pathfinder::Vec2(m_mapWidth - 1, m_mapHeight / 2);
			unsigned int mapSize = m_mapWidth * m_mapHeight;
			m_map = new unsigned char[mapSize];
			for (int y = 0; y < m_mapHeight; ++y)
			for (int x = 0; x < m_mapWidth; ++x) {
				m_map[x + y * m_mapWidth] = (x == y) ? 1 : 0;
			}
            m_solveable = false;
		} else if (eMapType == MapTypeSingleLane) {
			m_mapWidth = 10;
			m_mapHeight = 1;
            m_start = pathfinder::Vec2(0, 0);
            m_target = pathfinder::Vec2(m_mapWidth - 1, 0);
			unsigned int mapSize = m_mapWidth * m_mapHeight;
			m_map = new unsigned char[mapSize];
			for (unsigned int i = 0; i < mapSize; ++i) {
				m_map[i] = 0;
			}
            m_solveable = true;
        } else if (eMapType == MapTypeMiddleWall) {
            m_mapWidth = 3;
            m_mapHeight = 3; 
            m_start = pathfinder::Vec2(0, m_mapHeight / 2);
            m_target = pathfinder::Vec2(m_mapWidth - 1, m_mapHeight / 2);
            unsigned int mapSize = m_mapWidth * m_mapHeight;
            m_map = new unsigned char[mapSize];
            for (int y = 0; y < m_mapHeight; ++y)
            for (int x = 0; x < m_mapWidth; ++x) {
                int i = (x == m_mapWidth / 2 && y > 0 && y < m_mapHeight - 1) ? 1 : 0;
                m_map[x + y * m_mapWidth] = i;
            }
            m_solveable = true;
        }

        m_pathFinder.reset(new pathfinder::PathFinder(m_mapWidth, m_mapHeight, m_outBufferSize, m_map));
	}

    void testSolution(const int maxSteps, const int nSolutionSize, const pathfinder::Vec2& vStart, const pathfinder::Vec2& vTarget, const int* const pBuffer) {
        if (nSolutionSize > 0) {
            EXPECT_EQ(vStart.x + vStart.y * m_mapWidth, pBuffer[0]);
            EXPECT_EQ(vTarget.x + vTarget.y * m_mapWidth, pBuffer[nSolutionSize - 1]);
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
	pathfinder::Vec2 m_target;
	int m_mapWidth;
	int m_mapHeight;
    bool m_solveable;
    std::unique_ptr<pathfinder::PathFinder> m_pathFinder;
	int* m_outBuffer;
	int m_outBufferSize;
};

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
        EXPECT_EQ(pathfinder::SearchSpace::NoSolution, sSpace.getPathToTarget(m_outBuffer));
    }
    {
        pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, maxSteps, p3, p2);
        EXPECT_FALSE(finder.insertInitialNodes(p3, p2, &sSpace));
        EXPECT_TRUE(sSpace.updateActiveNode());
        EXPECT_EQ(pathfinder::SearchSpace::NoSolution, sSpace.getPathToTarget(m_outBuffer));
    }
    {
        pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, maxSteps, p4, p2);
        EXPECT_FALSE(finder.insertInitialNodes(p4, p2, &sSpace));
        EXPECT_TRUE(sSpace.updateActiveNode());
        EXPECT_EQ(pathfinder::SearchSpace::NoSolution, sSpace.getPathToTarget(m_outBuffer));
    }
    {
        pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, maxSteps, p4, p2);
        EXPECT_FALSE(finder.insertInitialNodes(p2, p2, &sSpace));
        EXPECT_TRUE(sSpace.updateActiveNode());
        EXPECT_EQ(pathfinder::SearchSpace::NoSolution, sSpace.getPathToTarget(m_outBuffer));
    }
    {
        pathfinder::SearchSpace sSpace(m_mapWidth, m_mapHeight, maxSteps, p1, p1);
        EXPECT_TRUE(finder.insertInitialNodes(p1, p1, &sSpace));
        EXPECT_TRUE(sSpace.updateActiveNode());
        EXPECT_NE(pathfinder::SearchSpace::NoSolution, sSpace.getPathToTarget(m_outBuffer));
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
    EXPECT_NE(pathfinder::SearchSpace::NoSolution, steps);
    testSolution(outBufferSize, steps, start, target, m_outBuffer);
    EXPECT_TRUE(sSpace.getNumVisitedNodes() > 1);
}

TEST_F(PathfinderTester, SimpleOneStep) {
    const unsigned int outBufferSize = 10;
    createMap(MapTypeSimple, outBufferSize);
    m_target = pathfinder::Vec2(m_start.x + 1, m_start.y);
    int numSteps = m_pathFinder->findPath(m_start, m_target, outBufferSize, m_outBuffer);
    testSolution(outBufferSize, numSteps, m_start, m_target, m_outBuffer);
    EXPECT_NE(pathfinder::SearchSpace::NoSolution, numSteps);
}

TEST_F(PathfinderTester, NoSolutionSmall) {
    const unsigned int outBufferSize = 10;
    createMap(MapTypeSmallImpossible, outBufferSize);
    int numSteps = m_pathFinder->findPath(m_start, m_target, outBufferSize, m_outBuffer);
    EXPECT_EQ(pathfinder::SearchSpace::NoSolution, numSteps);
}

TEST_F(PathfinderTester, NoSolutionDiagonal) {
    const unsigned int outBufferSize = 10;
    createMap(MapTypeSmallDiagonal, outBufferSize);
    int numSteps = m_pathFinder->findPath(m_start, m_target, outBufferSize, m_outBuffer);
    EXPECT_EQ(pathfinder::SearchSpace::NoSolution, numSteps);
}

TEST_F(PathfinderTester, NoSolutionLarge) {
    const unsigned int outBufferSize = 1000;
    createMap(MapTypeLargeImpossible, outBufferSize);
    int numSteps = pathfinder::FindPath(m_start.x, m_start.y, m_target.x, m_target.y, m_map, m_mapWidth, m_mapHeight, m_outBuffer, m_outBufferSize);
    //int numSteps = m_pathFinder->findPath(start, target, outBufferSize, m_outBuffer);
    EXPECT_EQ(pathfinder::SearchSpace::NoSolution, numSteps);
}

TEST_F(PathfinderTester, NoSolutionLargeCached) {
    const unsigned int outBufferSize = 1000;
    createMap(MapTypeLargeImpossible, outBufferSize);
    int numSteps = m_pathFinder->findPath(m_start, m_target, outBufferSize, m_outBuffer);
    EXPECT_EQ(pathfinder::SearchSpace::NoSolution, numSteps);
}

TEST_F(PathfinderTester, SingleLaneMap) {
    const unsigned int outBufferSize = 10;
    createMap(MapTypeSingleLane, outBufferSize);
    int numSteps = m_pathFinder->findPath(m_start, m_target, outBufferSize, m_outBuffer);
    EXPECT_NE(pathfinder::SearchSpace::NoSolution, numSteps);
    testSolution(outBufferSize, numSteps, m_start, m_target, m_outBuffer);
}

TEST_F(PathfinderTester, LoopAllowedLength) {
    createMap(MapTypeSingleLane, 0);
	for (int i = 0; i < m_mapWidth; ++i) {
        createMap(MapTypeSingleLane, i);
        int numSteps = m_pathFinder->findPath(m_start, m_target, static_cast<unsigned int>(i), m_outBuffer);
        bool canFindSolution = m_target.x - m_start.x < i;
        EXPECT_EQ(canFindSolution, numSteps != -1);
        testSolution(i, numSteps, m_start, m_target, m_outBuffer);
	}
}

TEST_F(PathfinderTester, CacheUsed) {
    createMap(MapTypeMiddleWall, 0);
    for (int i = m_mapWidth + 1; i < m_mapWidth * m_mapHeight; ++i) {
        createMap(MapTypeMiddleWall, i);
        int numSteps = m_pathFinder->findPath(m_start, m_target, static_cast<unsigned int>(i), m_outBuffer);
        EXPECT_EQ(numSteps == pathfinder::SearchSpace::NoSolution, i < 5);
        testSolution(i, numSteps, m_start, m_target, m_outBuffer);
    }
}

TEST_F(PathfinderTester, MultiThreadsSolveable) {
    const unsigned int outBufferSize = 10;
    createMap(MapTypeSimple, outBufferSize);
    std::vector<int*> outBuffers;
    std::vector<std::thread*> threads;
    const int NUM_THREADS = 5;
    for (int i = 0; i < NUM_THREADS; ++i) {
        int* buffer = new int[outBufferSize];
        outBuffers.push_back(buffer);
        threads.push_back(new std::thread(
            &PathfinderTester::findPath,
            this,
            pathfinder::Vec2(0, 0),
            pathfinder::Vec2(4, 0),
            outBufferSize,
            outBuffers[i]
            ));
    }
    for (int i = 0; i < NUM_THREADS; ++i) {
        threads[i]->join();
    }
    while (!threads.empty()) delete threads.back(), threads.pop_back();
    while (!outBuffers.empty()) delete outBuffers.back(), outBuffers.pop_back();
}

TEST_F(PathfinderTester, MultiThreadsImposible) {
    const unsigned int outBufferSize = 10;
    createMap(MapTypeSmallImpossible, outBufferSize);
    std::vector<int*> outBuffers;
    std::vector<std::thread*> threads;
    const int NUM_THREADS = 5;
    for (int i = 0; i < NUM_THREADS; ++i) {
        outBuffers.push_back(new int[outBufferSize]);
        threads.push_back(new std::thread(
            &PathfinderTester::findPath,
            this,
            pathfinder::Vec2(0, m_mapHeight / 2),
            pathfinder::Vec2(m_mapWidth - 1, m_mapHeight / 2),
            outBufferSize,
            outBuffers[i]
            ));
    }
    for (int i = 0; i < NUM_THREADS; ++i) {
        threads[i]->join();
    }
    while (!threads.empty()) delete threads.back(), threads.pop_back();
    while (!outBuffers.empty()) delete outBuffers.back(), outBuffers.pop_back();
}