#include <gtest/gtest.h>
#include <pathfinder/pathfinder.hpp>
#include <pathfinder/searchspace.hpp>

class PathfinderTester : public ::testing::Test 
{
public:
	enum MapType 
	{
		MapTypeNone = 1 << 0,
		MapTypeSimple = 1 << 1,
		MapTypeSmallImpossible = 1 << 2,
		MapTypeLargeImpossible = 1 << 3,
		MapTypeSmallDiagonal = 1 << 4,
		MapTypeSingleLane = 1 << 5,
	};

	int findPath(const pathfinder::Vec2& start, const pathfinder::Vec2& goal)
	{
		return pathfinder::FindPath(start.x, start.y, goal.x, goal.y, m_map, m_mapWidth, m_mapHeight, m_outBuffer, m_outBufferSize);
	}

	void createMap(unsigned int mapType, const unsigned int maxSteps)
	{
		delete[] m_map;
		delete[] m_outBuffer;
		m_outBufferSize = maxSteps;
		m_outBuffer = new int[m_outBufferSize];
		if (mapType == MapTypeNone)
		{
			m_mapWidth = 0;
			m_mapHeight = 0;
			m_map = new unsigned char[0];
		}
		else if (mapType == MapTypeSimple)
		{
			m_mapWidth = 5;
			m_mapHeight = 5;
			unsigned int mapSize = m_mapWidth * m_mapHeight;
			m_map = new unsigned char[mapSize];
			for (unsigned int i = 0; i < mapSize; ++i)
			{
				m_map[i] = 0;
			}
		}
		else if (mapType == MapTypeSmallImpossible)
		{
			m_mapWidth = 5;
			m_mapHeight = 5;
			unsigned int mapSize = m_mapWidth * m_mapHeight;
			m_map = new unsigned char[mapSize];
			for (int y = 0; y < m_mapHeight; ++y)
			for (int x = 0; x < m_mapWidth; ++x)
			{
				m_map[x + y * m_mapWidth] = (x == 2) ? 1 : 0;
			}
		}
		else if (mapType == MapTypeLargeImpossible)
		{
			m_mapWidth = 500;
			m_mapHeight = 500;
			unsigned int mapSize = m_mapWidth * m_mapHeight;
			m_map = new unsigned char[mapSize];
			for (int y = 0; y < m_mapHeight; ++y)
			for (int x = 0; x < m_mapWidth; ++x)
			{
				m_map[x + y * m_mapWidth] = (x == m_mapWidth - 3) ? 1 : 0;
			}
		}
		else if (mapType == MapTypeSmallDiagonal)
		{
			m_mapWidth = 10;
			m_mapHeight = 10;
			unsigned int mapSize = m_mapWidth * m_mapHeight;
			m_map = new unsigned char[mapSize];
			for (int y = 0; y < m_mapHeight; ++y)
			for (int x = 0; x < m_mapWidth; ++x)
			{
				m_map[x + y * m_mapWidth] = (x == y) ? 1 : 0;
			}
		}
		if (mapType == MapTypeSingleLane)
		{
			m_mapWidth = 10;
			m_mapHeight = 1;
			unsigned int mapSize = m_mapWidth * m_mapHeight;
			m_map = new unsigned char[mapSize];
			for (unsigned int i = 0; i < mapSize; ++i)
			{
				m_map[i] = 0;
			}
		}
		
		m_searchSpace = new pathfinder::SearchSpace(m_mapWidth, m_mapHeight, maxSteps, m_map);
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
		delete m_searchSpace;
	}
	unsigned char* m_map;
	pathfinder::Vec2 m_start;
	pathfinder::Vec2 m_goal;
	int m_mapWidth;
	int m_mapHeight;
	pathfinder::SearchSpace* m_searchSpace;
	int* m_outBuffer;
	int m_outBufferSize;
};

TEST_F(PathfinderTester, ManhattanDistance)
{
	EXPECT_EQ(0, pathfinder::distManhattan(m_start, m_goal));
	EXPECT_EQ(2, pathfinder::distManhattan(pathfinder::Vec2(), pathfinder::Vec2(1,1)));
}

TEST_F(PathfinderTester, InsertInitialNodes)
{

	createMap(MapTypeSimple, 2);
	pathfinder::Vec2 start(1, 1);
	pathfinder::Vec2 goal(2, 0);
	bool solutionState = false;
	EXPECT_TRUE(m_searchSpace->insertInitialNodes(start, goal));
	EXPECT_FALSE(m_searchSpace->update(&solutionState));
}

TEST_F(PathfinderTester, StartAndGoalTests)
{
	createMap(MapTypeSimple, 2);
	pathfinder::Vec2 p1(1, 1);
	pathfinder::Vec2 p2(-1, 1);
	pathfinder::Vec2 p3(m_mapWidth, 1);
	pathfinder::Vec2 p4(1, m_mapHeight);
	bool solutionState = false;
	EXPECT_FALSE(m_searchSpace->insertInitialNodes(p1, p2));
    EXPECT_TRUE(m_searchSpace->update(&solutionState));
	EXPECT_FALSE(solutionState);

	EXPECT_FALSE(m_searchSpace->insertInitialNodes(p3, p2));
    EXPECT_TRUE(m_searchSpace->update(&solutionState));
	EXPECT_FALSE(solutionState);

	EXPECT_FALSE(m_searchSpace->insertInitialNodes(p4, p2));
    EXPECT_TRUE(m_searchSpace->update(&solutionState));
	EXPECT_FALSE(solutionState);

	EXPECT_FALSE(m_searchSpace->insertInitialNodes(p2, p2));
    EXPECT_TRUE(m_searchSpace->update(&solutionState));
	EXPECT_FALSE(solutionState);

	EXPECT_TRUE(m_searchSpace->insertInitialNodes(p1, p1));
    EXPECT_TRUE(m_searchSpace->update(&solutionState));
	EXPECT_TRUE(solutionState);
}

TEST_F(PathfinderTester, AddNeighboringNodes)
{
	const int outBufferSize = 4;
	createMap(MapTypeSimple, outBufferSize);
	pathfinder::Vec2 start(1, 1);
	pathfinder::Vec2 goal(2, 0);
	bool solutionState = false;
	EXPECT_TRUE(m_searchSpace->insertInitialNodes(start, goal));
    EXPECT_FALSE(m_searchSpace->update(&solutionState));
	m_searchSpace->addNeighboringNodes();
	
    EXPECT_FALSE(m_searchSpace->update(&solutionState));
	m_searchSpace->addNeighboringNodes();

    EXPECT_TRUE(m_searchSpace->update(&solutionState));
	EXPECT_TRUE(solutionState);
	int steps = m_searchSpace->getSolution(m_outBuffer);
	EXPECT_TRUE(steps != -1);
}

TEST_F(PathfinderTester, NoSolutionSmall)
{
	createMap(MapTypeSmallImpossible, 10);
	int numSteps = findPath(pathfinder::Vec2(0, 2), pathfinder::Vec2(4, 2));
	EXPECT_EQ(-1, numSteps);
}

TEST_F(PathfinderTester, NoSolutionDiagonal)
{
	createMap(MapTypeSmallDiagonal, 10);
	pathfinder::Vec2 start(0, m_mapHeight / 2);
	pathfinder::Vec2 goal(m_mapWidth - 1, m_mapHeight / 2);
	int numSteps = findPath(start, goal);
	EXPECT_EQ(-1, numSteps);
}

TEST_F(PathfinderTester, NoSolutionLarge)
{
	createMap(MapTypeLargeImpossible, 1000);
	pathfinder::Vec2 start(0, m_mapHeight / 2);
	pathfinder::Vec2 goal(m_mapWidth - 1, m_mapHeight / 2);
	int numSteps = findPath(start, goal);
	EXPECT_EQ(-1, numSteps);
}

TEST_F(PathfinderTester, SingleLaneMap)
{
	createMap(MapTypeSingleLane, m_mapWidth);
	int numSteps = findPath(pathfinder::Vec2(0, 0), pathfinder::Vec2(m_mapWidth - 1, 0));
	EXPECT_FALSE(numSteps != -1);
}

TEST_F(PathfinderTester, LoopAllowedLength)
{
	pathfinder::Vec2 start(0, m_mapHeight / 2);
	pathfinder::Vec2 goal(m_mapWidth - 1, m_mapHeight / 2);
	for (int i = 0; i < m_mapWidth; ++i)
	{
		createMap(MapTypeSimple, i);
		int numSteps = findPath(start, goal);
		EXPECT_EQ(goal.x - start.x > i, numSteps == -1);
	}
}