#include <gtest/gtest.h>
#include <pathfinder/pathfinder.h>
#include <pathfinder/searchspace.h>

class PathfinderTester : public ::testing::Test 
{
public:
	enum MapType 
	{
		MapTypeNone = 1 << 0,
		MapTypeSimple = 1 << 1,
		MapTypeSmallImpossible = 1 << 2,
	};

	void createMap(unsigned int mapType)
	{
		if (mapType == MapTypeNone)
		{
			nMapWidth = 0;
			nMapHeight = 0;
			pMap = new unsigned int[0];
		}
		else
		{
			delete[] pMap;
		}
		if (mapType == MapTypeSimple)
		{
			nMapWidth = 3;
			nMapHeight = 3;
			unsigned int mapSize = nMapWidth * nMapHeight;
			pMap = new unsigned int[mapSize];
			for (unsigned int i = 0; i < mapSize; ++i)
			{
				pMap[i] = 0;
			}
		}
		else if (mapType == MapTypeSmallImpossible)
		{
			nMapWidth = 5;
			nMapHeight = 5;
			unsigned int mapSize = nMapWidth * nMapHeight;
			pMap = new unsigned int[mapSize];
			for (int y = 0; y < nMapWidth; ++y)
			for (int x = 0; x < nMapHeight; ++x)
			{
				pMap[x + y * nMapWidth] = (x == 2) ? 1 : 0;
			}
		}
		pSearchSpace = new pathfinder::searchSpace(nMapWidth, nMapHeight, &pMap);
	}

protected:
	virtual void SetUp() {
		createMap(MapTypeNone);
	}

	virtual void TearDown() {
		delete[] pMap;
		delete pSearchSpace;
	}
	unsigned int* pMap;
	pathfinder::vec2 sStart;
	pathfinder::vec2 sGoal;
	int nMapWidth;
	int nMapHeight;
	pathfinder::searchSpace* pSearchSpace;
};

TEST_F(PathfinderTester, ManhattanDistance)
{
	EXPECT_EQ(0, pathfinder::distManhattan(sStart, sGoal));
	EXPECT_EQ(2, pathfinder::distManhattan(pathfinder::vec2(), pathfinder::vec2(1,1)));
}

TEST_F(PathfinderTester, InsertInitialNodes)
{
	createMap(MapTypeSimple);
	pathfinder::vec2 start(1, 1);
	pathfinder::vec2 goal(2, 0);
	EXPECT_TRUE(pSearchSpace->insertInitialNodes(start, goal));
	EXPECT_FALSE(pSearchSpace->update());
}

TEST_F(PathfinderTester, StartEqGoal)
{
	createMap(MapTypeSimple);
	pathfinder::vec2 start(1, 1);
	EXPECT_TRUE(pSearchSpace->insertInitialNodes(start, start));
	EXPECT_TRUE(pSearchSpace->update());
}

TEST_F(PathfinderTester, AddNeighbouringNodes)
{
	createMap(MapTypeSimple);
	pathfinder::vec2 start(1, 1);
	pathfinder::vec2 goal(2, 0);
	EXPECT_TRUE(pSearchSpace->insertInitialNodes(start, goal));
	EXPECT_FALSE(pSearchSpace->update());
	EXPECT_EQ(0, pSearchSpace->m_vNodeVector.size());
	EXPECT_EQ(1, pSearchSpace->m_visitedNodes.size());
	pSearchSpace->addNeighbouringNodes();
	
	EXPECT_EQ(4, pSearchSpace->m_vNodeVector.size());
	EXPECT_FALSE(pSearchSpace->update());

	pSearchSpace->addNeighbouringNodes();
	EXPECT_TRUE(pSearchSpace->update());
}

TEST_F(PathfinderTester, NoSolution)
{
	createMap(MapTypeSmallImpossible);
	EXPECT_TRUE(pSearchSpace->insertInitialNodes(pathfinder::vec2(0, 2), pathfinder::vec2(4, 2)));
	EXPECT_FALSE(pSearchSpace->update());
	while (pSearchSpace->m_vNodeVector.size() > 0) {
		EXPECT_FALSE(pSearchSpace->update());
		pSearchSpace->addNeighbouringNodes();
	}
}