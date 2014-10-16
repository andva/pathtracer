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
		MapTypeLargeImpossible = 1 << 3,
		MapTypeSmallDiagonal = 1 << 4,
	};

	void createMap(unsigned int mapType)
	{
		if (mapType == MapTypeNone)
		{
			nMapWidth = 0;
			nMapHeight = 0;
			pMap = new unsigned char[0];
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
			pMap = new unsigned char[mapSize];
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
			pMap = new unsigned char[mapSize];
			for (int y = 0; y < nMapHeight; ++y)
			for (int x = 0; x < nMapWidth; ++x)
			{
				pMap[x + y * nMapWidth] = (x == 2) ? 1 : 0;
			}
		}
		else if (mapType == MapTypeLargeImpossible)
		{
			nMapWidth = 500;
			nMapHeight = 500;
			unsigned int mapSize = nMapWidth * nMapHeight;
			pMap = new unsigned char[mapSize];
			for (int y = 0; y < nMapHeight; ++y)
			for (int x = 0; x < nMapWidth; ++x)
			{
				pMap[x + y * nMapWidth] = (x == nMapWidth - 3) ? 1 : 0;
			}
		}
		else if (mapType == MapTypeSmallDiagonal)
		{
			nMapWidth = 10;
			nMapHeight = 10;
			unsigned int mapSize = nMapWidth * nMapHeight;
			pMap = new unsigned char[mapSize];
			for (int y = 0; y < nMapHeight; ++y)
			for (int x = 0; x < nMapWidth; ++x)
			{
				pMap[x + y * nMapWidth] = (x == y) ? 1 : 0;
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
	unsigned char* pMap;
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
	bool solutionState = false;
	EXPECT_TRUE(pSearchSpace->insertInitialNodes(start, goal));
	EXPECT_FALSE(pSearchSpace->update(solutionState));
}

TEST_F(PathfinderTester, StartAndGoalTests)
{
	createMap(MapTypeSimple);
	pathfinder::vec2 p1(1, 1);
	pathfinder::vec2 p2(-1, 1);
	pathfinder::vec2 p3(nMapWidth, 1);
	pathfinder::vec2 p4(1, nMapHeight);
	bool solutionState = false;
	EXPECT_FALSE(pSearchSpace->insertInitialNodes(p1, p2));
	EXPECT_TRUE(pSearchSpace->update(solutionState));
	EXPECT_FALSE(solutionState);

	EXPECT_FALSE(pSearchSpace->insertInitialNodes(p3, p2));
	EXPECT_TRUE(pSearchSpace->update(solutionState));
	EXPECT_FALSE(solutionState);

	EXPECT_FALSE(pSearchSpace->insertInitialNodes(p4, p2));
	EXPECT_TRUE(pSearchSpace->update(solutionState));
	EXPECT_FALSE(solutionState);

	EXPECT_FALSE(pSearchSpace->insertInitialNodes(p2, p2));
	EXPECT_TRUE(pSearchSpace->update(solutionState));
	EXPECT_FALSE(solutionState);

	EXPECT_TRUE(pSearchSpace->insertInitialNodes(p1, p1));
	EXPECT_TRUE(pSearchSpace->update(solutionState));
	EXPECT_TRUE(solutionState);
}

TEST_F(PathfinderTester, AddNeighboringNodes)
{
	createMap(MapTypeSimple);
	pathfinder::vec2 start(1, 1);
	pathfinder::vec2 goal(2, 0);
	bool solutionState = false;
	EXPECT_TRUE(pSearchSpace->insertInitialNodes(start, goal));
	EXPECT_FALSE(pSearchSpace->update(solutionState));
	EXPECT_EQ(0, pSearchSpace->m_vNodeVector.size());
	pSearchSpace->addNeighboringNodes();
	
	EXPECT_EQ(4, pSearchSpace->m_vNodeVector.size());
	EXPECT_FALSE(pSearchSpace->update(solutionState));

	pSearchSpace->addNeighboringNodes();
	EXPECT_TRUE(pSearchSpace->update(solutionState));
	EXPECT_TRUE(solutionState);
}

TEST_F(PathfinderTester, NoSolutionSmall)
{
	createMap(MapTypeSmallImpossible);
	bool solutionState = false;
	EXPECT_TRUE(pSearchSpace->insertInitialNodes(pathfinder::vec2(0, 2), pathfinder::vec2(4, 2)));
	while (pSearchSpace->m_vNodeVector.size() > 0) {
		if (pSearchSpace->update(solutionState))
			EXPECT_FALSE(solutionState);
		pSearchSpace->addNeighboringNodes();
	}
}

TEST_F(PathfinderTester, NoSolutionDiagonal)
{
	createMap(MapTypeSmallDiagonal);
	bool solutionState = false;
	EXPECT_TRUE(pSearchSpace->insertInitialNodes(pathfinder::vec2(0, nMapHeight / 2), pathfinder::vec2(nMapWidth - 1, nMapHeight / 2)));
	while (pSearchSpace->m_vNodeVector.size() > 0) {
		if (pSearchSpace->update(solutionState))
			EXPECT_FALSE(solutionState);
		pSearchSpace->addNeighboringNodes();
	}
}

TEST_F(PathfinderTester, NoSolutionLarge)
{
	createMap(MapTypeLargeImpossible);
	bool solutionState = false;
	EXPECT_TRUE(pSearchSpace->insertInitialNodes(pathfinder::vec2(0, nMapHeight / 2), pathfinder::vec2(nMapWidth - 1, nMapHeight / 2)));
	while (pSearchSpace->m_vNodeVector.size() > 0) {
		if (pSearchSpace->update(solutionState))
			EXPECT_FALSE(solutionState);
		pSearchSpace->addNeighboringNodes();
	}
}