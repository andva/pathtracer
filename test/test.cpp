#include <gtest/gtest.h>
#include <pathfinder/pathfinder.h>

class PathfinderTester : public ::testing::Test {
protected:
	virtual void SetUp() {
		nMapWidth = 3;
		nMapHeight = 3;
		sSearchSpace = new pathfinder::searchSpace(nMapWidth, nMapHeight, pMap);
	}

	virtual void TearDown() {
		delete sSearchSpace;
	}
	unsigned int* pMap;
	pathfinder::vec2 sStart;
	pathfinder::vec2 sGoal;
	int nMapWidth;
	int nMapHeight;
	pathfinder::searchSpace* sSearchSpace;
};

TEST_F(PathfinderTester, ManhattanDistance)
{
	EXPECT_EQ(0, pathfinder::distManhattan(sStart, sGoal));
	EXPECT_EQ(2, pathfinder::distManhattan(pathfinder::vec2(), pathfinder::vec2(1,1)));
}

TEST_F(PathfinderTester, AddNeighbouringNodes)
{
	pathfinder::node n(new pathfinder::vec2(1,1), 0, 0, nullptr);
	pathfinder::vec2 start(1,1);
	pathfinder::vec2 goal(2,0);
	EXPECT_TRUE(sSearchSpace->insertInitialNodes(start, goal));
	EXPECT_FALSE(sSearchSpace->update());
	EXPECT_EQ(0, sSearchSpace->m_vNodeVector.size());
	EXPECT_EQ(1, sSearchSpace->m_visitedNodes.size());
	sSearchSpace->addNeighbouringNodes();
	
	EXPECT_EQ(4, sSearchSpace->m_vNodeVector.size());
	EXPECT_FALSE(sSearchSpace->update());

	sSearchSpace->addNeighbouringNodes();
	EXPECT_TRUE(sSearchSpace->update());
}