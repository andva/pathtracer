#include <gtest/gtest.h>
#include <pathfinder/pathfinder.h>

class PathfinderTester : public ::testing::Test {
protected:
	virtual void SetUp() {

	}

	virtual void TearDown() {

	}

};

TEST(PathfinderTester, HelloTestWorld) {
	bool b = (0 == pathfinder::TestFunction(0));
	EXPECT_TRUE(b);
}