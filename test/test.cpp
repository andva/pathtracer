#include <gtest/gtest.h>
#include <pathfinder/pathfinder.h>

TEST(TestFunction, HelloTestWorld) {
	bool b = (0 == TestFunction(0));
	EXPECT_TRUE(b);
}