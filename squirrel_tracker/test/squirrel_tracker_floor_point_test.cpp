#include "squirrel_tracker/floor_pointer.h"
#include "gtest/gtest.h"

TEST(FloorPointerTest, testIsGestureReliable)
{
  FloorPointer floorpointer;

  EXPECT_FLOAT_EQ(false, floorpointer.isAverage(FloorPointer::NONE));
  EXPECT_FLOAT_EQ(false, floorpointer.isAverage(FloorPointer::LEFT));
  EXPECT_FLOAT_EQ(false, floorpointer.isAverage(FloorPointer::RIGHT));

  for (int i = 0; i < 10; ++i)
  {
    nite::Point3f testPoint(i, i + 1, i + 2);
    floorpointer.leftHandPositionHistory.push_back(testPoint);
    floorpointer.rightHandPositionHistory.push_back(testPoint);
  }

  EXPECT_FLOAT_EQ(false, floorpointer.isAverage(FloorPointer::NONE));
  EXPECT_FLOAT_EQ(true, floorpointer.isAverage(FloorPointer::LEFT));
  EXPECT_FLOAT_EQ(true, floorpointer.isAverage(FloorPointer::RIGHT));
  EXPECT_FLOAT_EQ(4.5,floorpointer.average[0]);
  EXPECT_FLOAT_EQ(5.5,floorpointer.average[1]);
  EXPECT_FLOAT_EQ(6.5,floorpointer.average[2]);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
