#include "gtest/gtest.h"

#include "highres_boundary.h"

TEST(HighresBoundaryTest, MaskUpsample2) {
  nacb::Image8 input(4, 4, 1);
  input = 0;
  input(1, 1) = 255;
  input(1, 2) = 255;
  input(2, 1) = 255;
  input(2, 2) = 255;

  nacb::Imagef result = HighresBoundarySimple(input, 2);
  ASSERT_EQ(8, result.w);
  ASSERT_EQ(8, result.h);
  ASSERT_EQ(1, result.nchannels);

  for (int x = 0; x < 2; ++x) {
    for (int y = 0; y < result.h; ++y) {
      EXPECT_LE(result(x, y), 0.5);
      EXPECT_LE(result(result.w - x - 1, y), 0.5);
    }
  }

  for (int y = 0; y < 2; ++y) {
    for (int x = 0; x < result.w; ++x) {
      EXPECT_LE(result(x, y), 0.5);
      EXPECT_LE(result(x, result.h - 1 - y), 0.5);
    }
  }
  for (int y = 2; y < result.h - 2; ++y) {
    for (int x = 2; x < result.w - 2; ++x) {
      EXPECT_GE(result(x, y), 0.5);
    }
  }
}

TEST(HighresBoundaryTest, ColorUpsample) {
  nacb::Image8 input(3, 4, 3);
  input = 0;
  input(1, 1, 0) = 255;
  input(1, 1, 1) = 0;
  input(1, 1, 2) = 0;

  input(1, 2, 0) = 0;
  input(1, 2, 1) = 255;
  input(1, 2, 2) = 0;
  Color3b bg(0, 0, 0);
  nacb::Image8 result =
      HighresBoundaryColor(input, HighresBoundarySimple, 2, &bg);
  ASSERT_EQ(6, result.w);
  ASSERT_EQ(8, result.h);
  ASSERT_EQ(4, result.nchannels);

  for (int x = 0; x < 3; ++x) {
    for (int y = 1; y < 7; ++y) {
      EXPECT_EQ(y < 4 ? 255 : 0, result(x, y, 0)) << x << "," << y;
      EXPECT_EQ(y < 4 ? 0 : 255, result(x, y, 1)) << x << "," << y;
    }
    // Outside pixels shouldn't be black--should be close color.
    EXPECT_EQ(255, result(x, 0, 0) | result(x, 0, 1));
    EXPECT_EQ(255, result(x, 7, 0) | result(x, 7, 1));
    EXPECT_LE(result(x, 0, 3), 10);
    EXPECT_LE(result(x, 7, 3), 10);
  }
}

int main(int ac, char* av[]) {
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
