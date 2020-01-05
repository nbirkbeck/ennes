#include "gtest/gtest.h"

#include "extract_colors.h"
#include <nimage/image.h>
#include <nmath/vec3.h>

template <> void nacb::Vec3<uint8_t>::print(std::ostream& o) const {
  o << int(x) << "," << int(y) << "," << int(z);
}

TEST(ColorIndexTest, TestColors) {
  EXPECT_EQ(0, ColorIndex(0, 0, 0));
  EXPECT_EQ(255, ColorIndex(0, 0, 255));
  EXPECT_EQ(256, ColorIndex(0, 1, 0));
  EXPECT_EQ(1, ColorIndex(0, 0, 1));
  EXPECT_EQ(0xABCDEF, ColorIndex(0xAB, 0xCD, 0xEF));
  EXPECT_EQ(0xFFFFFF, ColorIndex(0xFF, 0xFF, 0xFF));
}

TEST(NumOverlappingColors, NothingCommon) {
  EXPECT_EQ(0, NumOverlappingColors({}, {}));
}

TEST(NumOverlappingColors, OneInCommon) {
  EXPECT_EQ(1, NumOverlappingColors({1, 2, 3}, {3, 4, 5}));
}

TEST(GetUniqueColors, EmptyImage) {
  nacb::Image8 image(1, 1, 3);
  image = 0;
  EXPECT_EQ(std::set<int>{0}, GetUniqueColors(image));
}

TEST(GetUniqueColors, FourColors) {
  nacb::Image8 image(2, 2, 3);
  image = 0;
  image(0, 1, 0) = 255;
  image(1, 0, 1) = 255;
  image(1, 1, 2) = 255;
  EXPECT_EQ(std::set<int>({00, 0xFF, 0xFF00, 0xFF0000}),
            GetUniqueColors(image));
}

TEST(GetUniqueColors, FourColorsSubImage) {
  nacb::Image8 image(2, 2, 3);
  image = 0;
  image(0, 1, 0) = 255;
  image(1, 0, 1) = 255;
  image(1, 1, 2) = 255;
  EXPECT_EQ(std::set<int>({00}), GetUniqueColors(image, 0, 0, 1, 1));

  EXPECT_EQ(std::set<int>({0xFF}), GetUniqueColors(image, 1, 1, 1, 1));
}

TEST(ExtractColorMasks, ColorMasks) {
  nacb::Image8 image(2, 2, 3);
  image = 0;
  image(0, 1, 0) = 255;
  image(1, 0, 1) = 255;
  image(1, 1, 2) = 255;
  auto masks = ExtractColorMasks(image);
  ASSERT_EQ(4, masks.size());
  EXPECT_EQ(Color3b(0, 0, 0), masks[0].first);
  EXPECT_EQ(Color3b(0, 0, 255), masks[1].first);
  EXPECT_EQ(Color3b(0, 255, 0), masks[2].first);
  EXPECT_EQ(Color3b(255, 0, 0), masks[3].first);
}

TEST(PalettizeImage, EmptyImage) {
  nacb::Image8 image(2, 2, 3);
  image = 0;
  auto result = PalettizeImage(image);
  ASSERT_EQ(1, result.second.size());
  EXPECT_EQ(1, result.first.nchannels);
  EXPECT_EQ(2, result.first.w);
  EXPECT_EQ(2, result.first.h);
  for (int y = 0; y < 2; ++y) {
    for (int x = 0; x < 2; ++x) {
      EXPECT_EQ(0, result.first(x, y, 0));
    }
  }
}

TEST(PalettizeImage, TwoColors) {
  nacb::Image8 image(2, 2, 3);
  image = 0;
  image(0, 0, 2) = 255;
  image(0, 1, 0) = 255;
  image(1, 0, 0) = 255;
  image(1, 1, 2) = 255;
  auto result = PalettizeImage(image);
  ASSERT_EQ(2, result.second.size());
  EXPECT_EQ(1, result.first.nchannels);
  EXPECT_EQ(2, result.first.w);
  EXPECT_EQ(2, result.first.h);
  EXPECT_EQ(0, result.first(0, 0));
  EXPECT_EQ(0, result.first(1, 1));
  EXPECT_EQ(1, result.first(0, 1));
  EXPECT_EQ(1, result.first(1, 0));
}

int main(int ac, char* av[]) {
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
