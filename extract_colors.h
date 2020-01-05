#include <algorithm>
#include <set>
#include <vector>

#include <nimage/image.h>
#include <nmath/vec3.h>

#include "nes.h"

inline int ColorIndex(uint8_t r, uint8_t g, uint8_t b) {
  return (int(r) << 16) | (int(g) << 8) | int(b);
}

inline int ColorIndex(const Color3b& color) {
  return ColorIndex(color.x, color.y, color.z);
}

int NumOverlappingColors(const std::set<int>& s1, const std::set<int>& s2);

std::set<int> GetUniqueColors(const nacb::Image8& image, int x0 = 0, int y0 = 0,
                              int w = -1, int h = -1);

std::vector<std::pair<Color3b, nacb::Image8>>
ExtractColorMasks(const nacb::Image8& image);

std::pair<nacb::Image8, std::vector<Color3b>>
PalettizeImage(const nacb::Image8& image);
