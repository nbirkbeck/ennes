#include <vector>
#include <algorithm>
#include <set>

#include <nimage/image.h>
#include <nmath/vec3.h>

inline int ColorIndex(uint8_t r, uint8_t g, uint8_t b) {
  return (int(r) << 16) | (int(g) << 8) | int(b);
}

int NumOverlappingColors(const std::set<int>& s1,
                         const std::set<int>& s2);

std::set<int> GetUniqueColors(const nacb::Image8& image,
                              int x0 = 0, int y0 = 0, int w = -1, int h = -1);

std::vector<std::pair<nacb::Vec3f, nacb::Image8>>
  ExtractColorMasks(const nacb::Image8& image);

std::pair<nacb::Image8, std::vector<nacb::Vec3f> >
 PalettizeImage(const nacb::Image8& image);
