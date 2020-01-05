#include <map>
#include <vector>

#include "nes.h"
#include <nimage/image.h>

struct BackgroundGroup {
  BackgroundGroup(const std::vector<std::pair<int, int>>& b, bool walkable_arg)
      : blocks(b), walkable(walkable_arg) {
    ComputeBounds();
  }

  nacb::Image8 ExtractImage(const nacb::Image8& image,
                            const std::map<int, int>& line_starts) const {
    int x0 = line_starts.lower_bound(8 * min_y)->second;
    int w = (max_x + 1 - min_x) * 8;
    int h = (max_y + 1 - min_y) * 8;
    const int x1 = (x0 + min_x * 8) + w;
    const int y1 = min_y * 8 + h;
    if (x1 > image.w) {
      std::cerr << "x coordinate out of bounds, x0 = " << x0
                << " min_x = " << min_x << " max_x = " << max_x
                << " image.w = " << image.w;
    }
    if (y1 > image.h) {
      std::cerr << "y coordinate out of bounds, y = "
                << " min_y = " << min_y << " max_y = " << max_y
                << " image.h = " << image.h;
    }
    return image.subimage(x0 + min_x * 8, min_y * 8, (max_x + 1 - min_x) * 8,
                          (max_y + 1 - min_y) * 8);
  }

  void ComputeBounds() {
    min_x = kNesWidth;
    min_y = kNesHeight;
    max_x = 0;
    max_y = 0;
    for (const auto& b : blocks) {
      min_x = std::min(min_x, b.first);
      min_y = std::min(min_y, b.second);

      max_x = std::max(max_x, b.first);
      max_y = std::max(max_y, b.second);
    }
  }
  int min_x, min_y;
  int max_x, max_y;
  const std::vector<std::pair<int, int>> blocks;
  const bool walkable;
};

bool IsEdgeBackground(const nacb::Image8& im, int x, int y,
                      const Color3b& bg_color, int dx, int dy);

std::vector<BackgroundGroup>
FindBackgroundGroups(nacb::Image8* image, const Color3b& bg_color,
                     const std::map<int, int>& line_starts);

void SegmentIntoCubes(const BackgroundGroup& group,
                      std::vector<BackgroundGroup>* cubes);
