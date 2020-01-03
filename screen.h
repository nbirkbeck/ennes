#include <vector>
#include <nimage/image.h>

struct BackgroundGroup {
  int min_x, min_y;
  int max_x, max_y;
  std::vector<std::pair<int, int>> blocks;
  bool walkable;

  nacb::Image8 ExtractImage(const nacb::Image8& image, int x0) const {
    int w = (max_x + 1 - min_x) * 8;
    int h = (max_y + 1 - min_y) * 8;
    const int x1 = (x0 + min_x * 8) + w;
    const int y1 = min_y * 8 + h;
    if (x1 > image.w) {
      std::cerr << "x coordinate out of bounds, x0 = "
                << x0 << " min_x = " << min_x
                << " max_x = " << max_x <<  " image.w = " << image.w;
    }
    if (y1 > image.h) {
      std::cerr << "y coordinate out of bounds, y = "
                << " min_y = " << min_y
                << " max_y = " << max_y <<  " image.h = " << image.h;
    }
    return image.subimage(x0 + min_x * 8, min_y * 8,
                          (max_x + 1 - min_x) * 8,
                          (max_y + 1 - min_y) * 8);
  }
};


std::vector<BackgroundGroup>
FindBackgroundGroups(nacb::Image8* image, const uint8_t bg[3], int x0);
