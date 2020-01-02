#include "render_utils.h"

#include <string>
#include <nimage/image.h>

void DrawPattern(nacb::Image8& image, int x, int y,
                 const std::string& palette,
                 const std::string& pattern_table,
                 uint8_t entry, int color, TransformFlags flags) {
  const int index = 8 * entry;
  for (int yi = 0; yi < 8; ++yi) {
    const int yi_transformed = (flags & FLIP_VERTICAL) ? (7 - yi) : yi;
    const uint8_t b1 = pattern_table[2 * index + yi_transformed];
    const uint8_t b2 = pattern_table[2 * index + 8 + yi_transformed];
    for (int xi = 0; xi < 8; ++xi) {
      if (x + xi < 0) continue;
      if (x + xi >= image.w) break;
      const int xi_t = (flags & FLIP_HORIZONTAL) ? (7 - xi) : xi;
      const int cindex = ((b1 >> (7 - xi_t)) & 0x1) | (((b2 >> (7 - xi_t)) & 0x1) << 1);
      if (cindex) {
        const int val = (cindex | (color << 2)) & 0xFF;
        image(x + xi, y + yi, 0) = kNesPalette[palette[val]][0];
        image(x + xi, y + yi, 1) = kNesPalette[palette[val]][1];
        image(x + xi, y + yi, 2) = kNesPalette[palette[val]][2];
        if (image.nchannels > 3) {
          image(x + xi, y + yi, 3) = 255;
        }
      }
    }
  }
}

const int GetAttributeColor(const std::string& name_table, int x, int y) {
  const char* attributes = &name_table[0] + (kNesWidth >> 3) * (kNesHeight >> 3);
  const int sy = y >> 4;
  const int sx = x >> 4;
  const uint8_t attribute = attributes[((sy & ~0x1) << 2) + (sx >> 1)];
  return (attribute >> (((sy & 0x1)<< 2) + ((sx & 0x1) << 1))) & 0x3;
}

void ClearImage(nacb::Image8& image, const uint8_t color[3]) {
  for (int y = 0; y < image.h; ++y) {
    for (int x = 0; x < image.w; ++x) {
      for (int c = 0; c < 3; ++c) {
        image(x, y, c) = color[c];
      }
    }
  }
}

