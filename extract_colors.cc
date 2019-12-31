#include <nimage/image.h>
#include <nmath/vec3.h>
#include <unordered_map>
#include <stdint.h>
#include <set>

namespace {
int ColorIndex(uint8_t r, uint8_t g, uint8_t b) {
  return (int(r) << 16) | (int(g) << 8) | int(b);
}
}

std::vector<std::pair<nacb::Vec3f, nacb::Image8>> ExtractColorMasks(const nacb::Image8& image) {
  std::set<int> colors;
  for (int y = 0; y < image.h; ++y) {
    for (int x = 0; x < image.w; ++x) {
      int color = ColorIndex(image(x, y, 0), image(x, y, 1), image(x, y, 2));
      if (colors.count(color) == 0) {
        colors.insert(color);
      }
    }
  }
  printf("Number of colors: %d\n", colors.size());

  std::vector<std::pair<nacb::Vec3f, nacb::Image8>> color_images;
  for (const auto& color: colors) {
    nacb::Image8 mask(image.w, image.h, 3);
    mask = 0;
    for (int y = 0; y < image.h; ++y) {
      for (int x = 0; x < image.w; ++x) {
        const int on = (ColorIndex(image(x, y, 0), image(x, y, 1), image(x, y, 2)) == color) ? 255: 0;
        mask(x, y, 0) = mask(x, y, 1) = mask(x, y, 2) = on;
      }
    }
    nacb::Vec3f color_vec(float((color >> 16) & 0xFF) / 255.0f,
                          float((color >> 8) & 0xFF) / 255.0f,
                          float(color & 0xFF) / 255.0f);
    color_images.push_back(std::make_pair(color_vec, mask));
  }
  return color_images;
}

#ifdef EXTRACT_COLORS_MAIN

int main(int ac, char* av[]) {
  nacb::Image8 image(av[1]);
  if (ac < 3) {
    fprintf(stderr, "Need 2 input argument, the input image and output path\n");
    return -1;
  }
  int output_index =  0;
  for (const auto& color_image : ExtractColorMasks(image)) {
    char output[1024];
    snprintf(output, sizeof(output), av[2], output_index);
    mask.write(output);
    output_index++;    
  }
  return 0;
}

#endif
