#include <nimage/image.h>
#include <nmath/vec3.h>
#include <unordered_map>
#include <stdint.h>
#include <set>

namespace {
int ColorIndex(uint8_t r, uint8_t g, uint8_t b) {
  return (int(r) << 16) | (int(g) << 8) | int(b);
}

nacb::Vec3f IndexToColor(int color) {
  return nacb::Vec3f(float((color >> 16) & 0xFF) / 255.0f,
                     float((color >> 8) & 0xFF) / 255.0f,
                     float(color & 0xFF) / 255.0f);
}

std::set<int> GetUniqueColors(const nacb::Image8& image) {
  std::set<int> colors;
  for (int y = 0; y < image.h; ++y) {
    for (int x = 0; x < image.w; ++x) {
      int color = ColorIndex(image(x, y, 0), image(x, y, 1), image(x, y, 2));
      if (colors.count(color) == 0) {
        colors.insert(color);
      }
    }
  }
  return colors;
}
}

std::vector<std::pair<nacb::Vec3f, nacb::Image8>> ExtractColorMasks(const nacb::Image8& image) {
  std::set<int> colors = GetUniqueColors(image);
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
    color_images.push_back(std::make_pair(IndexToColor(color), mask));
  }
  return color_images;
}

std::pair<nacb::Image8, std::vector<nacb::Vec3f> >
PalettizeImage(const nacb::Image8& image) {
  std::set<int> unique_colors = GetUniqueColors(image);
  std::unordered_map<int, int> color_index;
  std::vector<nacb::Vec3f> colors;
  for (int c : unique_colors) {
    color_index[c] = colors.size();
    colors.push_back(IndexToColor(c));
  }
  nacb::Image8 indexed_image(image.w, image.h, 1);
  for (int y = 0; y < image.h; ++y) {
    for (int x = 0; x < image.w; ++x) {
      indexed_image(x, y, 0) = color_index[ColorIndex(image(x, y, 0), image(x, y, 1), image(x, y, 2))];
    }
  }
  return std::make_pair(indexed_image, colors);
}

#ifdef EXTRACT_COLORS_MAIN

int main(int ac, char* av[]) {
  nacb::Image8 image(av[1]);
  if (ac < 3) {
    fprintf(stderr, "Need 2 input argument, the input image and output path\n");
    return -1;
  }
  int output_index =  0;
  for (auto& color_image : ExtractColorMasks(image)) {
    char output[1024];
    snprintf(output, sizeof(output), av[2], output_index);
    color_image.second.write(output);
    output_index++;    
  }
  return 0;
}

#endif
