#ifndef _NES_SPRITE_
#define _NES_SPRITE_ 1

#include "nes.h"
#include "proto/render_state.pb.h"

#include <nimage/image.h>
#include <vector>

struct Sprite {
public:
  bool IsActive() const { return y_minus_1 < 248; }

  int HighColorBits() const { return att & 0x3; }

  TransformFlags GetTransformFlags() const { return TransformFlags(att >> 6); }

  const int y() const { return y_minus_1 + 1; }

  bool IsForeground() const { return !((att >> 5) & 0x1); }
  uint8_t y_minus_1;
  uint8_t pattern;
  uint8_t att; // upper_color:2, below_background:bit5
  uint8_t x;
};

struct SpriteGroup {
  int x, y;
  std::vector<int> indices;
  nacb::Image8 image;
};

std::vector<SpriteGroup> GroupSprites(const Sprite* sprites,
                                      const nes::RenderState& render_state);

template <class T> class SpriteDatabase {
public:
  bool Exists(const nacb::Image8& image) const {
    std::string str(reinterpret_cast<const char*>(image.data),
                    image.w * image.h * 3);
    return database_.count(str);
  }
  void Insert(const nacb::Image8& image, T t) {
    std::string str(reinterpret_cast<const char*>(image.data),
                    image.w * image.h * 3);
    database_[str] = image;
    secondary_data_[str] = std::move(t);
  }
  T& Lookup(const nacb::Image8& image) {
    std::string str(reinterpret_cast<const char*>(image.data),
                    image.w * image.h * 3);
    return secondary_data_[str];
  }

  std::unordered_map<std::string, nacb::Image8>::iterator begin() {
    return database_.begin();
  }
  std::unordered_map<std::string, nacb::Image8>::iterator end() {
    return database_.end();
  }

private:
  std::unordered_map<std::string, nacb::Image8> database_;
  std::unordered_map<std::string, T> secondary_data_;
};

#endif
