#ifndef _NES_SPRITE_
#define _NES_SPRITE_ 1

#include "proto/render_state.pb.h"
#include "nes.h"

#include <nimage/image.h>
#include <vector>

struct Sprite {
public:
  bool IsActive() const {
    return y_minus_1 < 248;
  }

  int HighColorBits() const {
    return att & 0x3;
  }

  TransformFlags GetTransformFlags() const {
    return TransformFlags(att >> 6);
  }

  const int y() const {
    return y_minus_1 + 1;
  }
  
  uint8_t y_minus_1;
  uint8_t pattern;
  uint8_t att; // upper_color:2, below_background:bit5
  uint8_t x;
};

struct SpriteGroup {
  std::vector<int> indices;
  nacb::Image8 image;
};

std::vector<SpriteGroup>
GroupSprites(const Sprite* sprites, const nes::RenderState& render_state);


#endif 
