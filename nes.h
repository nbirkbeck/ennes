#ifndef _NES_H_
#define _NES_H_ 1

#include <nmath/vec3.h>
#include <stdint.h>

static constexpr int kNesWidth = 256;
static constexpr int kNesHeight = 240;
static constexpr int kNumSprites = 64;
static constexpr int kNesBlocksWidth = kNesWidth / 8;
static constexpr int kNesBlocksHeight = kNesHeight / 8;
static constexpr int kBlockSize = 8;

typedef nacb::Vec3<uint8_t> Color3b;

extern Color3b kNesPalette[64];

enum TransformFlags {
   FLIP_NONE = 0,
   FLIP_HORIZONTAL = 1,
   FLIP_VERTICAL = 2,
};

#endif
