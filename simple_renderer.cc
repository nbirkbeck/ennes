#include <nimage/image.h>
#include "proto/render_state.pb.h"
#include <iostream>
#include <string>

const int kNesWidth = 256;
const int kNesHeight = 240;
const int kNumSprites = 64;

uint8_t kNesPalette[64][3] =
{
   {0x80,0x80,0x80}, {0x00,0x00,0xBB}, {0x37,0x00,0xBF}, {0x84,0x00,0xA6},
   {0xBB,0x00,0x6A}, {0xB7,0x00,0x1E}, {0xB3,0x00,0x00}, {0x91,0x26,0x00},
   {0x7B,0x2B,0x00}, {0x00,0x3E,0x00}, {0x00,0x48,0x0D}, {0x00,0x3C,0x22},
   {0x00,0x2F,0x66}, {0x00,0x00,0x00}, {0x05,0x05,0x05}, {0x05,0x05,0x05},

   {0xC8,0xC8,0xC8}, {0x00,0x59,0xFF}, {0x44,0x3C,0xFF}, {0xB7,0x33,0xCC},
   {0xFF,0x33,0xAA}, {0xFF,0x37,0x5E}, {0xFF,0x37,0x1A}, {0xD5,0x4B,0x00},
   {0xC4,0x62,0x00}, {0x3C,0x7B,0x00}, {0x1E,0x84,0x15}, {0x00,0x95,0x66},
   {0x00,0x84,0xC4}, {0x11,0x11,0x11}, {0x09,0x09,0x09}, {0x09,0x09,0x09},

   {0xFF,0xFF,0xFF}, {0x00,0x95,0xFF}, {0x6F,0x84,0xFF}, {0xD5,0x6F,0xFF},
   {0xFF,0x77,0xCC}, {0xFF,0x6F,0x99}, {0xFF,0x7B,0x59}, {0xFF,0x91,0x5F},
   {0xFF,0xA2,0x33}, {0xA6,0xBF,0x00}, {0x51,0xD9,0x6A}, {0x4D,0xD5,0xAE},
   {0x00,0xD9,0xFF}, {0x66,0x66,0x66}, {0x0D,0x0D,0x0D}, {0x0D,0x0D,0x0D},

   {0xFF,0xFF,0xFF}, {0x84,0xBF,0xFF}, {0xBB,0xBB,0xFF}, {0xD0,0xBB,0xFF},
   {0xFF,0xBF,0xEA}, {0xFF,0xBF,0xCC}, {0xFF,0xC4,0xB7}, {0xFF,0xCC,0xAE},
   {0xFF,0xD9,0xA2}, {0xCC,0xE1,0x99}, {0xAE,0xEE,0xB7}, {0xAA,0xF7,0xEE},
   {0xB3,0xEE,0xFF}, {0xDD,0xDD,0xDD}, {0x11,0x11,0x11}, {0x11,0x11,0x11}
};

struct Sprite {
  uint8_t y_minus_1;
  uint8_t pattern;
  uint8_t att; // upper_color:2, below_background:bit5
  uint8_t x;
};

enum TransformFlags {
   FLIP_NONE = 0,
   FLIP_HORIZONTAL = 1,
   FLIP_VERTICAL = 2,
};

void DrawPattern(nacb::Image8& image, int x, int y,
                 const std::string& palette,
                 const std::string& pattern_table,
                 uint8_t entry, int color, TransformFlags flags = FLIP_NONE) {
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

nacb::Image8 RenderFrame(const nes::RenderSequence::FrameState& frame_state) {
  nacb::Image8 image(kNesWidth, kNesHeight, 3);
  nes::RenderState render_state = frame_state.start_frame();

  for (int y = 0; y < image.h; ++y) {
    for (int x = 0; x < image.w; ++x) {
      for (int c = 0; c < 3; ++c) {
        image(x, y, c) = kNesPalette[render_state.image_palette()[0]][c];
      }
    }
  }
  // FIXME: This is a hack for SMB1.
  render_state.mutable_ppu()->set_name_table(0);

  // TODO: Render background sprites
  
  int next_update_index = 0;
  const nes::RenderState* current_state = &render_state;
  for (int y = 0; y < kNesHeight; y += 8) {
    if (next_update_index < frame_state.state_update_size() &&
        y + 4 >= frame_state.state_update(next_update_index).ppu().scan_line()) {
      current_state = &frame_state.state_update(next_update_index);
      next_update_index++;
    }

    // Currently only supports horizontal scrolling
    {
      const std::string& name_table =  render_state.name_table(current_state->ppu().name_table());
      for (int x = 8 * (current_state->ppu().vscroll() / 8); x < kNesWidth; x += 8) {
        const int high_color_bits = GetAttributeColor(name_table, x, y);
        DrawPattern(image, x - current_state->ppu().vscroll(), y,
                    render_state.image_palette(),
                    render_state.pattern_table(current_state->ppu().screen_pattern_table()),
                    name_table[(y >> 3) * (kNesWidth >> 3) + (x >> 3)], high_color_bits);
      }
    }
    {
      const std::string& name_table =  render_state.name_table((current_state->ppu().name_table() + 1) % 4);
      for (int x = 0; x <= 8 * (current_state->ppu().vscroll() / 8); x += 8) {
        const int high_color_bits = GetAttributeColor(name_table, x, y);
        DrawPattern(image, (x - current_state->ppu().vscroll()) + kNesWidth, y,
                    render_state.image_palette(),
                    render_state.pattern_table(current_state->ppu().screen_pattern_table()),
                    name_table[(y >> 3) * (kNesWidth >> 3) + (x >> 3)], high_color_bits);
      }
    }
  }

  // Render the foreground sprites
  Sprite* sprites = (Sprite*)render_state.sprite_data().c_str();
  for (int i = 0; i < kNumSprites; ++i) {
    if (sprites[i].y_minus_1 < 248) {
      std::cout << int(sprites[i].y_minus_1) << " " << int(sprites[i].x) << std::endl;
      DrawPattern(image, sprites[i].x, sprites[i].y_minus_1 + 1,
                  render_state.sprite_palette(),
                  render_state.pattern_table(render_state.ppu().sprite_pattern_table()),
                  sprites[i].pattern, sprites[i].att & 0x3, TransformFlags(sprites[i].att >> 6));
    }
  }

  
  return image;
}

int main(int ac, char* av[]) {
  nes::RenderSequence seq;
  FILE* file= fopen(av[1], "r");
  if (!file) {
    std::cout << "Unable to open " << av[1] << std::endl;
    return -1;
  }
  
  fseek(file, 0, SEEK_END);
  int size = ftell(file);
  fseek(file, 0, SEEK_SET);
  std::cout << "Size:" << size << std::endl;
  
  std::string bytes(size, 0);
  fread(&bytes[0], size, 1, file);
  seq.ParseFromString(bytes);
  std::cout << "Num frames:" <<  seq.frame_state_size() << std::endl;

  int offset = ac > 2 ? atoi(av[2]) : 0;
  for (int i = 0; i < 300; ++i) {
    nacb::Image8 image = RenderFrame(seq.frame_state(i));
    char filename[1024];
    snprintf(filename, sizeof(filename), "/tmp/rendered-%04d.png", i + offset);
    image.write(filename);
  }
  return 0;
}
