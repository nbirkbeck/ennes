#include "render_utils.h"
#include "sprite.h"

#include <string>
#include <nimage/image.h>

void DrawPattern(nacb::Image8& image, int x, int y,
                 const std::string& palette,
                 const std::string& pattern_table,
                 uint8_t entry, int color, TransformFlags flags) {
  const int index = 8 * entry;
  for (int yi = 0; yi < 8; ++yi) {
    if (y + yi < 0) continue;
    if (y + yi >= image.h) break;
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

void RenderSprites(const nes::RenderState& render_state, int x_offset,
                   bool foreground,
                   nacb::Image8* image_ptr) {
  nacb::Image8& image = *image_ptr;
  Sprite* sprites = (Sprite*)render_state.sprite_data().c_str();
  assert(render_state.sprite_data().size() == 4 * kNumSprites);
  for (int i = 0; i < kNumSprites; ++i) {
    if (sprites[i].IsActive() &&
        (sprites[i].IsForeground() == foreground)) {
      DrawPattern(image, sprites[i].x + x_offset, sprites[i].y(),
                  render_state.sprite_palette(),
                  render_state.pattern_table(render_state.ppu().sprite_pattern_table()),
                  sprites[i].pattern, sprites[i].HighColorBits(), sprites[i].GetTransformFlags());
    }
  }
}

std::map<int, int>
RenderBackground(const nes::RenderSequence::FrameState& frame_state,
                      int x_offset, nacb::Image8* image_ptr) {
  nacb::Image8& image = *image_ptr;
  nes::RenderState render_state = frame_state.start_frame();
  const nes::RenderState* current_state = &render_state;
  int next_update_index = 0;

  std::map<int, int> line_starts;
  for (int y = 0; y < kNesHeight; y += 8) {
    while (next_update_index < frame_state.state_update_size() &&
           y + 4 >= frame_state.state_update(next_update_index).ppu().scan_line()) {
      current_state = &frame_state.state_update(next_update_index);
      next_update_index++;
    }
    const int line_start = (x_offset - current_state->ppu().vscroll() % 8);
    if (line_starts.empty() || line_starts.lower_bound(y)->second != line_start) {
      line_starts[y] = line_start;
    }
    // Currently only supports horizontal scrolling
    {
      const std::string& name_table =  render_state.name_table(current_state->ppu().name_table());
      for (int x = 8 * (current_state->ppu().vscroll() / 8); x < kNesWidth; x += 8) {
        const int high_color_bits = GetAttributeColor(name_table, x, y);
        DrawPattern(image, x - current_state->ppu().vscroll() + x_offset, y,
                    render_state.image_palette(),
                    render_state.pattern_table(current_state->ppu().screen_pattern_table()),
                    name_table[(y >> 3) * (kNesWidth >> 3) + (x >> 3)], high_color_bits);
      }
    }
    {
      const std::string& name_table =  render_state.name_table((current_state->ppu().name_table() + 1) % 4);
      for (int x = 0; x <= 8 * (current_state->ppu().vscroll() / 8); x += 8) {
        const int high_color_bits = GetAttributeColor(name_table, x, y);
        DrawPattern(image, (x - current_state->ppu().vscroll()) + kNesWidth + x_offset, y,
                    render_state.image_palette(),
                    render_state.pattern_table(current_state->ppu().screen_pattern_table()),
                    name_table[(y >> 3) * (kNesWidth >> 3) + (x >> 3)], high_color_bits);
      }
    }
  }
  return line_starts;
}

bool LoadRenderSequence(const std::string& filename,
                        nes::RenderSequence* seq) {

  seq->Clear();
  FILE* file= fopen(filename.c_str(), "r");
  if (!file) {
    std::cout << "Unable to open " << filename << std::endl;
    return false;
  }
  
  fseek(file, 0, SEEK_END);
  const int size = ftell(file);
  fseek(file, 0, SEEK_SET);
  
  std::string bytes(size, 0);
  fread(&bytes[0], size, 1, file);
  fclose(file);
  seq->ParseFromString(bytes);
  std::cout << "Num frames:" <<  seq->frame_state_size() << std::endl;
  return true;
}
