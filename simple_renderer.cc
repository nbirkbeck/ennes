#include "proto/render_state.pb.h"

#include "nes.h"
#include "render_utils.h"
#include "sprite.h"

#include <iostream>
#include <string>
#include <queue>

#include <nimage/image.h>

class SpriteDatabase {
public:
  bool Exists(const nacb::Image8& image) const {
    std::string str(reinterpret_cast<const char*>(image.data), image.w * image.h * 3);
    return database_.count(str);
  }
  void Insert(const nacb::Image8& image) {
    std::string str(reinterpret_cast<const char*>(image.data), image.w * image.h * 3);
    database_[str] = image;
  }
   std::unordered_map<std::string, nacb::Image8> database_;
} sprite_database;


nacb::Image8 RenderFrame(const nes::RenderSequence::FrameState& frame_state) {
  nacb::Image8 image(kNesWidth, kNesHeight, 3);
  nes::RenderState render_state = frame_state.start_frame();

  ClearImage(image, kNesPalette[render_state.image_palette()[0]]);

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
    if (sprites[i].IsActive()) {
      DrawPattern(image, sprites[i].x, sprites[i].y(),
                  render_state.sprite_palette(),
                  render_state.pattern_table(render_state.ppu().sprite_pattern_table()),
                  sprites[i].pattern, sprites[i].HighColorBits(), sprites[i].GetTransformFlags());
    }
  }

  std::vector<SpriteGroup> groups = GroupSprites(sprites, render_state);
  for (auto& sprite_group : groups) {
    if (!sprite_database.Exists(sprite_group.image)) {
      sprite_database.Insert(sprite_group.image);
    
      if (sprite_group.indices.size() >= 2) {
        char filename[1024];
        static int sprite = 0;
        snprintf(filename, sizeof(filename), "/tmp/sprite-%04d.png", sprite);
        sprite_group.image.write(filename);
        sprite++;
      }
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
    std::cout << image.w << " " << image.h;
    image.write(filename);
  }
  return 0;
}
