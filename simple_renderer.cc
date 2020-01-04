#include "proto/render_state.pb.h"

#include "nes.h"
#include "render_utils.h"
#include "sprite.h"
#include "screen.h"

#include <iostream>
#include <string>
#include <queue>

#include <nimage/image.h>
#include <nmisc/commandline.h>

void AnnotateImage(nacb::Image8* image, int x0,
                   const std::vector<BackgroundGroup>& bg_groups) {
  for (const auto& group : bg_groups) {
    for (auto& b : group.blocks) {
      for (int y = 0; y < 8; ++y) {
        for (int x = 0; x < 8; ++x) {
          //is_walkable = IsBlockWalkable(b, x0, image, g2, bg);
          if (group.walkable) {
            (*image)(x + b.first*8 + x0, y + b.second*8, 0) /= 4;
            (*image)(x + b.first*8 + x0, y + b.second*8, 0) += 192;
            (*image)(x + b.first*8 + x0, y + b.second*8, 1) /= 2;
            (*image)(x + b.first*8 + x0, y + b.second*8, 2) /= 2;
          } else {
            (*image)(x + b.first*8 + x0, y + b.second*8, 1) /= 4;
            (*image)(x + b.first*8 + x0, y + b.second*8, 1) += 192;
            (*image)(x + b.first*8 + x0, y + b.second*8, 0) /= 2;
            (*image)(x + b.first*8 + x0, y + b.second*8, 2) /= 2;
          }
          /*
            (*image)(x + b.first*8 + x0, y + b.second*8, 0) = gi * 16;
            (*image)(x + b.first*8 + x0, y + b.second*8, 1) = 255 - gi * 16;
            (*image)(x + b.first*8 + x0, y + b.second*8, 2) /= 2;
          */
        }
      }
    }
  }
}

nacb::Image8 RenderFrame(const nes::RenderSequence::FrameState& frame_state,
                         SpriteDatabase<int>* sprite_database,
                         SpriteDatabase<int>* screen_database) {
  nes::RenderState render_state = frame_state.start_frame();

  const bool kBlockAligned = true;
  nacb::Image8 image(kNesWidth + (kBlockAligned ? 16: 0), kNesHeight, 3);
  ClearImage(image, kNesPalette[render_state.image_palette()[0]]);

  // FIXME: This is a hack for SMB1.
  render_state.mutable_ppu()->set_name_table(0);

  RenderSprites(render_state, kBlockAligned ? 8 : 0, /*foreground=*/false, &image);
  std::map<int, int> line_starts
    = RenderBackground(frame_state, kBlockAligned ? 8 : 0, &image);

  std::vector<BackgroundGroup> background_groups =
      FindBackgroundGroups(&image,  kNesPalette[render_state.image_palette()[0]], line_starts);
  if (screen_database) {
    for (auto& group : background_groups) {
      nacb::Image8 tex = group.ExtractImage(image, line_starts);
      if (!screen_database->Exists(tex)) {
        screen_database->Insert(tex, 1);
      }
    }
  }
    
  RenderSprites(render_state, kBlockAligned ? 8 : 0, /*foreground=*/true, &image);

  if (sprite_database) {
    Sprite* sprites = (Sprite*)render_state.sprite_data().c_str();                
    std::vector<SpriteGroup> groups = GroupSprites(sprites, render_state);
    for (auto& sprite_group : groups) {
      if (!sprite_database->Exists(sprite_group.image)) {
        sprite_database->Insert(sprite_group.image, 1);
      }
    }
  }
  return image.subimage(8, 0, kNesWidth, kNesHeight);
}

template <class T>
void WriteSpriteImages(SpriteDatabase<T>& db, const std::string& path) {
  int sprite = 0;
  for (auto& entry: db) {
    char filename[1024];
    snprintf(filename, sizeof(filename), path.c_str(), sprite);
    entry.second.write(filename);
    sprite++;
  }
}

int main(int ac, char* av[]) {
  int output_offset = 0;
  std::string render_path = "/tmp/rendered-%04d.png";
  std::string sprite_path;
  std::string screen_path;

  nacb::CommandLine cline;
  cline.registerOption("render_path", "Path to render files", &render_path);
  cline.registerOption("sprite_path", "Path to sprite files", &sprite_path);
  cline.registerOption("screen_path", "Path to background sprites", &screen_path);
  cline.registerOption("offset", "Offset to use when writing outputs", &output_offset);
  cline.parse(ac, av);

  if (cline.extra_args.empty()) {
    return -1;
  }
  nes::RenderSequence seq;
  if (!LoadRenderSequence(cline.extra_args[0].c_str(), &seq)) {
    return -1;
  }

  SpriteDatabase<int> screen_database;
  SpriteDatabase<int> sprite_database;
  for (int i = 0; i < seq.frame_state_size(); ++i) {
    std::cout << i << std::endl;
    nacb::Image8 image = RenderFrame(seq.frame_state(i), &sprite_database, &screen_database);
    char filename[1024];
    snprintf(filename, sizeof(filename), render_path.c_str(), i + output_offset);
    image.write(filename);
  }

  if (!sprite_path.empty()) {
    std::cout << "Writing sprites to " << sprite_path;
    WriteSpriteImages(sprite_database, sprite_path);
  }
  if (!screen_path.empty()) {
    WriteSpriteImages(screen_database, screen_path);
  }

  return 0;
}
