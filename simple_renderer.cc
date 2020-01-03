#include "proto/render_state.pb.h"

#include "nes.h"
#include "render_utils.h"
#include "sprite.h"

#include <iostream>
#include <string>
#include <queue>

#include <nimage/image.h>

nacb::Image8 RenderFrame(const nes::RenderSequence::FrameState& frame_state) {
  nes::RenderState render_state = frame_state.start_frame();

  const bool kBlockAligned = true;
  nacb::Image8 image(kNesWidth + (kBlockAligned ? 16: 0), kNesHeight, 3);
  ClearImage(image, kNesPalette[render_state.image_palette()[0]]);

  // FIXME: This is a hack for SMB1.
  render_state.mutable_ppu()->set_name_table(0);

  RenderSprites(render_state, kBlockAligned ? 8 : 0, /*foreground=*/false, &image);
  int vscroll_mod8 = RenderBackground(frame_state, kBlockAligned ? 8 : 0, &image);
  FindWalkingSurfaces(&image,  kNesPalette[render_state.image_palette()[0]], 8 - vscroll_mod8);
  RenderSprites(render_state, kBlockAligned ? 8 : 0, /*foreground=*/true, &image);

  Sprite* sprites = (Sprite*)render_state.sprite_data().c_str();                
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
  return image.subimage(8, 0, kNesWidth, kNesHeight);
}

int main(int ac, char* av[]) {
  nes::RenderSequence seq;
  if (!LoadRenderSequence(av[1], &seq)) {
    return -1;
  }
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
