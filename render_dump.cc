#include "plugins/render_hook.h"
#include "proto/render_state.pb.h"
#include <iostream>
#include <nimage/image.h>

bool SaveRenderState(const RenderState& render_state, std::string* data) {
  int offset = 0;
  size_t state_size = 4 + kPaletteSize * 2 + kPatternTableSize * 2 +
                      kSpriteDataSize + kNameTableSize * 4 +
                      sizeof(RenderState::PPUState);
  *data = std::string(state_size, 0);
  memcpy(&data[0], "NES0", 4);
  offset += 4;
  memcpy(&data[offset], render_state.image_palette, kPaletteSize);
  offset += kPaletteSize;
  memcpy(&data[offset], render_state.sprite_palette, kPaletteSize);
  offset += kPaletteSize;

  for (int i = 0; i < 2; ++i) {
    memcpy(&data[offset], render_state.pattern_tables[i], kPatternTableSize);
    offset += kPatternTableSize;
  }
  memcpy(&data[offset], render_state.sprite_data, kSpriteDataSize);
  offset += kSpriteDataSize;

  for (int i = 0; i < 4; ++i) {
    memcpy(&data[offset], render_state.name_tables[i], kNameTableSize);
    offset += kSpriteDataSize;
  }

  memcpy(&data[offset], &render_state.ppu, sizeof(render_state.ppu));
  offset += sizeof(render_state.ppu);
  return true;
}

bool ConvertState(const RenderState& render_state, nes::RenderState* nes_state,
                  bool copy_data = true) {
  if (copy_data) {
    nes_state->set_image_palette(render_state.image_palette, kPaletteSize);
    nes_state->set_sprite_palette(render_state.sprite_palette, kPaletteSize);
    for (int i = 0; i < 2; ++i) {
      nes_state->add_pattern_table(render_state.pattern_tables[i],
                                   kPatternTableSize);
    }
    nes_state->set_sprite_data(render_state.sprite_data, kSpriteDataSize);

    for (int i = 0; i < 4; ++i) {
      nes_state->add_name_table(render_state.name_tables[i], kNameTableSize);
    }
  }

  auto* ppu = nes_state->mutable_ppu();
  ppu->set_sprite_pattern_table(render_state.ppu.sprite_pattern_table);
  ppu->set_screen_pattern_table(render_state.ppu.screen_pattern_table);
  ppu->set_sprite_8_by_16_size(render_state.ppu.sprite_8_by_16_size);
  ppu->set_hit_switch(render_state.ppu.hit_switch);
  ppu->set_vblank_switch(render_state.ppu.vblank_switch);
  ppu->set_hscroll(render_state.ppu.hscroll);
  ppu->set_vscroll(render_state.ppu.vscroll);
  ppu->set_vertical_mirror(render_state.ppu.vertical_mirror);
  ppu->set_name_table(render_state.ppu.name_table);
  ppu->set_scan_line(render_state.ppu.scan_line);
  return true;
}

class RenderDump : public RenderHook {
public:
  void StartFrame(const RenderState& render_state) override {
    auto* frame_state = sequence_.add_frame_state();
    ConvertState(render_state, frame_state->mutable_start_frame());
  }

  void StateUpdate(const RenderState& render_state) override {
    auto* frame_state =
        sequence_.mutable_frame_state(sequence_.frame_state_size() - 1);
    ConvertState(render_state, frame_state->add_state_update(), false);
  }

  void EndFrame(const RenderState& render_state) override {
    auto* frame_state =
        sequence_.mutable_frame_state(sequence_.frame_state_size() - 1);
    ConvertState(render_state, frame_state->mutable_end_frame());
  }

  void RenderedFrame(const uint8_t* image, int w, int h) override {
    if (frame_ % 30 == 0) {
      nacb::Image8 full_image(w, h, 3);
      memcpy(full_image.data, image, w * h * 3);

      char image_path[1024];
      snprintf(image_path, sizeof(image_path), "/tmp/image-%04d.png", frame_);
      full_image.write(image_path);
    }
    if (frame_ % 300 == 0) {
      std::string bytes;
      sequence_.SerializeToString(&bytes);
      sequence_.Clear();

      char image_path[1024];
      snprintf(image_path, sizeof(image_path), "/tmp/data-%04d.pb", frame_);
      FILE* file = fopen(image_path, "w");
      fwrite(bytes.data(), 1, bytes.size(), file);
      fclose(file);
    }
    frame_++;
  }

  bool WantRenderedFrame() override { return true; }

  int frame_ = 0;
  nes::RenderSequence sequence_;
};

static RenderHook* render_hook = 0;

extern "C" RenderHook* CreateRenderHook() {
  return (render_hook = new RenderDump);
}
