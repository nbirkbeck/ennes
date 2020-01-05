#include "nes.h"

#include "proto/render_state.pb.h"
#include <map>
#include <nimage/image.h>

void DrawPattern(nacb::Image8& image, int x, int y, const std::string& palette,
                 const std::string& pattern_table, uint8_t entry, int color,
                 TransformFlags flags = FLIP_NONE);

void RenderSprites(const nes::RenderState& render_state, int x_offset,
                   bool foreground, nacb::Image8* image_ptr);

std::map<int, int>
RenderBackground(const nes::RenderSequence::FrameState& frame_state,
                 int x_offset, nacb::Image8* image_ptr);

const int GetAttributeColor(const std::string& name_table, int x, int y);

void ClearImage(nacb::Image8& image, const Color3b& color);

bool LoadRenderSequence(const std::string& filename,
                        nes::RenderSequence* render_sequence);
