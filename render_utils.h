#include "nes.h"

#include "proto/render_state.pb.h"
#include <map>
#include <nimage/image.h>

// Extract the attribute color for entry x,y of the name-table. Assumes
// that the attribute data is tacked onto the end of the name table.
const int GetAttributeColor(const std::string& name_table, int x, int y);

// Clear the image the provided background color.
void ClearImage(nacb::Image8& image, const Color3b& color);

// Draw the pattern into the image at the given offset.
// -image: the source to draw onto.
// -pallette indices the nes palette.
// -entry indices the pattern table.
void DrawPattern(nacb::Image8& image, int x, int y, const std::string& palette,
                 const std::string& pattern_table, uint8_t entry, int color,
                 TransformFlags flags = FLIP_NONE);

// Renders all sprites from the render state into the image.
void RenderSprites(const nes::RenderState& render_state, int x_offset,
                   bool foreground, nacb::Image8* image_ptr);

// Render the background from the frame state.
// Returns an ordered map of the offset into the image as a function
// of the scanline (this takes into account scrolling).
std::map<int, int>
RenderBackground(const nes::RenderSequence::FrameState& frame_state,
                 int x_offset, nacb::Image8* image_ptr);

bool LoadRenderSequence(const std::string& filename,
                        nes::RenderSequence* render_sequence);
