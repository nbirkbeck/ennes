#include "nes.h"

#include <nimage/image.h>

void DrawPattern(nacb::Image8& image, int x, int y,
                 const std::string& palette,
                 const std::string& pattern_table,
                 uint8_t entry, int color, TransformFlags flags = FLIP_NONE);

const int GetAttributeColor(const std::string& name_table, int x, int y);

void ClearImage(nacb::Image8& image, const uint8_t color[3]);
