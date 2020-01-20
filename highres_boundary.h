#include "nes.h"
#include <algorithm>
#include <functional>
#include <nimage/image.h>

// Perform a simple upsampling of a binary mask by the given factor.
nacb::Imagef HighresBoundarySimple(const nacb::Image8& image, const int factor);

// Perform an upsampling of a color image.
// The upsamling is aware that the image is a palettized image and tries
// to preserve the boundaries of colors.
//
// If the background color is provided, then the areas that correspond to
// the background will have alpha < 255 and the color will be set to the
// nearest color from the foreground to ensure that rendering looks good.
nacb::Image8 HighresBoundaryColor(
    const nacb::Image8& image,
    std::function<nacb::Imagef(const nacb::Image8&, int factor)> upsample,
    int factor, const Color3b* bg);
