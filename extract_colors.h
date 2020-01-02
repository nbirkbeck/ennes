#include <vector>
#include <algorithm>
#include <nimage/image.h>

std::vector<std::pair<nacb::Vec3f, nacb::Image8>>
  ExtractColorMasks(const nacb::Image8& image);

std::pair<nacb::Image8, std::vector<nacb::Vec3f> >
 PalettizeImage(const nacb::Image8& image);
