#include "sprite.h"

#include <nimage/image.h>
#include <vector>
#include <queue>
#include <string>
#include <nmath/vec3.h>
#include "render_utils.h"
#include "extract_colors.h"

namespace {
bool SharesCommonBoundary(const nacb::Image8& im1,
                          const nacb::Image8& im2,
                          const nacb::Vec3<int>& bg_color,
                          int x1, int y1,
                          int x2, int y2) {
  const nacb::Image8* ims[2] = {&im1, &im2};
  std::set<int> colors[2];
  for (int j = 0; j < 2; ++j) {
    colors[j] = GetUniqueColors(*ims[j]);
  }
  int cnt = 0;
  for (int c : colors[0]) {
    cnt += colors[1].count(c);
  }
  if (cnt < 3) {
    return false;
  }
  
  int high = 1;
  int coord = 0;
  if (x2 > x1) {
  } else if (x2 < x1) {
    high = 0;
  } else if (y2 > y1) {
    coord = 1;
  } else if (y2 < y1) {
    coord = 1;
    high = 0;
  }
  //int num_good = 0;
  //int num_bad = 0;
  // Check that the silhouette edges between the two sprites mostly overlap.
  int num_bg_equal = 0;
  for (int k = 0; k < 8; ++k) {
    double d2 = 0;
    nacb::Vec3<int> c1, c2;
    for (int c = 0; c < 3; ++c) {
      c1[c] = float(ims[!high]->get(coord == 0 ? 7 : k, coord == 1 ? 7 : k, c));
      c2[c] = float(ims[high]->get(coord == 0 ? 0 : k, coord == 1 ? 0 : k, c));
      //float d = float(ims[!high]->get(coord == 0 ? 7 : k, coord == 1 ? 7 : k, c)) -
      //  float(ims[high]->get(coord == 0 ? 0 : k, coord == 1 ? 0 : k, c));
      //d2 += d*d;
    }
    num_bg_equal += (c1 == bg_color) == (c2 == bg_color);
    //num_good += (sqrt(d2) <= 1);
    //num_bad += ims[!high]->get(coord == 0 ? 7 : k, coord == 1 ? 7 : k, 3) == 0;
  }
  return num_bg_equal > 4;
  //if (num_bad >= 7) return false;
  //return num_good >= 3;
}

void GetSpriteImages(const Sprite* sprites,
                     const nes::RenderState& render_state,
                     std::vector<nacb::Image8>* sprite_images,
                     std::unordered_map<int, int>* sprite_pos) {
  for (int i = 0; i < kNumSprites; ++i) {
    if (sprites[i].IsActive()) {
      const int index = sprites[i].y() * kNesWidth + sprites[i].x;
      (*sprite_pos)[index] = i;

      nacb::Image8 sprite_image(8, 8, 4);
      ClearImage(sprite_image, kNesPalette[render_state.image_palette()[0]]);
      
      DrawPattern(sprite_image, 0, 0,
                  render_state.sprite_palette(),
                  render_state.pattern_table(render_state.ppu().sprite_pattern_table()),
                  sprites[i].pattern, sprites[i].HighColorBits(), sprites[i].GetTransformFlags());
      sprite_images->push_back(sprite_image);
    } else {
      sprite_images->push_back(nacb::Image8(1, 1, 1));
    }
  }
}

std::vector<std::vector<int> >
FindConnectedSprites(const Sprite* sprites,
                     const nacb::Vec3<int>& bg_color,
                     const std::vector<nacb::Image8>& sprite_images,
                     const std::unordered_map<int, int>& sprite_pos) {
  std::unordered_map<int, int> sprite_group_index;
  std::vector<std::vector<int> > sprite_groups;
  for (int i = 0; i < kNumSprites; ++i) {
    int index = sprites[i].y() * kNesWidth + sprites[i].x;
    if (!sprites[i].IsActive()) continue;
    if (sprite_group_index.count(index)) continue;

    std::vector<int> sprite_group;
    std::queue<int> q;

    q.push(index);
    sprite_group_index[index] = sprite_groups.size();

    int neigh[4][2] = {{-8, 0}, {8, 0}, {0, -8}, {0, 8}};
    while (!q.empty()) {
      index = q.front();
      q.pop();

      sprite_group.push_back(sprite_pos.find(index)->second);

      const int x = index % kNesWidth;
      const int y = index / kNesWidth;
      for (int j = 0; j < 4; ++j) {
        const int nx = x + neigh[j][0];
        const int ny = y + neigh[j][1];
        const int neigh_index = ny * kNesWidth + nx;
        if (sprite_pos.count(neigh_index) &&
            !sprite_group_index.count(neigh_index) &&
            SharesCommonBoundary(sprite_images[sprite_pos.find(index)->second],
                                 sprite_images[sprite_pos.find(neigh_index)->second],
                                 bg_color,
                                 x, y, nx, ny)) {
          q.push(neigh_index);
          sprite_group_index[neigh_index] = sprite_groups.size();
        }
      }
    }
    sprite_groups.push_back(sprite_group);
  }
  return sprite_groups;
}  
} // namespace

std::vector<SpriteGroup>
GroupSprites(const Sprite* sprites, const nes::RenderState& render_state) {
  std::unordered_map<int, int> sprite_pos;
  std::vector<nacb::Image8> sprite_images;

  nacb::Vec3<int> bg_color(kNesPalette[render_state.image_palette()[0]][0],
                           kNesPalette[render_state.image_palette()[0]][1],
                           kNesPalette[render_state.image_palette()[0]][2]);
  GetSpriteImages(sprites, render_state, &sprite_images, &sprite_pos);
  std::vector<std::vector<int> > sprite_groups =
    FindConnectedSprites(sprites, bg_color, sprite_images, sprite_pos);

  std::cout << "Num active sprites: " << sprite_pos.size() 
            << " num groups: " << sprite_groups.size()
            << std::endl;
  std::vector<SpriteGroup> output;
  for (int i = 0; i < sprite_groups.size(); ++i) {
     int min_x = kNesWidth;
    int min_y = kNesHeight;
    int max_x = 0;
    int max_y = 0;
    for (int j : sprite_groups[i]) {
      std::cout << " " << j << "(" << int(sprites[j].x) << "," << sprites[j].y() << ")";
      min_x = std::min(min_x, int(sprites[j].x));
      max_x = std::max(max_x, int(sprites[j].x));

      min_y = std::min(min_y, int(sprites[j].y()));
      max_y = std::max(max_y, int(sprites[j].y()));
    }
    std::cout << std::endl;
    nacb::Image8 group_sprite(max_x + 8 - min_x, max_y + 8 - min_y, 3);
    for (int j : sprite_groups[i]) {
      for (int y = 0; y < sprite_images[j].h; ++y) {
        for (int x = 0; x < sprite_images[j].w; ++x) {
          for (int c = 0; c < 3; ++c) {
            group_sprite(x + sprites[j].x - min_x, y + sprites[j].y() - min_y, c) = sprite_images[j](x, y, c);
          }
        }
      }
    }
    output.push_back({sprite_groups[i], group_sprite});
  }
  return output;
}
