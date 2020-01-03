#include "screen.h"

#include "nes.h"

#include <queue>
#include <vector>
#include <set>
#include <unordered_map>
#include <nimage/image.h>
#include "extract_colors.h"

const int kGridWidth = kNesBlocksWidth + 1;

std::vector<std::pair<int, int>> GetTopBlocks(const std::vector<std::pair<int, int> >& group,
                                              const std::unordered_map<int, int>& block_to_group) {
  std::vector<std::pair<int, int> > top_blocks;
  if (group.size() == 0) return top_blocks;
  const int group_index = block_to_group.find(group[0].second * kGridWidth + group[0].first)->second;
  for (const auto& block : group) {
    int y = block.second - 1;
    if (y < 0) continue;
    // Block above should be background
    auto it = block_to_group.find(y * kGridWidth + block.first);
    if (it == block_to_group.end() || it->second != group_index) {
      top_blocks.push_back(block);
    }
  }
  return top_blocks;
}

bool IsBlockWalkable(const std::pair<int, int>& block, int x0,
                     nacb::Image8* image,
                     const nacb::Imagef& g2,
                     const uint8_t bg[3]) {
  int by = block.second;
  int bx = block.first;
  if (by == 0) return false;
  int num_grad = 0;
  for (int x = 0; x < 8; ++x) {
    double g = g2(bx * 8 + x + x0, by * 8, 1) +
      g2(bx * 8 + x + x0, by * 8 - 1, 1);
    num_grad += (g > 40);
  }
  if (num_grad >= 6) {
    int num_set = 0;
    for (int x = 0; x < 8; ++x) {
      if (image->get(bx * 8 + x + x0, by * 8, 0) == bg[0] &&
          image->get(bx * 8 + x + x0, by * 8, 1) == bg[1] &&
          image->get(bx * 8 + x + x0, by * 8, 2) == bg[2]) {
        continue;
      }
      num_set++;
    }
    if (num_set >= 6) {
      return true;
    }
  }
  return false;
}
  

std::vector<BackgroundGroup>
FindBackgroundGroups(nacb::Image8* image, const uint8_t bg[3], int x0) {
  nacb::Imagef grads[3] = {nacb::Imagef(1,1,1),
                           nacb::Imagef(1,1,1),
                           nacb::Imagef(1,1,1)};
  for (int c = 0; c < 3; ++c) {
    grads[c] = image->gradient(c);
  }

  std::vector<std::set<int>> block_colors(kGridWidth * kNesBlocksHeight);

  nacb::Imagef g2 = (grads[0] * grads[0]) + (grads[1] * grads[1]) + (grads[2] * grads[2]);
  nacb::Image8 mask(kGridWidth, kNesBlocksHeight, 1);
  mask = 0;
  
  for (int by = 0; by < kNesBlocksHeight; by++) {
    for (int bx = 0; bx < kGridWidth; bx++) {
      std::set<int> colors = GetUniqueColors(*image, bx * 8 + x0, by *8, 8, 8);
      auto it = colors.find(ColorIndex(bg[0], bg[1], bg[2]));
      if (it != colors.end()) {
        colors.erase(it);
      }
      block_colors[by * kGridWidth + bx] = colors;
      if (colors.size()) {
        mask(bx, by) = 1;
      }
    }
  }
  // For each candidate block, find connected neighbors.
  std::set<int> visited;
  std::vector<std::vector<std::pair<int, int>>> groups;
  std::unordered_map<int, int> block_to_group;

  for (int its = 0; its < 2; ++its) {
  for (int by = 0; by < kNesBlocksHeight; by++) {
    for (int bx = 0; bx < kGridWidth; bx++) {
      if (mask(bx, by) == 1 &&
          (its == 1 || IsBlockWalkable(std::make_pair(bx, by), x0, image, g2, bg))) {
        std::set<int>& colors = block_colors[by * kGridWidth + bx];
        
        std::queue<std::pair<int, int>> q;
        q.push(std::make_pair(bx, by));
        
        mask(bx, by) = 2;
        
        std::vector<std::pair<int, int>> group;
        while (!q.empty()) {
          auto block = q.front();
          q.pop();
          group.push_back(block);
          block_to_group[block.second * kGridWidth + block.first] = groups.size();
          
          const std::set<int>& cur_colors = block_colors[block.second * kGridWidth + block.first];
          
          const int neigh[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
          for (int k = 0; k < 4; ++k) {
            const int nx = block.first + neigh[k][0];
            const int ny = block.second + neigh[k][1];
            if (!(nx >= 0 && ny >= 0 && nx < kGridWidth && ny < kNesBlocksHeight)) continue;
            if (mask(nx, ny) == 2) continue;
            //if (mask(nx, ny) != 1) continue;

            const std::set<int>& neigh_colors = block_colors[ny * kGridWidth + nx];
            const int num_shared1 = NumOverlappingColors(colors, neigh_colors);
            const int num_shared2 = NumOverlappingColors(cur_colors, neigh_colors);
                                                   
            if (num_shared1 >= 3 || num_shared2 >= 3 ||
                //(its >= 0 && num_shared1 == cur_colors.size()) ||
                (its >= 0 && num_shared2 == cur_colors.size()) ||
                (its == 1 && neigh_colors.size() == 1 && num_shared1 == 1) ||
                (its == 0 && IsBlockWalkable(std::make_pair(nx, ny), x0, image, g2, bg))) {
              mask(nx, ny) = 2;
              q.push(std::make_pair(nx, ny));
            }
          }
        }
        groups.push_back(group);
      }
    }
  }
  }

  std::vector<BackgroundGroup>  bg_groups;
  for (const auto& g : groups) {
    bool is_walkable = true;
    
    std::vector<std::pair<int, int>> top_blocks = GetTopBlocks(g, block_to_group);
    int num_walkable = 0;
    for (const auto& block : top_blocks) {
      num_walkable += IsBlockWalkable(block, x0, image, g2, bg);
    }
    is_walkable = (top_blocks.size() && num_walkable >= 0.8 * top_blocks.size());

    int min_x = kNesWidth;
    int min_y = kNesHeight;
    int max_x = 0;
    int max_y = 0;
    for (const auto& b : g) {
      min_x = std::min(min_x, b.first);
      min_y = std::min(min_y, b.second);

      max_x = std::max(max_x, b.first);
      max_y = std::max(max_y, b.second);
    }
    BackgroundGroup bg_group;
    bg_group.min_x = min_x;
    bg_group.min_y = min_y;
    bg_group.max_x = max_x;
    bg_group.max_y = max_y;
    bg_group.walkable = is_walkable;
    bg_group.blocks = g;
    bg_groups.push_back(bg_group);
  }
  return bg_groups;
}
