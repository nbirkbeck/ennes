#include "screen.h"

#include "nes.h"

#include <queue>
#include <vector>
#include <set>
#include <unordered_map>
#include <nimage/image.h>
#include <cassert>
#include "extract_colors.h"

const int kGridWidth = kNesBlocksWidth + 1;

bool IsEdgeBackground(const nacb::Image8& im,
                      int x, int y,
                      const Color3b& bg_color,
                      int dx, int dy) {
  int high = 7;
  int coord = 0;
  if (dx > 0) {
    high = 7;
    coord = 0;
  } else if (dx < 0) {
    high = 0;
    coord = 0;
  } else if (dy > 0) {
    high = 7;
    coord = 1;
  } else if (dy < 0) {
    coord = 1;
    high = 0;
  }
  int num_bg_equal = 0;
  for (int k = 0; k < 8; ++k) {
    int num_c_eq = 0;
    for (int c = 0; c < 3; ++c) {
      int xco = x + (coord == 0 ? high : k);
      int yco = y + (coord == 1 ? high : k);
      assert(xco >= 0 || xco < im.w);
      assert(yco >= 0 || yco < im.h);
      num_c_eq += (im.get(x + (coord == 0 ? high : k), y + (coord == 1 ? high : k), c) == bg_color[c]);
    }
    num_bg_equal += (num_c_eq == 3);
  }
  return num_bg_equal == 8;
}

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
                     const Color3b& bg) {
  int by = block.second;
  int bx = block.first;
  if (by == 0) return false;
  int num_grad = 0;
  for (int x = 0; x < 8; ++x) {
    assert(bx * 8 + x + x0 < g2.w);
    assert(by * 8 < g2.h);
    assert(by * 8 - 1 >= 0);

    double g = g2(bx * 8 + x + x0, by * 8, 1) +
      g2(bx * 8 + x + x0, by * 8 - 1, 1);
    num_grad += (g > 40);
  }
  if (num_grad >= 6) {
    int num_set = 0;
    for (int x = 0; x < 8; ++x) {
      assert(bx * 8 + x + x0 < image->w);
      assert(by * 8 < image->h);
      
      if (image->get(bx * 8 + x + x0, by * 8, 0) == bg[0] &&
          image->get(bx * 8 + x + x0, by * 8, 1) == bg[1] &&
          image->get(bx * 8 + x + x0, by * 8, 2) == bg[2]) {
        continue;
      }
      num_set++;
    }
    if (num_set >= 6) {
      // This is a hack to avoid making the scoreboard background.
      int bg_edges = IsEdgeBackground(*image, bx * 8 + x0, by * 8, bg, 1, 0) +
        IsEdgeBackground(*image, bx * 8 + x0, by * 8, bg, -1, 0) +
        IsEdgeBackground(*image, bx * 8 + x0, by * 8, bg, 0,  1);
      return bg_edges <= 1;
    }
  }
  return false;
}

bool IsBlockListWalkable(const std::vector<std::pair<int, int>>& g,
                         const std::map<int, int>& line_starts,
                         nacb::Image8* image,
                         const nacb::Imagef& g2,
                         const std::unordered_map<int, int>& block_to_group,
                         const Color3b& bg) {
  std::vector<std::pair<int, int>> top_blocks = GetTopBlocks(g, block_to_group);
  int num_walkable = 0;
  for (const auto& block : top_blocks) {
    const int x0 =  line_starts.lower_bound(block.second * 8)->second;
    num_walkable += IsBlockWalkable(block, x0, image, g2, bg);
  }
  return (top_blocks.size() && num_walkable >= 0.8 * top_blocks.size()) /*&& num_walkable >= 2)*/;
}


std::vector<BackgroundGroup>
FindBackgroundGroups(nacb::Image8* image, const Color3b& bg,
                     const std::map<int, int>& line_starts) {
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
      const int x0 = line_starts.lower_bound(by * 8)->second;
      std::set<int> colors = GetUniqueColors(*image, bx * 8 + x0, by *8, 8, 8);
      auto it = colors.find(ColorIndex(bg));
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
      int x0 = line_starts.lower_bound(by * 8)->second;
      if (mask(bx, by) == 1 &&
          (its == 1 || IsBlockWalkable(std::make_pair(bx, by), x0, image, g2, bg))) {
        std::set<int> colors = block_colors[by * kGridWidth + bx];
        
        std::queue<std::pair<int, int>> q;
        q.push(std::make_pair(bx, by));

        assert(bx < mask.w && by < mask.h);
        mask(bx, by) = 2;
        
        std::vector<std::pair<int, int>> group;
        while (!q.empty()) {
          auto block = q.front();
          q.pop();
          group.push_back(block);
          block_to_group[block.second * kGridWidth + block.first] = groups.size();
          
          const std::set<int>& cur_colors = block_colors[block.second * kGridWidth + block.first];
          //colors.insert(cur_colors.begin(), cur_colors.end());
          
          assert(block.first * 8 + x0  >= 0);
          assert(block.first * 8 + x0 + 8 <= image->w);
          assert(block.second * 8 >= 0);
          assert(block.second * 8 + 8 <= image->h);
          
          const int neigh[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
          for (int k = 0; k < 4; ++k) {
            const int nx = block.first + neigh[k][0];
            const int ny = block.second + neigh[k][1];
            if (!(nx >= 0 && ny >= 0 && nx < kGridWidth && ny < kNesBlocksHeight)) continue;
            if (mask(nx, ny) == 2) continue;

            // If the joining edge is all background, don't connect.
            int x0 = line_starts.lower_bound(block.second * 8)->second;
            if (IsEdgeBackground(*image, block.first * 8 + x0, block.second * 8, bg, neigh[k][0], neigh[k][1])) {
              continue;
            }
            x0 = line_starts.lower_bound(ny * 8)->second;
            if (IsEdgeBackground(*image, nx * 8 + x0, ny * 8, bg, -neigh[k][0], -neigh[k][1])) {
              continue;
            }

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
        if (its == 0 && !IsBlockListWalkable(group, line_starts, image, g2, block_to_group, bg)) {
          for (const auto& b : group) {
            mask(b.first, b.second) = 1;
            block_to_group.erase(b.second * kGridWidth + b.first);
          }
        } else {
          groups.push_back(group);
        }
      }
    }
  }
  }

  // Merge single blocks surrounded by others.
  for (auto& g : groups) {
    if (g.size() != 1) continue;
    const int neigh[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    std::set<int> neighs;
    int num_neigh = 0;
    for (int k = 0; k < 4; ++k) {
      int nx = g.begin()->first + neigh[k][0];
      int ny = g.begin()->second + neigh[k][1];
      if (block_to_group.count(ny * kGridWidth + nx)) {
        neighs.insert(block_to_group[ny * kGridWidth + nx]);
        num_neigh++;
      }
    }
    if (num_neigh >= 3) {
      groups[*neighs.begin()].push_back(*g.begin());
      g.pop_back();
    }
  }
  
  std::vector<BackgroundGroup>  bg_groups;
  for (const auto& g : groups) {
    if (!g.size()) continue;
    const bool is_walkable = IsBlockListWalkable(g, line_starts, image, g2, block_to_group, bg);
    bg_groups.push_back(BackgroundGroup(g, is_walkable));
  }
  return bg_groups;
}


void SegmentIntoCubes(const BackgroundGroup& group,
                      std::vector<BackgroundGroup>* cubes) {
  std::map<int, int> pos;
  std::set<int> assigned;
  for (int i = 0; i < group.blocks.size(); ++i) {
    const auto& block = group.blocks[i];
    pos[block.second * kGridWidth + block.first] = i;
  }
  for (auto p : pos) {
    if (assigned.count(p.second)) continue;
    std::vector<std::pair<int, int>> blocks;
    blocks.push_back(group.blocks[p.second]);

    assigned.insert(p.second);
    const auto& block = group.blocks[p.second];

    if ((block.first + 1 < kGridWidth) &&
        (block.second + 1 < kNesBlocksHeight)) {
      int neigh[3] = {
                      (block.second + 1) * kGridWidth + block.first,
                      block.second * kGridWidth + block.first + 1,
                      (block.second + 1) * kGridWidth + block.first + 1};
      if (pos.count(neigh[0]) && pos.count(neigh[1]) && pos.count(neigh[2])) {
        for (int n : neigh) {
          blocks.push_back(group.blocks[pos[n]]);
          assigned.insert(pos[n]);
        }
      }
    }
    BackgroundGroup cube_group(blocks, group.walkable);
    cubes->push_back(cube_group);
  }
}
