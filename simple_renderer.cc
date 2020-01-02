#include "proto/render_state.pb.h"

#include "nes.h"
#include "extract_colors.h"
#include "render_utils.h"
#include "sprite.h"

#include <iostream>
#include <string>
#include <queue>

#include <nimage/image.h>

class SpriteDatabase {
public:
  bool Exists(const nacb::Image8& image) const {
    std::string str(reinterpret_cast<const char*>(image.data), image.w * image.h * 3);
    return database_.count(str);
  }
  void Insert(const nacb::Image8& image) {
    std::string str(reinterpret_cast<const char*>(image.data), image.w * image.h * 3);
    database_[str] = image;
  }
   std::unordered_map<std::string, nacb::Image8> database_;
} sprite_database;

/*
template <class T>
class ImageView {
  ImageView(nacb::Image<T>* source, int x0, int y0) : source_(source), x0_(x0), y0_(y0) {
  }
  T& operator()(int x, int y, int c) {
    return source(x + x0_, y + y0_, c);
  }
  int x0_;
  int y0_;
  nacb::Image<T>* source_;
};
*/
void FindWalkingSurfaces(nacb::Image8* image, const uint8_t bg[3], int x0) {
  nacb::Imagef grads[3] = {nacb::Imagef(1,1,1),
                           nacb::Imagef(1,1,1),
                           nacb::Imagef(1,1,1)};
  for (int c = 0; c < 3; ++c) {
    grads[c] = image->gradient(c);
  }
  nacb::Imagef g2 = (grads[0] * grads[0]) + (grads[1] * grads[1]) + (grads[2] * grads[2]);
  nacb::Image8 mask(kNesWidth / 8, kNesHeight / 8, 1);
  mask = 0;

  for (int by = 1; by < kNesHeight / 8; ++by) {
    for (int bx = 0; bx < kNesWidth / 8; ++bx) {
      int num_grad = 0;
      for (int x = 0; x < 8; ++x) {
        double g = g2(bx * 8 + x + x0, by * 8, 1) +
          g2(bx * 8 + x + x0, by * 8 - 1, 1);
        num_grad += (g > 60);
      }
      if (num_grad >= 6) {
        int num_set = 0;
        for (int x = 0; x < 8; ++x) {
          if (image->get(bx * 8 + x + x0, by * 8, 0) == bg[0] &&
              image->get(bx * 8 + x + x0, by * 8, 1) == bg[1] &&
              image->get(bx * 8 + x + x0, by * 8, 2) == bg[2]) {
            continue;
          }
              
          // (*image)(bx * 8 + x + x0, by * 8, 0) = 0;
          // (*image)(bx * 8 + x + x0, by * 8, 1) = 255;
          // (*image)(bx * 8 + x + x0, by * 8, 2) = 0;
          num_set++;
        }
        if (num_set >= 6) {
          mask(bx, by, 0) = 1;
        }
      }
    }
  }
  // For each candidate block, find connected neighbors.
  std::set<int> visited;
  std::vector<std::vector<std::pair<int, int>>> groups;
  for (int by = 0; by < kNesHeight / 8; by++) {
    for (int bx = 0; bx < kNesWidth / 8; bx++) {
      if (mask(bx, by) == 1) {
        std::set<int> colors = GetUniqueColors(*image, bx * 8 + x0, by *8, 8, 8);
        auto it = colors.find(ColorIndex(bg[0], bg[1], bg[2]));
        if (it != colors.end()) {
          colors.erase(it);
        }
        
        std::queue<std::pair<int, int>> q;
        q.push(std::make_pair(bx, by));
        
        mask(bx, by) = 2;
        

        std::vector<std::pair<int, int>> group;
        while (!q.empty()) {
          auto block = q.front();
          q.pop();
          group.push_back(block);
          std::set<int> cur_colors = GetUniqueColors(*image, block.first * 8 + x0, block.second *8, 8, 8);
          
          int neigh[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
          for (int k = 0; k < 4; ++k) {
            const int nx = block.first + neigh[k][0];
            const int ny = block.second + neigh[k][1];
            if (!(nx >= 0 && ny >= 0 && nx < kNesWidth / 8 && ny < kNesHeight / 8)) continue;
            if (mask(nx, ny) == 2) continue;
            //if (mask(nx, ny) != 1) continue;

            std::set<int> neigh_colors = GetUniqueColors(*image, nx*8 + x0, ny*8, 8, 8);
            int num_shared1 = NumOverlappingColors(colors, neigh_colors);
            int num_shared2 = NumOverlappingColors(cur_colors, neigh_colors);
                                                   
            //            std::cout << "Overlapping:" << colors.size() << " " << num_shared << std::endl;
            if (num_shared1 >= 3 || num_shared2 >= 3 ||
                (num_shared1 == cur_colors.size()) ||
                (neigh_colors.size() == 1 && num_shared1 == 1)) {
              mask(nx, ny) = 2;
              q.push(std::make_pair(nx, ny));
            }
          }
        }
        groups.push_back(group);
      }
    }
  }
  std::cout << "Num groups:" << groups.size() << std::endl;
  for (const auto& g : groups) {
    std::cout << " " << g.size() << std::endl;
    if (g.size() >= 1) {
      for (auto& b : g) {
        for (int y = 0; y < 8; ++y) {
          for (int x = 0; x < 8; ++x) {
            (*image)(x + b.first*8 + x0, y + b.second*8, 0) /= 4;
            (*image)(x + b.first*8 + x0, y + b.second*8, 0) += 192;
            (*image)(x + b.first*8 + x0, y + b.second*8, 1) /= 2;
            (*image)(x + b.first*8 + x0, y + b.second*8, 2) /= 2;
          }
        }
      }
    }
  }
}

nacb::Image8 RenderFrame(const nes::RenderSequence::FrameState& frame_state) {
  nes::RenderState render_state = frame_state.start_frame();

  const bool kBlockAligned = true;
  nacb::Image8 image(kNesWidth + (kBlockAligned ? 8: 0), kNesHeight, 3);
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
  return image;
}

int main(int ac, char* av[]) {
  nes::RenderSequence seq;
  FILE* file= fopen(av[1], "r");
  if (!file) {
    std::cout << "Unable to open " << av[1] << std::endl;
    return -1;
  }
  
  fseek(file, 0, SEEK_END);
  const int size = ftell(file);
  fseek(file, 0, SEEK_SET);
  
  std::string bytes(size, 0);
  fread(&bytes[0], size, 1, file);
  fclose(file);
  seq.ParseFromString(bytes);
  std::cout << "Num frames:" <<  seq.frame_state_size() << std::endl;

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
