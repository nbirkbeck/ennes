#include <GL/glew.h>
#include <GL/gl.h>

#include <utigl/glwindow.h>
#include "nes.h"
#include "render_utils.h"
#include <nappear/mesh.h>
#include "proto/render_state.pb.h"
#include "sprite.h"
#include "screen.h"
#include "highres_boundary.h"
#include "mesh_utils.h"

const int kFactor = 8;

void MakeQuad(nappear::Mesh* mesh) {
  const double kAspect = 1920.0 / 1080.0;
  mesh->vert.push_back(nacb::Vec3d(-kAspect, -1, 0));
  mesh->vert.push_back(nacb::Vec3d( kAspect, -1, 0));
  mesh->vert.push_back(nacb::Vec3d( kAspect,  1, 0));
  mesh->vert.push_back(nacb::Vec3d(-kAspect,  1, 0));
  mesh->tvert.push_back(nacb::Vec2d( 0, 1));
  mesh->tvert.push_back(nacb::Vec2d( 1, 1));
  mesh->tvert.push_back(nacb::Vec2d( 1,  0));
  mesh->tvert.push_back(nacb::Vec2d( 0, 0));
  nappear::Mesh::Face face;
  face.vi[0] = 0;
  face.vi[1] = 1;
  face.vi[2] = 2;
  face.tci[0] = 0;
  face.tci[1] = 1;
  face.tci[2] = 2;
  mesh->faces.push_back(face);
  face.vi[0] = 0;
  face.vi[1] = 2;
  face.vi[2] = 3;
  face.tci[0] = 0;
  face.tci[1] = 2;
  face.tci[2] = 3;
  mesh->faces.push_back(face);
  mesh->initNormals();
}

void CreateMeshForSprite(const nacb::Image8& sprite_image,
                         const nacb::Vec3<int>& bg_color,
                         nappear::Mesh* mesh,
                         nacb::Image8* texture) {
  *texture =
    HighresBoundaryColor(sprite_image, HighresBoundarySimple, kFactor);
  *mesh = CreateMeshFromImages(sprite_image, *texture, bg_color);
  for (auto& v : mesh->vert) {
    //v.y = kNesHeight - v.y;
  }
  for (auto& tv : mesh->tvert) {
    tv.y = 1.0 - tv.y;
  }
}

void CreateCube(nappear::Mesh* mesh, double w=8, double h=8, double d=16, bool flip = false) {
  mesh->vert.push_back(nacb::Vec3d(0, 0, 0));
  mesh->vert.push_back(nacb::Vec3d(w, 0, 0));
  mesh->vert.push_back(nacb::Vec3d(w, h, 0));
  mesh->vert.push_back(nacb::Vec3d(0, h, 0));
  
  mesh->vert.push_back(nacb::Vec3d(0, 0, d));
  mesh->vert.push_back(nacb::Vec3d(w, 0, d));
  mesh->vert.push_back(nacb::Vec3d(w, h, d));
  mesh->vert.push_back(nacb::Vec3d(0, h, d));

  int quads[6][4] = {
                     {0, 1, 2, 3}, {4, 5, 6, 7},
                     {0, 1, 5, 4}, {1, 2, 6, 5},
                     {2, 3, 7, 6}, {3, 0, 4, 7}
  };
  mesh->tvert.push_back(nacb::Vec2d( 0, 0));
  mesh->tvert.push_back(nacb::Vec2d( 1, 0));
  mesh->tvert.push_back(nacb::Vec2d( 1, 1));
  mesh->tvert.push_back(nacb::Vec2d( 0, 1));
  
  for (int i = 0; i < 6; ++i) {
    int q[4] = {quads[i][0], quads[i][1], quads[i][2], quads[i][3]};
    nappear::Mesh::Face face;
    face.vi[0] = q[0];
    face.vi[1] = q[1];
    face.vi[2] = q[2];
    face.tci[0] = q[0] % 4;
    face.tci[1] = q[1] % 4;
    face.tci[2] = q[2] % 4;
    face.ni[0] = face.ni[1] = face.ni[2] = i;
    mesh->faces.push_back(face);

    face.vi[0] = q[0];
    face.vi[1] = q[2];
    face.vi[2] = q[3];
    face.tci[0] = q[0] % 4;
    face.tci[1] = q[2] % 4;
    face.tci[2] = q[3] % 4;
    face.ni[0] = face.ni[1] = face.ni[2] = i;
    mesh->faces.push_back(face);

    nacb::Vec3d n = (mesh->vert[q[3]] - mesh->vert[q[0]]).cross(mesh->vert[q[1]] - mesh->vert[q[0]]);
    n.normalize();
    mesh->norm.push_back(n * -1);
    std::cout << n.x << " " << n.y << " " << n.z;
  }
  if (flip) {
    for (int i = 0; i < mesh->faces.size(); ++i) {
      std::swap(mesh->faces[i].vi[1], mesh->faces[i].vi[2]);
      std::swap(mesh->faces[i].tci[1], mesh->faces[i].tci[2]);
    }
  }
}

class Geometry {
public:
  Geometry(std::unique_ptr<nappear::Mesh> mesh,
           std::unique_ptr<nacb::Image8> image) : mesh_(std::move(mesh)), image_(std::move(image)) {
    glGenTextures(1, &tex_);
    glBindTexture(GL_TEXTURE_2D, tex_);
    image_->initTexture();
  }
  void Draw() {
    glBindTexture(GL_TEXTURE_2D, tex_);
    mesh_->draw();
  }
  std::unique_ptr<nappear::Mesh> mesh_;
  std::unique_ptr<nacb::Image8> image_;
  GLuint tex_ = 0;
};

class GeomRef {
public:
  GeomRef(Geometry* geometry, nacb::Vec3d pos) : geometry_(geometry), pos_(pos) { }
  void Draw() {
    glPushMatrix();
    glTranslatef(pos_.x, pos_.y, pos_.z);
    geometry_->Draw();
    glPopMatrix();
  }
  Geometry* geometry_;
  nacb::Vec3d pos_;
};


class RenderWindow : public GLWindow {
public:
  RenderWindow(): GLWindow(1920, 1080) {
    MakeQuad(&background_mesh_);
    CreateCube(&container_mesh_, 2 * 16.0/9, 2, 2, true);
    
    glGenTextures(1, &background_tex_);
    cpos = nacb::Vec3d(0, 0, 3);
    glEnable(GL_DEPTH_TEST);
    glCullFace(GL_BACK);
    fov = 55;
  }

  bool LoadSequence(const std::string& str) {
    return LoadRenderSequence(str.c_str(), &sequence_);
  }
  void ProcessFrame() {
    geoms_.clear();
    
    auto frame_state = sequence_.frame_state(frame_number_);
    nes::RenderState render_state = frame_state.start_frame();
    const uint8_t* bg = kNesPalette[render_state.image_palette()[0]];

    const bool kBlockAligned = true;
    nacb::Image8 image(kNesWidth + (kBlockAligned ? 16: 0), kNesHeight, 3);
    ClearImage(image, kNesPalette[render_state.image_palette()[0]]);

    // FIXME: This is a hack for SMB1.
    render_state.mutable_ppu()->set_name_table(0);

    //RenderSprites(render_state, kBlockAligned ? 8 : 0, /*foreground=*/false, &image);
    int vscroll_mod8 = RenderBackground(frame_state, kBlockAligned ? 8 : 0, &image);
    //FindWalkingSurfaces(&image,  kNesPalette[render_state.image_palette()[0]], 8 - vscroll_mod8);
    //RenderSprites(render_state, kBlockAligned ? 8 : 0, /*foreground=*/true, &image);

    std::vector<BackgroundGroup> bg_groups =
      FindBackgroundGroups(&image, bg, 8 - vscroll_mod8);

    int k = 0;
    for (auto& group : bg_groups) {
      if (!group.walkable) {
        k++;
        nacb::Image8 tex = image.subimage((8 - vscroll_mod8) + group.min_x * 8,
                                          group.min_y * 8,
                                          (group.max_x + 1 - group.min_x) * 8,
                                          (group.max_y + 1 - group.min_y) * 8);
        if (!sprite_db_.Exists(tex)) {
          std::unique_ptr<nappear::Mesh> mesh(new nappear::Mesh);
          std::unique_ptr<nacb::Image8> texture(new nacb::Image8);
          CreateMeshForSprite(tex, nacb::Vec3<int>(bg[0], bg[1], bg[2]),
                              mesh.get(), texture.get());
          
          auto geom = std::make_unique<Geometry>(std::move(mesh), std::move(texture));
          geoms_.push_back(GeomRef(geom.get(), nacb::Vec3d(group.min_x * 8 - vscroll_mod8, group.min_y * 8, 0)));
          sprite_db_.Insert(tex, std::move(geom));
        } else {
          auto& geom = sprite_db_.Lookup(tex);
          geoms_.push_back(GeomRef(geom.get(), nacb::Vec3d(group.min_x * 8 - vscroll_mod8, group.min_y * 8, 0)));
        }
        for (auto& b : group.blocks) {
          for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
              for (int c = 0; c < 3; ++c) {
                image(b.first * 8 + x + (8 - vscroll_mod8), b.second * 8 + y, c) = bg[c];
              }
            }
          }
        }
      } else {        
        for (auto& b : group.blocks) {
          nacb::Image8 tex = image.subimage((8 - vscroll_mod8) + b.first * 8,
                                            b.second * 8, 8, 8);
          if (!sprite_db_.Exists(tex)) {
            std::unique_ptr<nappear::Mesh> mesh(new nappear::Mesh);
            CreateCube(mesh.get());

            std::unique_ptr<nacb::Image8> texture(new nacb::Image8);
            *texture =
              HighresBoundaryColor(tex, HighresBoundarySimple, kFactor);

            auto geom = std::make_unique<Geometry>(std::move(mesh), std::move(texture));
            geoms_.push_back(GeomRef(geom.get(), nacb::Vec3d(b.first * 8- vscroll_mod8, b.second * 8, 0)));
            sprite_db_.Insert(tex, std::move(geom));
          } else {
            auto& geom = sprite_db_.Lookup(tex);
            geoms_.push_back(GeomRef(geom.get(), nacb::Vec3d(b.first * 8- vscroll_mod8, b.second * 8, 0)));
          }
          for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
              int xo = b.first * 8 + x + (8- vscroll_mod8);
              if (xo < 0 || xo >= image.w) continue;
              for (int c = 0; c < 3; ++c) {
                image(xo, b.second * 8 + y, c) = bg[c];
              }
            }
          }
        }
      }
    }
    
    glBindTexture(GL_TEXTURE_2D, background_tex_);
    image.subimage(8, 8, kNesWidth, kNesHeight).initTexture();
    glBindTexture(GL_TEXTURE_2D, 0);

    Sprite* sprites = (Sprite*)render_state.sprite_data().c_str();                
    std::vector<SpriteGroup> groups = GroupSprites(sprites, render_state);
    std::cout << "Num sprites:" << groups.size();
    for (auto& sprite_group : groups) {
      if (!sprite_db_.Exists(sprite_group.image)) {
        std::cout << "Creating new mesh..." << std::endl;
        std::unique_ptr<nappear::Mesh> mesh(new nappear::Mesh);
        std::unique_ptr<nacb::Image8> texture(new nacb::Image8);
        CreateMeshForSprite(sprite_group.image, nacb::Vec3<int>(bg[0], bg[1], bg[2]),
                            mesh.get(), texture.get());

        auto geom = std::make_unique<Geometry>(std::move(mesh), std::move(texture));
        geoms_.push_back(GeomRef(geom.get(), nacb::Vec3d(sprite_group.x, sprite_group.y, 10)));
        sprite_db_.Insert(sprite_group.image, std::move(geom));
      } else {
        auto& geom = sprite_db_.Lookup(sprite_group.image);
        geoms_.push_back(GeomRef(geom.get(), nacb::Vec3d(sprite_group.x, sprite_group.y, 10)));
      }
    }    
  }
  
  virtual bool keyboard(unsigned char c, int x, int y) {
    switch (c) {
    case ' ':
      ProcessFrame();
      frame_number_++;
      break;
    }

    refresh();
    return true;
  }

  void drawScene() {
    float ones[4] = {1, 1, 1, 1};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, ones);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    float light[4] = {0.3, 0.6, 0.8, 0};
    float white[4] = {0.7, 0.7, 0.7, 1};
    glLightfv(GL_LIGHT0, GL_POSITION, light);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);

    float light1[4] = {0, 0.7, 0.7, 0};
    glEnable(GL_LIGHT1);
    glLightfv(GL_LIGHT1, GL_POSITION, light1);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, white);

    float light2[4] = {-0.3, -0.6, 0.8, 0};
    float gray[4] = {0.2, 0.2, 0.2, 1};
    glEnable(GL_LIGHT2);
    glLightfv(GL_LIGHT2, GL_POSITION, light2);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, gray);

    glEnable(GL_NORMALIZE);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, background_tex_);
    background_mesh_.draw();

    glPushMatrix();
    glScalef(2.0 * (16.0 / 9.0) / kNesWidth, -2.0 / kNesHeight, 3.0 / kNesWidth);
    glTranslatef(-kNesWidth/2, -kNesHeight/2, 0);
    for (auto& geom : geoms_) {
      geom.Draw();
    }
    glPopMatrix();

    glEnable(GL_CULL_FACE);
    glDisable(GL_TEXTURE_2D);
    glPushMatrix();
    glTranslatef(-16.0/9, -1, 0);

    auto frame_state = sequence_.frame_state(frame_number_);
    nes::RenderState render_state = frame_state.start_frame();
    const uint8_t* bg = kNesPalette[render_state.image_palette()[0]];
    nacb::Vec4f bg_color(bg[0] / 255.0f, bg[1] / 255.0f, bg[2] / 255.0f);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, bg_color.data);
    container_mesh_.draw();
    glPopMatrix();
    glDisable(GL_CULL_FACE);
    
    ProcessFrame();
    frame_number_++;
    if (frame_number_ >= sequence_.frame_state_size()) {
      frame_number_ = 0;
    }
  }
  
  int frame_number_ = 0;
  nappear::Mesh background_mesh_;
  nappear::Mesh container_mesh_;
  GLuint background_tex_;
  nes::RenderSequence sequence_;
  SpriteDatabase<std::unique_ptr<Geometry>> sprite_db_;
  std::vector<GeomRef> geoms_;
};

int main(int ac, char* av[]) {
  glewInit();
    

  RenderWindow window;

  if (!window.LoadSequence(av[1])) {
    return -1;
  }
  window.setRefreshRate(60);
  window.loop(true);
  return 0;
}
