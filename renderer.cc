#include <GL/glew.h>
#include <GL/gl.h>

#include <nmisc/timer.h>
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
  uint8_t bg[3] = {(uint8_t)bg_color.x, (uint8_t) bg_color.y, (uint8_t) bg_color.z};
  *texture =
    HighresBoundaryColor(sprite_image, HighresBoundarySimple, kFactor, bg);
  if (1) { //sprite_image.w % 8 == 0  && sprite_image.h % 16 == 0) {
    *mesh = CreateMeshFromImages3D(sprite_image, *texture, bg_color);
  } else {
    *mesh = CreateMeshFromImages(sprite_image, *texture, bg_color);
  }
  for (auto& v : mesh->vert) {
    //v.y = kNesHeight - v.y;
  }
  for (auto& tv : mesh->tvert) {
    tv.y = 1.0 - tv.y;
  }
}

void CreateCube(nappear::Mesh* mesh,
                double w=8.0, double h=8.0, double d=16.0,
                bool flip = false,
                const nacb::Image8* tex = 0, const uint8_t* bg = 0) {
  double x0 = 0;
  double x1 = w;
  if (tex && bg) {
    if (IsEdgeBackground(*tex, 0, 0, bg, -1, 0)) x0 = 2;
    if (IsEdgeBackground(*tex, tex->w - 8, 0, bg,  1, 0)) x1 -= 2;
  }
  mesh->vert.push_back(nacb::Vec3d(x0, 0, 0));
  mesh->vert.push_back(nacb::Vec3d(x1, 0, 0));
  mesh->vert.push_back(nacb::Vec3d(x1, h, 0));
  mesh->vert.push_back(nacb::Vec3d(x0, h, 0));
  
  mesh->vert.push_back(nacb::Vec3d(x0, 0, d));
  mesh->vert.push_back(nacb::Vec3d(x1, 0, d));
  mesh->vert.push_back(nacb::Vec3d(x1, h, d));
  mesh->vert.push_back(nacb::Vec3d(x0, h, d));

  int quads[6][4] = {
                     {0, 1, 5, 4}, {1, 2, 6, 5},
                     {2, 3, 7, 6}, {3, 0, 4, 7},
                     {0, 3, 2, 1}, {4, 5, 6, 7},
  };
  mesh->tvert.push_back(nacb::Vec2d( x0 / w, 0));
  mesh->tvert.push_back(nacb::Vec2d( x1 / w, 0));
  mesh->tvert.push_back(nacb::Vec2d( x1 / w, 1));
  mesh->tvert.push_back(nacb::Vec2d( x0 / w, 1));
  
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
    if (!flip) n *= -1;
    mesh->norm.push_back(n);
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
  GeomRef() : geometry_(nullptr), pos_() {}
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

void ClearBlock(nacb::Image8& image, int x0, int y0, const uint8_t bg[3]) {
  for (int y = 0; y < 8; ++y) {
    const int yo = y0 + y;
    if (yo < 0 || yo >= image.h) continue;
    for (int x = 0; x < 8; ++x) {
      const int xo = x0 + x;
      if (xo < 0 || xo >= image.w) continue;
      for (int c = 0; c < 3; ++c) {
        image(xo, yo, c) = bg[c];
      }
    }
  }
}

template <class T>
GeomRef CreateGroupMeshGeom(const BackgroundGroup& group,
                            nacb::Image8& image,
                            const uint8_t bg[3],
                            const std::map<int, int>& line_starts,
                            SpriteDatabase<T>* db) {
  nacb::Image8 tex = group.ExtractImage(image, line_starts);
  const int line_start = line_starts.lower_bound(group.min_y * 8)->second;
  GeomRef geom_ref;
  nacb::Vec3d pos(group.min_x * 8 + (line_start - 8) % 8, group.min_y * 8, 0);
  if (!db->Exists(tex)) {
    std::unique_ptr<nappear::Mesh> mesh(new nappear::Mesh);
    std::unique_ptr<nacb::Image8> texture(new nacb::Image8);
    CreateMeshForSprite(tex, nacb::Vec3<int>(bg[0], bg[1], bg[2]),
                        mesh.get(), texture.get());
    auto geom = std::make_unique<Geometry>(std::move(mesh), std::move(texture));
    int dx = (8 - line_start);
    if (dx == 8) dx = 0;
    geom_ref = GeomRef(geom.get(), pos);
    db->Insert(tex, std::move(geom));
  } else {
    auto& geom = db->Lookup(tex);
    geom_ref = GeomRef(geom.get(), pos);
  }
  for (auto& b : group.blocks) {
    ClearBlock(image, b.first * 8 + line_start, b.second * 8, bg);
  }
  return geom_ref;
}

template <class T>
GeomRef CreateGroupCubeGeom(const BackgroundGroup& group,
                            nacb::Image8& image,
                            const uint8_t* bg,
                            const std::map<int, int>& line_starts,
                            SpriteDatabase<T>* db) {
  const int line_start = line_starts.lower_bound(group.min_y * 8)->second;
  nacb::Image8 tex = group.ExtractImage(image, line_starts);

  GeomRef geom_ref;
  nacb::Vec3d pos(group.min_x * 8 + (line_start - 8) % 8, group.min_y * 8, 0);
  if (!db->Exists(tex)) {
    std::unique_ptr<nappear::Mesh> mesh(new nappear::Mesh);
    CreateCube(mesh.get(), (group.max_x - group.min_x + 1) * 8,
               (group.max_y - group.min_y + 1) * 8, 16, false, &tex, bg);

    std::unique_ptr<nacb::Image8> texture(new nacb::Image8);
    *texture =
      HighresBoundaryColor(tex, HighresBoundarySimple, kFactor, nullptr);
    
    auto geom = std::make_unique<Geometry>(std::move(mesh), std::move(texture));
    geom_ref = GeomRef(geom.get(), pos);
    db->Insert(tex, std::move(geom));
  } else {
    auto& geom = db->Lookup(tex);
    geom_ref = GeomRef(geom.get(), pos);
  }
  for (auto& b : group.blocks) {
    const int line_start = line_starts.lower_bound(b.second * 8)->second;
    ClearBlock(image, b.first * 8 + line_start, b.second * 8, bg);
  }
  return geom_ref;
}

class RenderWindow : public GLWindow {
public:
  RenderWindow(): GLWindow(1920, 1080) {
    MakeQuad(&background_mesh_);
    CreateCube(&container_mesh_, 2 * 16.0 / 9.0, 2., 4., true);
    
    glGenTextures(1, &background_tex_);
    cpos = nacb::Vec3d(0, 0, 3);
    glEnable(GL_DEPTH_TEST);
    glCullFace(GL_BACK);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_ALPHA_TEST);
    fov = 55;
    farPlane = 10.0;
  }

  bool LoadSequence(const std::string& str) {
    return LoadRenderSequence(str.c_str(), &sequence_);
  }
  
  void ProcessFrame() {
    geoms_.clear();
    
    auto frame_state = sequence_.frame_state(frame_number_);
    nes::RenderState& render_state = *frame_state.mutable_start_frame();
    const uint8_t* bg = kNesPalette[render_state.image_palette()[0]];

    const bool kBlockAligned = true;
    nacb::Image8 image(kNesWidth + (kBlockAligned ? 16: 0), kNesHeight, 3);
    ClearImage(image, kNesPalette[render_state.image_palette()[0]]);

    // FIXME: This is a hack for SMB1.
    render_state.mutable_ppu()->set_name_table(0);

    //RenderSprites(render_state, kBlockAligned ? 8 : 0, /*foreground=*/false, &image);
    std::map<int, int> line_starts = RenderBackground(frame_state, kBlockAligned ? 8 : 0, &image);

    std::vector<BackgroundGroup> bg_groups =
      FindBackgroundGroups(&image, bg, line_starts);

    for (auto& group : bg_groups) {
      if (!group.walkable) {
        auto geom = CreateGroupMeshGeom(group, image, bg, line_starts, &screen_db_);
        geoms_.push_back(geom);
      } else {
        std::vector<BackgroundGroup> cubes;
        SegmentIntoCubes(group, &cubes);
        for (auto& cube : cubes) {
          auto geom = CreateGroupCubeGeom(cube, image, bg, line_starts, &screen_db_);
          geoms_.push_back(geom);
        }
      }
    }

    glBindTexture(GL_TEXTURE_2D, background_tex_);
    image.subimage(8, 8, kNesWidth, kNesHeight).initTexture();
    glBindTexture(GL_TEXTURE_2D, 0);

    Sprite* sprites = (Sprite*)render_state.sprite_data().c_str();                
    std::vector<SpriteGroup> groups = GroupSprites(sprites, render_state);

    sprite_start_ = geoms_.size();
    int sprite_index = 0;
    for (auto& sprite_group : groups) {
      if (!sprite_db_.Exists(sprite_group.image)) {
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
      if (sprite_index == 1) {
        sprite_pos_ = nacb::Vec3d(sprite_group.x, sprite_group.y, 0);
      }
      sprite_index++;
    }    
  }
  
  virtual bool keyboard(unsigned char c, int x, int y) {
    switch (c) {
    case ' ':
      ProcessFrame();
      frame_number_++;
      break;
    case 'f':
      flip_ = !flip_;
      if (flip_) {
        cpos.y = -2;
      } else {
        cpos.y = 0;
      }
      break;
    case 'F':
      follow_ = true;
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
    static double angle = 0;
    angle += M_PI / 180.0;
    float light[4] = {0.7*cos(angle), 0.7*sin(angle), 0.7, 0};
    float white[4] = {0.7, 0.7, 0.7, 1};
    glLightfv(GL_LIGHT0, GL_POSITION, light);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);

    float gray[4] = {0.2, 0.2, 0.2, 1};
    float light1[4] = {0, 0.7, 0.7, 0};
    glEnable(GL_LIGHT1);
    glLightfv(GL_LIGHT1, GL_POSITION, light1);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, gray);

    float light2[4] = {-0.3, -0.6, 0.8, 0};

    glEnable(GL_LIGHT2);
    glLightfv(GL_LIGHT2, GL_POSITION, light2);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, gray);

    glEnable(GL_NORMALIZE);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, background_tex_);
    background_mesh_.draw();

    //glDisable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);
    glPushMatrix();
    glScalef(2.0 * (16.0 / 9.0) / kNesWidth, -2.0 / kNesHeight, 3.0 / kNesWidth);
    glTranslatef(-kNesWidth/2, -kNesHeight/2, 0);
    for (auto& geom : geoms_) {
      geom.Draw();
    }
    glPopMatrix();
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glDisable(GL_TEXTURE_2D);
    glPushMatrix();
    glTranslatef(-16.0/9.0, -1, -0.001);

    auto frame_state = sequence_.frame_state(frame_number_);
    nes::RenderState render_state = frame_state.start_frame();
    const uint8_t* bg = kNesPalette[render_state.image_palette()[0]];
    nacb::Vec4f bg_color(bg[0] / 255.0f, bg[1] / 255.0f, bg[2] / 255.0f);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, bg_color.data);
    container_mesh_.draw();
    glPopMatrix();

    
    ProcessFrame();
    frame_number_++;
    if (frame_number_ >= sequence_.frame_state_size()) {
      frame_number_ = 0;
    }

    if (follow_) {
      cpos = cpos * 0.95 + 0.05*nacb::Vec3d(16.0/9.0*(2 * sprite_pos_.x / kNesWidth - 1),
        2 * (1.0 - sprite_pos_.y / kNesHeight) - 1.0, cpos.z);
    }
  }
  
  int frame_number_ = 0;
  bool flip_ = false;
  bool follow_ = false;
  nappear::Mesh background_mesh_;
  nappear::Mesh container_mesh_;
  GLuint background_tex_;
  nes::RenderSequence sequence_;
  SpriteDatabase<std::unique_ptr<Geometry>> sprite_db_;
  SpriteDatabase<std::unique_ptr<Geometry>> screen_db_;
  std::vector<GeomRef> geoms_;
  int sprite_start_;
  nacb::Vec3d sprite_pos_;
};

int main(int ac, char* av[]) {
  glewInit();
    

  RenderWindow window;

  nacb::Timer timer;
  if (!window.LoadSequence(av[1])) {
    return -1;
  }
  std::cout << "Loading took;" << timer.stop();
  
  window.setRefreshRate(60);
  window.loop(true);
  return 0;
}
