#include <nimage/image.h>
#include <levset/levset2d.h>
#include <nappear/mesh.h>
#include <nmath/vec2.h>
#include <nmath/vec3.h>
#include "poly/cpp/delaunay.h"

#include <set>
#include <vector>

nacb::Image8 GetMask(const nacb::Image8& image,
                     const nacb::Vec3<int>& bg_color,
                     int num_pad = 0) {
  nacb::Image8 mask(image.w + 2 * num_pad, image.h + 2 * num_pad, 1);
  mask = 0;
  for (int y = 0; y < image.h; ++y) {
    for (int x = 0; x < image.w; ++x) {
      mask(num_pad + x, num_pad + y, 0) =
        (image(x, y, 0) == bg_color.x &&
         image(x, y, 1) == bg_color.y &&
         image(x, y, 2) == bg_color.z) ? 0 : 255;
    }
  }
  return mask;
}

std::vector<nacb::Vec2d> RemoveNearPoints(const std::vector<nacb::Vec2d>& points,
                                          double dist) {
  std::vector<nacb::Vec2d> unique_points;
  for (int i = 0; i < points.size(); ++i) {
    bool found = false;
    for (int j = 0; j < unique_points.size(); ++j) {
      if ((points[i] - unique_points[j]).len() < dist) {
        found = true;
        break;
      }
    }
    if (found) continue;
    unique_points.push_back(points[i]);
  }
  return unique_points;
}

void SmoothMesh(nappear::Mesh* mesh) {
  std::vector<std::set<int>> adj(mesh->vert.size());
  for (const auto& face : mesh->faces) {
    for (int e = 0; e < 3; ++e) {
      adj[face.vi[e]].insert(face.vi[(e + 1) % 3]);
    }
  }
  std::vector<nacb::Vec3d> vert = mesh->vert;
  for (int i = 0; i < vert.size(); ++i) {
    nacb::Vec3d sum = vert[i];
    for (int j : adj[i]) {
      sum += vert[j];
    }
    mesh->vert[i] = sum * (1.0 / (1 + adj[i].size()));
  }
}

double TriangleArea(const nappear::Mesh& mesh, int i) {
  double ls[3];
  
  // Get the lengths
  for(int k = 0; k < 3; k++){
    int v1 = mesh.faces[i].vi[k];
    int v2 = mesh.faces[i].vi[(k+1)%3];

    double l = (mesh.vert[v2] - mesh.vert[v1]).len();
    ls[k] = l;
  }
    
  // Semi-perimeter
  double S = (ls[0] + ls[1] + ls[2])/2;

  // Make sure semi-perimeter is bigger than all sides
  S = std::max(S, std::max(ls[0], std::max(ls[1], ls[2])));

  return sqrt(S*(S - ls[0])*(S - ls[1])*(S - ls[2]));
}


std::vector<std::map<int, double>>  GetVertexNeighbors(const nappear::Mesh & mesh){
  std::vector<std::map<int, double>> vneigh(mesh.vert.size());

  for (int i = 0; i < mesh.faces.size(); i++){
    const double A = TriangleArea(mesh, i);
    
    for (int k=0; k < 3 ; k++){
      int v1 = mesh.faces[i].vi[k];
      int v2 = mesh.faces[i].vi[(k+1)%3];

      if(!vneigh[v1].count(v2))
	vneigh[v1][v2] = 0;
    
      vneigh[v1][v2] += A;

      if(!vneigh[v2].count(v1))
	vneigh[v2][v1] = 0;

      vneigh[v2][v1] += A;
    }
  }
  return vneigh;
}


struct blowup_options_t {
  int ntime;
  double alpha; // Controls blow-up of the mesh.
  double beta; 
  int ninner; // Number of inner iterations (to reduce strain/stretch).

  blowup_options_t() {
    ntime = 30;
    alpha = 0.1;
    beta = 1.0;
    ninner = 15;
  }
};


nappear::Mesh BlowUpMesh(const nappear::Mesh& mesh,
                         const blowup_options_t & opts = blowup_options_t()){
  nappear::Mesh result;
  result = mesh;

  // Mesh adjacency
  std::vector<std::map<int, double>> vneigh = GetVertexNeighbors(mesh);

  for (int t=0; t < opts.ntime; t++) {
    result.initNormals();

    // Move along normals
    for (int i=0; i < result.vert.size(); i++)
      result.vert[i] += result.norm[i] * opts.alpha;

    // Fix the lengths; this force moves the vertex in the direction that preserves lengths.
    for (int inner=0; inner<opts.ninner; inner++) {
      std::vector<nacb::Vec3d> updates = result.vert;

      // For each vertex, compute the displacement (area weighted average)
      for (int i=0; i < result.vert.size(); i++) {
	map<int, double>::iterator it(vneigh[i].begin());
	double wsum = 0;
	nacb::Vec3d update(0, 0, 0);

	// Check all neighboring vertices.
	for(; it != vneigh[i].end(); ++it){
	  int j = it->first;
	  double wt = it->second;

	  nacb::Vec3d drest = mesh.vert[j] - mesh.vert[i];
	  nacb::Vec3d dcur = result.vert[j] - result.vert[i];

	  double lrest = drest.len();
	  double  lcur = dcur.len();

	  if(lcur > lrest)
	    update += (dcur * ((lcur - lrest)/lcur))*0.5*opts.beta*wt;
	  else
	    update -= (dcur * ((lrest - lcur)/lrest))*0.5*opts.beta*wt;

	  wsum += wt;
	}

	wsum = std::max(wsum, 1e-8);
	update *= (1.0/wsum);

	updates[i] += update;
      }
      
      result.vert = updates;      
    }
  }
  return result;
}

bool AllWhite(const nacb::Image8& image) {
  for (int y = 0; y <image.h;++y) {
    for (int x = 0; x< image.w; ++x) {
      if (!image(x, y, 0)) return false;
    }
  }
  return true;
}

nappear::Mesh CreateMeshFromImages(const nacb::Image8& lowres_image,
                                   const nacb::Image8& highres_image,
                                   const nacb::Vec3<int>& bg_color) {
  const int num_pad_highres = 1;
    
  nacb::Image8 lowres_mask = GetMask(lowres_image, bg_color);
  nacb::Imagef highres_mask = GetMask(highres_image, bg_color, num_pad_highres);
  const int factor = highres_image.w / lowres_mask.w;

  std::vector<nacb::Vec2d> points;
  std::vector<std::pair<int, int> > lines;

  if (AllWhite(lowres_mask)) {
    std::cout << "All white:" << lowres_mask.w << " " << lowres_mask.h;
    for (int y = 0; y <= lowres_image.h; ++y) {
      points.push_back(nacb::Vec2d(0, y));
      points.push_back(nacb::Vec2d(lowres_image.w, y));
    }
    for (int x = 0; x <= lowres_image.w; ++x) {
      points.push_back(nacb::Vec2d(x, 0));
      points.push_back(nacb::Vec2d(x, lowres_image.h));
    }
  } else {
    FullLevelSet2D levelset(highres_mask, 1.0, 1.0);
    // These points kind of suck.
    levelset.getLines(points, lines);

    // TODO: need to add more points to the border edges
    //  addBorderEdges(points, lines, &levelset);
    for (auto& p : points) {
      p.x = (p.x - num_pad_highres) * double(lowres_mask.w) / (highres_mask.w - 2 * num_pad_highres);
      p.y = (p.y - num_pad_highres) * double(lowres_mask.h) / (highres_mask.h - 2 * num_pad_highres);
    }
    points = RemoveNearPoints(points, 0.125);
  }
  const int num_points_border = points.size();
  
  std::set<int> interior_points;
  std::set<int> exterior_points;
  double ox = 0.0;
  double oy = 0.0;
  for (int y = 0; y < lowres_mask.h * 2; ++y) {
    for (int x = 0; x < lowres_mask.w * 2; ++x) {
      if (lowres_mask(x / 2, y / 2)) {
        interior_points.insert(points.size());
        points.push_back(nacb::Vec2d((ox + x / 2.0), (oy + y / 2.0)));
      }
    }
  }
  int num_points_keep = points.size();
  for (int y = 0; y < lowres_mask.h; ++y) {
    for (int x = 0; x < lowres_mask.w; ++x) {
      if (!lowres_mask(x, y)) {
        exterior_points.insert(points.size());
        points.push_back(nacb::Vec2d((ox + x), (oy + y)));
      }
    }
  }

  for (auto& p: points) {
    p.x += 0.1 * ((double(rand()) / RAND_MAX) - 0.5);
    p.y += 0.1 * ((double(rand()) / RAND_MAX) - 0.5);
  }

  std::vector<nacb::Vec3<int> > tris = delaunay(points);
  std::vector<nacb::Vec3<int> > good_tris;
  nappear::Mesh mesh;

  points.resize(num_points_keep);

  std::vector<nacb::Vec3d> points3;
  for (const auto& point : points) {
    points3.push_back(nacb::Vec3d(point.x, point.y, -1));
  }
  for (const auto& point : points) {
    points3.push_back(nacb::Vec3d(point.x, point.y, 1));
  }
  std::vector<nacb::Vec2d> points2;
  for (const auto& point : points) {
    points2.push_back(nacb::Vec2d(point.x / lowres_mask.w, 1.0 - point.y / lowres_mask.h));
  }

  mesh.tvert = points2;
  mesh.vert = points3;
  
  for (const auto& tri: tris) {
    if (exterior_points.count(tri[0]) ||
        exterior_points.count(tri[1]) ||
        exterior_points.count(tri[2]))
      continue;
    if (interior_points.count(tri[0]) ||
        interior_points.count(tri[1]) ||
        interior_points.count(tri[2])) {
      nappear::Mesh::Face face;
      face.vi[0] = tri[0];
      face.vi[1] = tri[1];
      face.vi[2] = tri[2];
      face.tci[0] = tri[0];
      face.tci[1] = tri[1];
      face.tci[2] = tri[2];
      mesh.faces.push_back(face);

      // Add the backface
      face.vi[0] = tri[0] + num_points_keep;
      face.vi[1] = tri[2] + num_points_keep;
      face.vi[2] = tri[1] + num_points_keep;
      face.tci[0] = tri[0];
      face.tci[1] = tri[2];
      face.tci[2] = tri[1];
      mesh.faces.push_back(face);
    }
  }
  // Add extruded edges
  for (const auto& tri: tris) {
    if (exterior_points.count(tri[0]) ||
        exterior_points.count(tri[1]) ||
        exterior_points.count(tri[2]))
      continue;

    if (interior_points.count(tri[0]) ||
        interior_points.count(tri[1]) ||
        interior_points.count(tri[2])) {
      for (int e = 0; e < 3; ++e) {
        int v1 = tri[e];
        int v2 = tri[(e + 1) % 3];

        if (!interior_points.count(v1) && !interior_points.count(v2)) {
          nappear::Mesh::Face face;
          face.vi[0] = v2;
          face.vi[1] = v1;
          face.vi[2] = v1 + num_points_keep;
          face.tci[0] = v2;
          face.tci[1] = v1;
          face.tci[2] = v1;
          mesh.faces.push_back(face);
        
          face.vi[0] = v1 + num_points_keep;
          face.vi[1] = v2 + num_points_keep;;
          face.vi[2] = v2;
          face.tci[0] = v1;
          face.tci[1] = v2;
          face.tci[2] = v2;
          mesh.faces.push_back(face);
        }
      }
    }
  }
  mesh.initNormals();
  if (lowres_image.w == 8 || lowres_image.h == 8) {
    return mesh;
  }
  SmoothMesh(&mesh);
  SmoothMesh(&mesh);
  mesh.initNormals();
  return BlowUpMesh(mesh);  
}

#ifdef MESH_UTILS_MAIN
int main(int ac, char* av[]) {
  nacb::Image8 lowres_image;
  nacb::Image8 highres_image;

  if (!lowres_image.read(av[1])) {
    std::cerr << "Unable to read lowres image from:" << av[1] << std::endl;
    return -1;
  }
  if (!highres_image.read(av[2])) {
    std::cerr << "Unable to read highres image from:" << av[2] << std::endl;
    return -1;
  }
  nacb::Vec3<int> bg_color(lowres_image(0, 0, 0),
                           lowres_image(0, 0, 1),
                           lowres_image(0, 0, 2));
  auto mesh = CreateMeshFromImages(lowres_image, highres_image, bg_color);
  mesh.saveObj("/tmp/mesh_utils.obj");
  
  return 0;
}
#endif
