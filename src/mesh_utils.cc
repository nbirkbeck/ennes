#include "mesh_utils.h"

#include "poly/cpp/delaunay.h"
#include <levset/levset2d.h>
#include <levset/levset3d.h>
#include <map>
#include <nappear/fbo.h>
#include <nappear/mesh.h>
#include <nimage/image.h>
#include <nmath/vec2.h>
#include <nmath/vec3.h>
#include <set>
#include <unordered_map>
#include <vector>

namespace {
// Get a mask from the image using the alpha channel (if exists) or by
// using the background color otherwise.
nacb::Image8 GetMask(const nacb::Image8& image, const Color3b& bg_color,
                     int num_pad = 0) {
  nacb::Image8 mask(image.w + 2 * num_pad, image.h + 2 * num_pad, 1);
  mask = 0; // TODO(efficiency)
  for (int y = 0; y < image.h; ++y) {
    for (int x = 0; x < image.w; ++x) {
      if (image.nchannels == 4) {
        mask(num_pad + x, num_pad + y, 0) = (image(x, y, 3) > 128) ? 255 : 0;
      } else {
        mask(num_pad + x, num_pad + y, 0) =
            (image(x, y, 0) == bg_color.x && image(x, y, 1) == bg_color.y &&
             image(x, y, 2) == bg_color.z)
                ? 0
                : 255;
      }
    }
  }
  return mask;
}

// O(n^2) removal of similar points within the same distance.
std::vector<nacb::Vec2d>
RemoveNearPoints(const std::vector<nacb::Vec2d>& points, double dist) {
  std::vector<nacb::Vec2d> unique_points;
  for (int i = 0; i < points.size(); ++i) {
    bool found = false;
    for (int j = 0; j < unique_points.size(); ++j) {
      if ((points[i] - unique_points[j]).len() < dist) {
        found = true;
        break;
      }
    }
    if (found)
      continue;
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
  for (int k = 0; k < 3; k++) {
    const int v1 = mesh.faces[i].vi[k];
    const int v2 = mesh.faces[i].vi[(k + 1) % 3];

    ls[k] = (mesh.vert[v2] - mesh.vert[v1]).len();
  }

  // Semi-perimeter
  double S = (ls[0] + ls[1] + ls[2]) / 2;

  // Make sure semi-perimeter is bigger than all sides
  S = std::max(S, std::max(ls[0], std::max(ls[1], ls[2])));

  return sqrt(S * (S - ls[0]) * (S - ls[1]) * (S - ls[2]));
}

std::vector<std::map<int, double>>
GetVertexNeighbors(const nappear::Mesh& mesh) {
  std::vector<std::map<int, double>> vneigh(mesh.vert.size());

  for (int i = 0; i < mesh.faces.size(); i++) {
    const double A = TriangleArea(mesh, i);

    for (int k = 0; k < 3; k++) {
      const int v1 = mesh.faces[i].vi[k];
      const int v2 = mesh.faces[i].vi[(k + 1) % 3];

      if (!vneigh[v1].count(v2))
        vneigh[v1][v2] = 0;

      vneigh[v1][v2] += A;

      if (!vneigh[v2].count(v1))
        vneigh[v2][v1] = 0;

      vneigh[v2][v1] += A;
    }
  }
  return vneigh;
}

nappear::Mesh BlowUpMesh(const nappear::Mesh& mesh,
                         const BlowUpOptions& opts = BlowUpOptions()) {
  nappear::Mesh result;
  result = mesh;

  // Mesh adjacency
  std::vector<std::map<int, double>> vneigh = GetVertexNeighbors(mesh);

  for (int t = 0; t < opts.ntime; t++) {
    result.initNormals();

    // Move along normals
    for (int i = 0; i < result.vert.size(); i++)
      result.vert[i] += result.norm[i] * opts.alpha;

    // Fix the lengths; this force moves the vertex in the direction that
    // preserves lengths.
    for (int inner = 0; inner < opts.ninner; inner++) {
      std::vector<nacb::Vec3d> updates = result.vert;

      // For each vertex, compute the displacement (area weighted average)
      for (int i = 0; i < result.vert.size(); i++) {
        map<int, double>::iterator it(vneigh[i].begin());
        double wsum = 0;
        nacb::Vec3d update(0, 0, 0);

        // Check all neighboring vertices.
        for (; it != vneigh[i].end(); ++it) {
          int j = it->first;
          double wt = it->second;

          const nacb::Vec3d drest = mesh.vert[j] - mesh.vert[i];
          const nacb::Vec3d dcur = result.vert[j] - result.vert[i];

          const double lrest = drest.len();
          const double lcur = dcur.len();

          if (lcur > lrest)
            update += (dcur * ((lcur - lrest) / lcur)) * 0.5 * opts.beta * wt;
          else
            update -= (dcur * ((lrest - lcur) / lrest)) * 0.5 * opts.beta * wt;

          wsum += wt;
        }

        wsum = std::max(wsum, 1e-8);
        update *= (1.0 / wsum);

        updates[i] += update;
      }

      result.vert = updates;
    }
  }
  return result;
}

bool AllWhite(const nacb::Image8& image) {
  for (int y = 0; y < image.h; ++y) {
    for (int x = 0; x < image.w; ++x) {
      if (!image(x, y, 0))
        return false;
    }
  }
  return true;
}

typedef map<int, double> vertex_neigh_t;

// FIXME: this was copied from bone_weights.
//       bone_weights should be fixed to
template <class T1, class T2>
vector<vertex_neigh_t> mesh_laplacian(const std::vector<Vec3<T1>>& vert,
                                      const std::vector<T2>& tris) {
  int nvert = vert.size();
  int ntris = tris.size();

  std::vector<vertex_neigh_t> vneigh(nvert);
  std::vector<double> A(nvert, 0);

  for (int i = 0; i < ntris; i++) {
    for (int k = 0; k < 3; k++) {
      int i1 = tris[i].vi[k];
      int i2 = tris[i].vi[(k + 1) % 3];

      if (!vneigh[i1].count(i2))
        vneigh[i1][i2] = 0.0;
      if (!vneigh[i2].count(i1))
        vneigh[i2][i1] = 0.0;
    }
  }

  for (int i = 0; i < ntris; i++) {
    int vs[3] = {tris[i].vi[0], tris[i].vi[1], tris[i].vi[2]};
    Vec3d es[3] = {vert[vs[1]] - vert[vs[0]], vert[vs[2]] - vert[vs[1]],
                   vert[vs[0]] - vert[vs[2]]};
    double lens[3];
    for (int k = 0; k < 3; k++)
      lens[k] = es[k].normalize();

    double cts[3] = {-es[0].dot(es[2]), -es[0].dot(es[1]), -es[1].dot(es[2])};
    double S = (lens[0] + lens[1] + lens[2]) / 2.0;
    double At = sqrt(S * (S - lens[0]) * (S - lens[1]) * (S - lens[2]));
    bool obtuse = (cts[0] < 0 || cts[1] < 0 || cts[2] < 0);

    for (int k = 0; k < 3; k++) {
      // sometimes the cos^2 was greater than 1 causing s to be nan
      double s = sqrt(1.0 - std::min(cts[(k + 2) % 3] * cts[(k + 2) % 3], 1.0));
      // This means the triangle is a little tiny sliver
      if (s < 1e-12)
        s = 1e-12;

#warning These weights were sometimes negative (I see no reason for this).  abs fixed problems in arap_deform (some meshes were getting bunched up).  These meshes were the ones with large negative values in the laplacian matrix.  Make sure skinning works!
      double cot = fabs(cts[(k + 2) % 3] / s); // fabs????

      if (obtuse) {
        if (cts[k] < 0)
          A[vs[k]] += At / 2.;
        else
          A[vs[k]] += At / 4.;

        if (cts[(k + 1) % 3] < 0)
          A[vs[(k + 1) % 3]] += At / 2.;
        else
          A[vs[(k + 1) % 3]] += At / 4.;
      } else {
        A[vs[k]] = A[vs[k]] + cot * lens[k] * lens[k] / 8;
        A[vs[(k + 1) % 3]] = A[vs[(k + 1) % 3]] + cot * lens[k] * lens[k] / 8;
      }

      int i1 = vs[k];
      int i2 = vs[(k + 1) % 3];

      vneigh[i1][i1] -= cot;
      vneigh[i2][i1] += cot;

      vneigh[i2][i2] -= cot;
      vneigh[i1][i2] += cot;
    }
  }

  // Divide by 2*A[i]
  for (int i = 0; i < nvert; i++) {
    vertex_neigh_t::iterator b = vneigh[i].begin(), e = vneigh[i].end();
    for (; b != e; b++) {
      (*b).second /= (2.0 * A[i]);
    }
  }

  // for(int i=0; i<nvert; i++)printf("A[%d] = %lf\n", i, A[i]);
  return vneigh;
}

SparseMatrix convertToSparse(const std::vector<vertex_neigh_t>& neigh,
                             const vector<int>& extra, double cwt) {

  int m = neigh.size();
  std::vector<std::vector<std::pair<int, double>>> cols(m);

  for (int i = 0; i < m; i++) {
    int hadi = 0;
    for (vertex_neigh_t::const_iterator b = neigh[i].begin();
         b != neigh[i].end(); b++) {
      hadi |= (b->first == i);
      cols[b->first].push_back(std::pair<int, double>(i, b->second));
    }
    if (!hadi)
      cols[i].push_back(std::pair<int, double>(i, 0.0));
  }

  printf("adding extra %d\n", extra.size());

  for (int i = 0; i < extra.size(); i++) {
    cols[extra[i]].push_back(std::pair<int, double>(m + i, cwt));
  }
  printf("got sparse matrix\n");

  return SparseMatrix(m + extra.size(), cols);
}

SparseMatrix constraintMatrix(int n, const std::vector<int>& indices,
                              double cwt) {
  std::vector<std::vector<std::pair<int, double>>> cols(n);

  for (int i = 0; i < indices.size(); i++) {
    cols[indices[i]].push_back(std::pair<int, double>(i, cwt));
  }
  return SparseMatrix(indices.size(), cols);
}

} // namespace

nappear::Mesh CreateMeshFromImages(const nacb::Image8& lowres_image,
                                   const nacb::Image8& highres_image,
                                   const Color3b& bg_color,
                                   const BlowUpOptions& opts) {
  const int num_pad_highres = 1;

  nacb::Image8 lowres_mask = GetMask(lowres_image, bg_color);
  nacb::Imagef highres_mask = GetMask(highres_image, bg_color, num_pad_highres);
  const int factor = highres_image.w / lowres_mask.w;

  std::vector<nacb::Vec2d> points;
  std::vector<std::pair<int, int>> lines;

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
      p.x = (p.x - num_pad_highres) * double(lowres_mask.w) /
            (highres_mask.w - 2 * num_pad_highres);
      p.y = (p.y - num_pad_highres) * double(lowres_mask.h) /
            (highres_mask.h - 2 * num_pad_highres);
    }
    points = RemoveNearPoints(points, 0.125);
  }
  const int num_points_border = points.size();

  std::set<int> interior_points;
  std::set<int> exterior_points;
  double ox = 0.25;
  double oy = 0.25;
  for (int y = 0; y < lowres_mask.h * 2; ++y) {
    for (int x = 0; x < lowres_mask.w * 2; ++x) {
      if (lowres_mask(x / 2, y / 2)) {
        interior_points.insert(points.size());
        points.push_back(nacb::Vec2d((ox + x / 2.0), (oy + y / 2.0)));
      }
    }
  }
  ox = 0.5;
  oy = 0.5;
  int num_points_keep = points.size();
  for (int y = 0; y < lowres_mask.h; ++y) {
    for (int x = 0; x < lowres_mask.w; ++x) {
      if (!lowres_mask(x, y)) {
        exterior_points.insert(points.size());
        points.push_back(nacb::Vec2d((ox + x), (oy + y)));
      }
    }
  }

  // The delaunay triangulation flakes out if points are in regular grid.
  // Add some noise.
  for (auto& p : points) {
    p.x += 0.1 * ((double(rand()) / RAND_MAX) - 0.5);
    p.y += 0.1 * ((double(rand()) / RAND_MAX) - 0.5);
  }

  std::vector<nacb::Vec3<int>> tris = delaunay(points);
  std::vector<nacb::Vec3<int>> good_tris;
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
    points2.push_back(
        nacb::Vec2d(point.x / lowres_mask.w, 1.0 - point.y / lowres_mask.h));
  }

  mesh.tvert = points2;
  mesh.vert = points3;

  for (const auto& tri : tris) {
    if (exterior_points.count(tri[0]) || exterior_points.count(tri[1]) ||
        exterior_points.count(tri[2]))
      continue;
    if (interior_points.count(tri[0]) || interior_points.count(tri[1]) ||
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
  // Add extruded edges (should use edges that don't have a pair)
  for (const auto& tri : tris) {
    if (exterior_points.count(tri[0]) || exterior_points.count(tri[1]) ||
        exterior_points.count(tri[2]))
      continue;

    if (interior_points.count(tri[0]) || interior_points.count(tri[1]) ||
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
          face.vi[1] = v2 + num_points_keep;
          ;
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
  return BlowUpMesh(mesh, opts);
}

template <class T> struct VecHash {
  VecHash(const nacb::Vec3d& min_p, const nacb::Vec3d& max_p)
      : min_point(min_p), range(max_p - min_p) {}

  size_t operator()(const T& point) const {
    const Vec3<int> co = GetCoordinate(point);
    return co.z * (1024 * 1024) + co.y * 1024 + co.x;
  }
  Vec3<int> GetCoordinate(const nacb::Vec3d& point) const {
    nacb::Vec3d cp = point - min_point;
    return nacb::Vec3<int>(floor(cp.x / range.x * 1024.0),
                           floor(cp.y / range.y * 1024.0),
                           floor(cp.z / range.z * 1024.0));
  }
  nacb::Vec3d GetBinCenter(const nacb::Vec3<int>& co) const {
    return nacb::Vec3d((0.5 + co.x) / 1024.0 * range.x,
                       (0.5 + co.y) / 1024.0 * range.y,
                       (0.5 + co.z) / 1024.0 * range.z) +
           min_point;
  }
  nacb::Vec3d min_point;
  nacb::Vec3d range;
};

nappear::Mesh RemoveDuplicates(const std::vector<Vec3d>& points,
                               const std::vector<Vec3d>& normals,
                               const std::vector<gpoint3>& tri, int w, int h,
                               int d) {
  std::vector<Vec3d> new_points;
  std::vector<Vec3d> new_normals;
  nacb::Vec3d max_point(0, 0, 0);
  nacb::Vec3d min_point(0, 0, 0);
  for (const auto& v : points) {
    max_point = max_point.max(v);
    min_point = min_point.min(v);
  }
  max_point += nacb::Vec3d(0.1, 0.1, 0.1);

  VecHash<nacb::Vec3d> hash(min_point, max_point);
  std::unordered_multimap<nacb::Vec3d, int, VecHash<nacb::Vec3d>> mapper(100001,
                                                                         hash);
  std::unordered_map<int, int> index_map;
  for (int i = 0; i < points.size(); ++i) {
    const nacb::Vec3d& pt = points[i];
    int found = -1;
    Vec3<int> center = hash.GetCoordinate(pt);
    int checked = 0;
    auto check = [&](int dx, int dy, int dz) -> bool {
      for (auto el = mapper.find(
               hash.GetBinCenter(center + nacb::Vec3<int>(dx, dy, dz)));
           el != mapper.end(); ++el) {
        if ((new_points[el->second] - pt).len() < 1e-3) {
          found = el->second;
        }
      }
      return found >= 0;
    };
    if (check(0, 0, 0)) {
      for (int z = -1; z <= 1 && found < 0; z++) {
        for (int y = -1; y <= 1 && found < 0; y++) {
          for (int x = -1; x <= 1 && found < 0; x++) {
            check(x, y, z);
          }
        }
      }
    }
    if (found == -1) {
      auto co = hash.GetCoordinate(pt);
      mapper.insert(
          std::make_pair(hash.GetBinCenter(center), new_points.size()));
      index_map[i] = new_points.size();
      new_points.push_back(pt);
      new_normals.push_back(normals[i]);
    } else {
      index_map[i] = found;
    }
  }
  nappear::Mesh mesh;
  mesh.vert = new_points;
  mesh.norm = new_normals;
  for (const auto& v : mesh.vert) {
    mesh.tvert.push_back(nacb::Vec2d(v.x / w, 1.0 - v.y / h));
  }
  for (const auto& t : tri) {
    nappear::Mesh::Face face;
    face.vi[0] = index_map[t.x];
    face.vi[1] = index_map[t.y];
    face.vi[2] = index_map[t.z];
    face.tci[0] = index_map[t.x];
    face.tci[1] = index_map[t.y];
    face.tci[2] = index_map[t.z];
    face.ni[0] = index_map[t.x];
    face.ni[1] = index_map[t.y];
    face.ni[2] = index_map[t.z];
    mesh.faces.push_back(face);
  }
  return mesh;
}

nappear::Mesh CreateMeshFromImages3D(const nacb::Image8& lowres_image,
                                     const nacb::Image8& highres_image,
                                     const Color3b& bg_color,
                                     const BlowUpOptions& opts) {
  const int factor = highres_image.w / lowres_image.w;
  const int num_pad_highres = factor;

  nacb::Image8 lowres_mask = GetMask(lowres_image, bg_color);
  nacb::Image8 highres_mask = GetMask(highres_image, bg_color, num_pad_highres);

  nacb::Image8 highres_mask3d(highres_mask.w, highres_mask.h, 7);
  highres_mask3d = 0; // TODO(efficiency)
  for (int z = 2; z < highres_mask3d.nchannels - 2; ++z) {
    for (int y = 0; y < highres_mask.h; ++y) {
      for (int x = 0; x < highres_mask.w; ++x) {
        highres_mask3d(x, y, z) = highres_mask(x, y);
      }
    }
  }
  FullLevelSet3D levset(highres_mask3d);
  // levset.moveAlongCurvature(); // Smoothing the levelset is problematic for
  // small size inputs (e.g., text)

  const int num_pad_sample = 2;
  FullLevelSet3D levset_sample((lowres_mask.w + 2) * 2, (lowres_mask.h + 2) * 2,
                               highres_mask3d.nchannels);
  assert(levset_sample.getWidth() * 4 == highres_mask.w);
  assert(levset_sample.getHeight() * 4 == highres_mask.h);

  // The offsets here are very hack and require an upsample factor of 8
  for (int z = 0; z < highres_mask3d.nchannels; ++z) {
    for (int y = 0; y < highres_mask.h; ++y) {
      for (int x = 0; x < highres_mask.w; ++x) {
        levset_sample(x / 4, y / 4, z) += levset(x, y, z) / 16.0;
      }
    }
  }

  nappear::Mesh mesh;
  std::vector<gpoint3> tri;
  levset_sample.getTriangles(mesh.vert, mesh.norm, tri);
  for (auto& p : mesh.vert) {
    p.x = (p.x - num_pad_sample) * double(lowres_mask.w) /
              (levset_sample.getWidth() - 2 * num_pad_sample) +
          2.0 / lowres_mask.w;
    p.y = (p.y - num_pad_sample) * double(lowres_mask.h) /
              (levset_sample.getHeight() - 2 * num_pad_sample) +
          2.0 / lowres_mask.h;
    p.z = p.z - 3;
  }

  mesh = RemoveDuplicates(mesh.vert, mesh.norm, tri, lowres_mask.w,
                          lowres_mask.h, 7);

  std::vector<int> extra;
  for (int i = 0; i < mesh.vert.size(); ++i) {
    if (abs(mesh.vert[i].z) <= 0.5) {
      extra.push_back(i);
    }
  }
  auto original_vert = mesh.vert;

  SmoothMesh(&mesh);
  mesh.initNormals();

  const bool small_image = lowres_image.w <= 8 || lowres_image.h <= 8;
  BlowUpOptions opts2;
  opts2.alpha = small_image ? 0.07 : 0.185;
  opts2.ntime = small_image ? 10 : 20;
  opts2.alpha += 0.3;
  opts2.beta += 2;

  mesh = BlowUpMesh(mesh, opts2);

  if (extra.size() > 0) {
    Matrix xco(mesh.vert.size(), 1);
    Matrix yco(mesh.vert.size(), 1);
    Matrix zco(mesh.vert.size(), 1);

    std::vector<vertex_neigh_t> neigh = mesh_laplacian(mesh.vert, mesh.faces);

    for (int i = 0; i < mesh.vert.size(); i++) {
      xco[i] = mesh.vert[i].x;
      yco[i] = mesh.vert[i].y;
      zco[i] = mesh.vert[i].z;
    }
    const double cwt = 0.5;
    SparseMatrix mat = convertToSparse(neigh, extra, cwt);
    Matrix lx(mat.m, 1), ly(mat.m, 1), lz(mat.m, 1);

    lx = mat * xco;
    ly = mat * yco;
    lz = mat * zco;
    for (int i = 0; i < extra.size(); i++) {
      lx[i + mesh.vert.size()] = cwt * (original_vert[extra[i]].x);
      ly[i + mesh.vert.size()] = cwt * (original_vert[extra[i]].y);
      lz[i + mesh.vert.size()] = cwt * (original_vert[extra[i]].z);
    }

    Matrix xr = mat.linLeastSqQr(lx);
    Matrix yr = mat.linLeastSqQr(ly);
    Matrix zr = mat.linLeastSqQr(lz);

    for (int i = 0; i < mesh.vert.size(); i++) {
      mesh.vert[i] = Vec3f(xr[i], yr[i], zr[i]);
    }
  }
  return mesh;
}

#ifdef MESH_UTILS_MAIN
#include <nmisc/commandline.h>
int main(int ac, char* av[]) {
  int use_3d = 1;
  std::string lowres_path, highres_path,
      output_mesh_path = "/tmp/mesh_utils.obj";

  BlowUpOptions opts;

  nacb::CommandLine cline;
  cline.registerOption("lowres_path", "Low res image", &lowres_path);
  cline.registerOption("highres_path", "Highres image", &highres_path);
  cline.registerOption("output_mesh_path", "Path to the output mesh",
                       &output_mesh_path);
  cline.registerOption("use_3d", "Use the 3d option", &use_3d);

  cline.registerOption("ntime", "Number of time iterations (blow-up)",
                       &opts.ntime);
  cline.registerOption(
      "alpha", "(Blow-up) parameter, higher means more blow-up", &opts.alpha);
  cline.registerOption("beta",
                       "(Blow-up) parameter, highers means preserve structure",
                       &opts.beta);
  cline.registerOption(
      "ninner", "(Blow-up) parameter, inner structure preserving iterations",
      &opts.ninner);

  cline.parse(ac, av);

  nacb::Image8 lowres_image;
  nacb::Image8 highres_image;

  if (!lowres_image.read(lowres_path.c_str())) {
    std::cerr << "Unable to read lowres image from:" << lowres_path
              << std::endl;
    return -1;
  }
  if (!highres_image.read(highres_path.c_str())) {
    std::cerr << "Unable to read highres image from:" << highres_path
              << std::endl;
    return -1;
  }
  const Color3b bg_color(lowres_image(0, 0, 0), lowres_image(0, 0, 1),
                         lowres_image(0, 0, 2));
  nappear::Mesh mesh;
  if (use_3d) {
    mesh = CreateMeshFromImages3D(lowres_image, highres_image, bg_color, opts);
  } else {
    mesh = CreateMeshFromImages(lowres_image, highres_image, bg_color, opts);
  }
  mesh.saveObj(output_mesh_path.c_str());

  return 0;
}
#endif
