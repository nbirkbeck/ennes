#ifndef MESH_UTILS_H
#define MESH_UTILS_H 1

#include "nes.h"
#include <nappear/mesh.h>
#include <nimage/image.h>
#include <nmath/vec3.h>

struct BlowUpOptions {
  int ntime = 30;
  double alpha = 0.1; // Controls blow-up of the mesh.
  double beta = 1.0;
  int ninner = 15; // Number of inner iterations (to reduce strain/stretch).
  int constrain_boundary = 1;
};

// Creates a mesh from a lowres and highres image. If the highres image
// doesn't have an alpha channel, then the bg_color is used to create
// the mask. This version uses a 2D levelset to extract the boundary
// followed by delaunay triangulation of interior points (doesn't
// work that great).
nappear::Mesh
CreateMeshFromImages(const nacb::Image8& lowres_image,
                     const nacb::Image8& highres_image, const Color3b& bg_color,
                     const BlowUpOptions& options = BlowUpOptions());

// Same as CreateMeshFromImages, except it uses a 3D levelset to create
// the mesh. Much more reliable and consistent mesh sampling.
nappear::Mesh CreateMeshFromImages3D(
    const nacb::Image8& lowres_image, const nacb::Image8& highres_image,
    const Color3b& bg_color, const BlowUpOptions& options = BlowUpOptions());
#endif
