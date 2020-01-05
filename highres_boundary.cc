#include "highres_boundary.h"

#include "extract_colors.h"
#include <limits>
#include <nmath/sparsematrix.h>
#include <nmisc/commandline.h>

namespace {
nacb::Imagef GaussianFilter(const nacb::Imagef& upsampled, double filter_sigma,
                            int filter_half_size) {
  nacb::Imagef filter(filter_half_size * 2 + 1, 1,
                      1); // 2 * filter_half_size + 1, 1);
  double filter_sum = 0;
  for (int i = 0; i < filter.w; ++i) {
    double x = i - (filter.w / 2.0);
    filter[i] = exp(-0.5 * x * x / filter_sigma / filter_sigma);
    filter_sum += filter[i];
  }

  filter *= 1.0 / filter_sum;

  nacb::Imagef smoothed(1, 1, 1);
  smoothed = upsampled.convolve(filter);
  std::swap(filter.h, filter.w);
  smoothed = smoothed.convolve(filter);
  for (int y = 0; y < smoothed.h; ++y) {
    for (int x = 0; x < smoothed.w; ++x) {
      for (int c = 0; c < smoothed.nchannels; ++c) {
        smoothed(x, y, c) = std::max(0.0f, std::min(1.0f, smoothed(x, y, c)));
      }
    }
  }
  return smoothed;
}
} // namespace

template <class T>
nacb::Image<T> PadImage(const nacb::Image<T>& image, int pad_size) {
  nacb::Image<T> padded(image.w + 2 * pad_size, image.h + 2 * pad_size,
                        image.nchannels);
  padded = 0;
  for (int y = 0; y < image.h; ++y) {
    for (int x = 0; x < image.w; ++x) {
      for (int c = 0; c < image.nchannels; ++c) {
        padded(x + pad_size, y + pad_size, c) = image(x, y, c);
      }
    }
  }
  return padded;
}

nacb::Imagef HighresBoundarySimple(const nacb::Image8& image,
                                   const int factor) {
  nacb::Image8 upsampled;
  const int pad_size = 4;
  upsampled = PadImage(image, pad_size)
                  .resize((image.w + 2 * pad_size) * factor,
                          (image.h + 2 * pad_size) * factor);
  const double filter_sigma = factor / 3.0;
  const int filter_half_size = std::max(1, factor * 3);
  nacb::Image8 filtered =
      GaussianFilter(upsampled, filter_sigma, filter_half_size);
  return filtered.subimage(pad_size * factor - factor / 2,
                           pad_size * factor - factor / 2, image.w * factor,
                           image.h * factor);
}

// There is lots of hacking in here, and it is not clear that it really helped.
nacb::Imagef HighresBoundary(const nacb::Image8& image, const int factor) {
  nacb::Imagef upsampled;
  upsampled = image.resize(image.w * factor, image.h * factor);

  const double filter_sigma = factor / 3.0;
  const int filter_half_size = std::max(1, factor * 3);

  nacb::Imagef smoothed =
      GaussianFilter(upsampled, filter_sigma, filter_half_size);
  smoothed.write("/tmp/smoothed.png");
  (smoothed > 0.5).write("/tmp/smoothed_thresh.png");

  nacb::Image8 mask(smoothed.w, smoothed.h, 1);
  const float eps = 1.0 / 255.0;
  mask = 0;

  nacb::Image<uint32_t> index(smoothed.w, smoothed.h, 1);
  std::vector<std::pair<int, int>> vars;
  index = -1;

  int num_mask = 0;
  for (int y = 0; y < smoothed.h; ++y) {
    for (int x = 0; x < smoothed.w; ++x) {
      if (((x % factor) == 0) && ((y % factor) == 0)) {
        smoothed(x, y, 0) =
            image((x + factor / 2) / factor, (y + factor / 2) / factor) > 128;
        ;
        continue;
      }

      if ((smoothed(x, y, 0) > eps) && (smoothed(x, y, 0) < 1.0 - eps)) {
        mask(x, y, 0) = 255;
        index(x, y, 0) = num_mask;
        vars.push_back(std::make_pair(x, y));
        num_mask++;
      }
    }
  }

  nacb::MappedSparseMatrix M(vars.size(), vars.size());
  nacb::Matrix b = nacb::Matrix::zeros(vars.size(), 1);
  nacb::Imagef lap_filter(3, 3, 1);
  lap_filter = 0;
  lap_filter(0, 1) = 1;
  lap_filter(1, 1) = -4;
  lap_filter(2, 1) = 1;
  lap_filter(1, 0) = 1;
  lap_filter(1, 2) = 1;
  nacb::Imagef lap;
  lap = smoothed.convolve(lap_filter);

  for (int i = 0; i < vars.size(); ++i) {
    int neigh[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    int x = vars[i].first;
    int y = vars[i].second;

    double xd = factor * floor(x / factor) - x;
    double yd = factor * floor(y / factor) - y;
    double r2 = xd * xd + yd * yd;
    double sigma = factor / 2.0;
    double wt = 1; // 0.9*exp(-r2 / (sigma*sigma) / 2.0) + 0.1;
    double center = 0;
    std::vector<std::pair<int, double>> col;
    for (int ni = 0; ni < 4; ++ni) {
      const int xn = x + neigh[ni][0];
      const int yn = y + neigh[ni][1];
      if (xn < 0 || yn < 0 || xn >= upsampled.w || yn >= upsampled.h) {
        continue;
      } else {
        double g = wt;
        if (index(xn, yn) < std::numeric_limits<uint32_t>::max()) {
          M(i, index(xn, yn)) = g;
        } else {
          b[i] -= image(xn / factor, yn / factor, 0) >
                  128; //  (image(xn / factor, yn / factor, 0) > 128) * g;
        }
        center += g;
      }
    }
    double prior = 0;
    b[i] += prior * (image(x / factor, y / factor, 0) > 128) + 0.5 * lap(x, y);
    M(i, i) = -center + prior;
  }
  nacb::SparseMatrix S = M;
  nacb::Matrix x = nacb::SparseMatrix::umfsolve(S, b);
  smoothed = smoothed > 0.5;
  for (int i = 0; i < vars.size(); ++i) {
    double val = std::max(0.0, std::min(1.0, x[i]));
    smoothed(vars[i].first, vars[i].second, 0) = val;
    smoothed(vars[i].first, vars[i].second, 1) = val;
    smoothed(vars[i].first, vars[i].second, 2) = val;
  }
  nacb::Imagef downsampled(image.w, image.h, 1);
  downsampled = 0;
  for (int y = 0; y < downsampled.h; ++y) {
    for (int x = 0; x < downsampled.w; ++x) {
      downsampled(x, y, 0) = smoothed(x * factor, y * factor, 0) > 0;
    }
  }
  printf("num_mask: %d, %f\n", num_mask, sqrt(num_mask));
  mask.write("/tmp/mask.png");
  smoothed.write("/tmp/soln_raw.png");
  (smoothed > 0.45).write("/tmp/soln.png");
  downsampled.write("/tmp/soln_ds.png");
  upsampled.write("/tmp/upsampled.png");
  (upsampled > 0.5).write("/tmp/upsampled_thresh.png");
  return smoothed;
}

nacb::Imagef HighresBoundaryBinary(
    const nacb::Image8& image,
    std::function<nacb::Imagef(const nacb::Image8&, int factor)> upsample,
    int factor, const Color3b& bg) {
  nacb::Imagef highres;

  int fg_index = 0;
  auto color_masks = ExtractColorMasks(image);
  for (int i = 0; i < color_masks.size(); ++i) {
    auto& color_mask = color_masks[i];
    const Color3b color = color_mask.first;
    if (!(color == bg)) {
      highres = upsample(color_mask.second, factor);
      fg_index = i;
    }
  }

  nacb::Image8 final(highres.w, highres.h, 3);
  for (int y = 0; y < final.h; ++y) {
    for (int x = 0; x < final.w; ++x) {
      const auto& color =
          color_masks[highres(x, y) > 0.5 ? fg_index : !fg_index].first;
      final(x, y, 0) = color.x;
      final(x, y, 1) = color.y;
      final(x, y, 2) = color.z;
    }
  }
  return final;
}

nacb::Image8 HighresBoundaryColor(
    const nacb::Image8& image,
    std::function<nacb::Imagef(const nacb::Image8&, int factor)> upsample,
    int factor, const Color3b* bg) {
  std::vector<nacb::Imagef> highres_masks;
  int bg_index = -1;
  auto color_masks = ExtractColorMasks(image);

  int index = 0;
  for (const auto& color_mask : color_masks) {
    const auto& color = color_mask.first;
    if (bg && color == *bg) {
      bg_index = index;
    }
    highres_masks.push_back(upsample(color_mask.second, factor));
    index++;
  }
  nacb::Image8 final(highres_masks[0].w, highres_masks[0].h,
                     3 + (bg_index >= 0));
  for (int y = 0; y < final.h; ++y) {
    for (int x = 0; x < final.w; ++x) {
      int maxi = 0;
      for (int i = 0; i < highres_masks.size(); ++i) {
        if (i == bg_index)
          continue;
        if (highres_masks[i](x, y, 0) > highres_masks[maxi](x, y, 0) ||
            maxi == bg_index) {
          maxi = i;
        }
      }
      const auto& color = color_masks[maxi].first;
      final(x, y, 0) = color.x;
      final(x, y, 1) = color.y;
      final(x, y, 2) = color.z;
      if (bg_index >= 0) {
        final(x, y, 3) =
            (highres_masks[maxi](x, y, 0) > highres_masks[bg_index](x, y, 0))
                ? 255
                : 1;
      }
    }
  }
  return final;
}

#ifdef HIGHRES_BOUNDARY_MAIN
int main(int ac, char* av[]) {
  nacb::Image8 image(av[1]);
  const int factor = atoi(av[2]);
  if (ac > 3 && atoi(av[3])) {
    Color3b bg_color{image(0, 0, 0), image(0, 0, 1), image(0, 0, 2)};
    nacb::Image8 final =
        HighresBoundaryColor(image, HighresBoundarySimple, factor, &bg_color);
    final.write("/tmp/final.png");
  } else if (ac > 3 && atoi(av[3]) == 0) {
    Color3b bg_color{image(0, 0, 0), image(0, 0, 1), image(0, 0, 2)};
    nacb::Image8 final =
        HighresBoundaryBinary(image, HighresBoundarySimple, factor, bg_color);
    final.write("/tmp/final.png");
  } else {
    HighresBoundary(image, factor);
  }
  return 0;
}
#endif
