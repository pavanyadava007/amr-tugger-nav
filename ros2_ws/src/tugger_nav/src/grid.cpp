#include "tugger_nav/grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tugger_nav {

void Grid::toCell(double x, double y, int & ix, int & iy) const {
  ix = static_cast<int>(std::floor((x - origin_x) / resolution));
  iy = static_cast<int>(std::floor((y - origin_y) / resolution));
}

bool Grid::occAt(double x, double y) const {
  int ix = 0;
  int iy = 0;
  toCell(x, y, ix, iy);
  return occ(ix, iy);
}

double raycast(const Grid & g, double x, double y, double angle, double max_range) {
  const double dx = std::cos(angle);
  const double dy = std::sin(angle);
  const double gx = (x - g.origin_x) / g.resolution;
  const double gy = (y - g.origin_y) / g.resolution;
  int ix = static_cast<int>(std::floor(gx));
  int iy = static_cast<int>(std::floor(gy));
  if (g.occ(ix, iy)) {
    return 0.0;
  }
  const int step_x = dx > 0 ? 1 : -1;
  const int step_y = dy > 0 ? 1 : -1;
  const double inf = std::numeric_limits<double>::infinity();
  // Ray parameter (in cells) to the next vertical / horizontal cell border, and per-cell increment.
  double t_max_x = dx != 0.0 ? ((dx > 0 ? (ix + 1 - gx) : (gx - ix)) / std::abs(dx)) : inf;
  double t_max_y = dy != 0.0 ? ((dy > 0 ? (iy + 1 - gy) : (gy - iy)) / std::abs(dy)) : inf;
  const double t_dx = dx != 0.0 ? 1.0 / std::abs(dx) : inf;
  const double t_dy = dy != 0.0 ? 1.0 / std::abs(dy) : inf;
  const double t_end = max_range / g.resolution;
  while (true) {
    double t = 0.0;
    if (t_max_x < t_max_y) {
      t = t_max_x;
      t_max_x += t_dx;
      ix += step_x;
    } else {
      t = t_max_y;
      t_max_y += t_dy;
      iy += step_y;
    }
    if (t > t_end) {
      return max_range;
    }
    if (g.occ(ix, iy)) {
      return t * g.resolution;
    }
  }
}

namespace {
// 1D squared distance transform of f (lower envelope of parabolas).
void edt1d(const std::vector<float> & f, std::vector<float> & d, std::vector<int> & v, std::vector<float> & z) {
  const int n = static_cast<int>(f.size());
  const float inf = std::numeric_limits<float>::infinity();
  int k = 0;
  v[0] = 0;
  z[0] = -inf;
  z[1] = inf;
  for (int q = 1; q < n; ++q) {
    if (f[q] == inf) {
      continue;
    }
    if (f[v[k]] == inf) {  // envelope only holds an infinite parabola so far: replace it
      v[k] = q;
      continue;
    }
    float s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0f * (q - v[k]));
    while (k > 0 && s <= z[k]) {
      --k;
      s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0f * (q - v[k]));
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = inf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {
      ++k;
    }
    const float diff = static_cast<float>(q - v[k]);
    d[q] = f[v[k]] == inf ? inf : diff * diff + f[v[k]];
  }
}
}  // namespace

std::vector<float> distanceTransform(const Grid & g) {
  const float inf = std::numeric_limits<float>::infinity();
  const int w = g.width;
  const int h = g.height;
  std::vector<float> out(static_cast<size_t>(w) * h, inf);
  for (size_t i = 0; i < out.size(); ++i) {
    out[i] = g.occupied[i] ? 0.0f : inf;
  }
  const int n = std::max(w, h);
  std::vector<float> f(n);
  std::vector<float> d(n);
  std::vector<int> v(n);
  std::vector<float> z(n + 1);
  // Columns, then rows.
  f.resize(h);
  d.resize(h);
  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) {
      f[y] = out[static_cast<size_t>(y) * w + x];
    }
    edt1d(f, d, v, z);
    for (int y = 0; y < h; ++y) {
      out[static_cast<size_t>(y) * w + x] = d[y];
    }
  }
  f.resize(w);
  d.resize(w);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      f[x] = out[static_cast<size_t>(y) * w + x];
    }
    edt1d(f, d, v, z);
    for (int x = 0; x < w; ++x) {
      out[static_cast<size_t>(y) * w + x] = std::sqrt(d[x]) * static_cast<float>(g.resolution);
    }
  }
  return out;
}

}  // namespace tugger_nav
