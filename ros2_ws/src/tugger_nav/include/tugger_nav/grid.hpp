// Occupancy grid helpers used by the simulator: ray casting and a Euclidean distance transform.
#pragma once
#include <cstdint>
#include <vector>

namespace tugger_nav {

struct Grid {
  int width = 0;
  int height = 0;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<uint8_t> occupied;  // row-major, row 0 = lowest y; 1 = occupied

  bool inside(int ix, int iy) const { return ix >= 0 && iy >= 0 && ix < width && iy < height; }
  // Cells outside the map count as occupied.
  bool occ(int ix, int iy) const { return !inside(ix, iy) || occupied[static_cast<size_t>(iy) * width + ix] != 0; }
  bool occAt(double x, double y) const;
  void toCell(double x, double y, int & ix, int & iy) const;
};

// Distance from (x, y) along heading `angle` to the first occupied cell, at most max_range.
// Amanatides-Woo grid traversal, exact for axis-aligned cells.
double raycast(const Grid & g, double x, double y, double angle, double max_range);

// Exact Euclidean distance transform (Felzenszwalb and Huttenlocher 2012): distance in metres from each
// cell centre to the nearest occupied cell centre (0 for occupied cells).
std::vector<float> distanceTransform(const Grid & g);

}  // namespace tugger_nav
