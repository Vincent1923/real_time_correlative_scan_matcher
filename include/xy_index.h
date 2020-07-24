#ifndef XY_INDEX_H_
#define XY_INDEX_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>

#include "Eigen/Core"

struct CellLimits {
  CellLimits() = default;
  CellLimits(int init_num_x_cells, int init_num_y_cells)
      : num_x_cells(init_num_x_cells), num_y_cells(init_num_y_cells) {}

  int num_x_cells = 0;  // x方向划分的栅格数，也是pixel坐标情况下的最大范围
  int num_y_cells = 0;  // y方向划分的栅格数
};

#endif  // XY_INDEX_H_
