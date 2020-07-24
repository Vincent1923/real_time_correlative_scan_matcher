#ifndef PROBABILITY_GRID_H_
#define PROBABILITY_GRID_H_

#include <vector>

#include "map_limits.h"
#include "xy_index.h"

class ProbabilityGrid {
 public:
  ProbabilityGrid();

  const MapLimits& limits() const { return limits_; }

  float GetProbability(const Eigen::Array2i& cell_index) const;

 private:
  MapLimits limits_;  // 地图的范围
  std::vector<uint16> correspondence_cost_cells_;  // 记录各个栅格单元的空闲概率 p(free)
};

#endif  // PROBABILITY_GRID_H_
