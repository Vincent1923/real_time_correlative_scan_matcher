#ifndef MAP_LIMITS_H_
#define MAP_LIMITS_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "xy_index.h"
#include "point_cloud.h"
#include "rigid_transform.h"

#include "glog/logging.h"

class MapLimits {
 public:
  /**
   * @brief MapLimits    构造函数，主要是初始化成员变量
   * @param resolution   地图分辨率，程序中设置是0.05m，也就是5cm
   * @param max          这是一个浮点型二维向量，max_.x()和.y()分别表示x、y方向的最大值
   * @param cell_limits  栅格化后的 x 和 y 方向的最大范围，以 pixel 为单位
   * @return
   */
  MapLimits(const double resolution, const Eigen::Vector2d& max,
            const CellLimits& cell_limits)
      : resolution_(resolution), max_(max), cell_limits_(cell_limits) {
    CHECK_GT(resolution_, 0.);              // 检查 resolution_ 是否大于0
    CHECK_GT(cell_limits.num_x_cells, 0.);  // 检查 cell_limits.num_x_cells 是否大于0
    CHECK_GT(cell_limits.num_y_cells, 0.);  // 检查 cell_limits.num_y_cells 是否大于0
  }

  // 获取分辨率
  double resolution() const { return resolution_; }

  // 获取最大范围值
  const Eigen::Vector2d& max() const { return max_; }

  // 获取pixel坐标的最大范围
  const CellLimits& cell_limits() const { return cell_limits_; }

  // 返回“point”的单元格的索引，包含可能位于地图之外的“point”，例如，对于Contains()如果是负数或太大的索引，那么将返回false。
  // 给出一个point在Submap中的坐标，求其pixel坐标。
  Eigen::Array2i GetCellIndex(const Eigen::Vector2f& point) const {
    return Eigen::Array2i(
        RoundToInt((max_.y() - point.y()) / resolution_ - 0.5),
        RoundToInt((max_.x() - point.x()) / resolution_ - 0.5));
  }

  // 返回布尔型，判断所给pixel坐标是否大于0，小于等于最大值
  bool Contains(const Eigen::Array2i& cell_index) const {
    return (Eigen::Array2i(0, 0) <= cell_index).all() &&
           (cell_index <
            Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
               .all();
  }

 private:
  double resolution_;    // 地图的分辨率，即一个栅格单元对应的地图尺寸。程序中设置是0.05m，也就是5cm。
  Eigen::Vector2d max_;  // 这是一个浮点型二维向量，max_.x() 和 max_.y() 分别记录了地图的 x,y 方向的最大值。
  /**
   * 1. x 和 y 方向上的栅格数量。
   *    在 MapLimits 中根据最大范围 max_ 和分辨率 resolution 就可以创建 cell_limits_。
   * 2. 数据类型 CellLimits 定义在 "/mapping/2d/xy_index.h" 中，它是一个结构体，包括两个 int 型成员变量：
   *    num_x_cells 是 x 轴上的栅格数量，num_y_cells 是 y 轴上的栅格数量。
   */
  CellLimits cell_limits_;
};

#endif  // MAP_LIMITS_H_
