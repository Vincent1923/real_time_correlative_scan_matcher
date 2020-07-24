#include "correlative_scan_matcher_2d.h"

#include <cmath>

#include "common/math.h"

SearchParameters::SearchParameters(const double linear_search_window,
                                   const double angular_search_window,
                                   const PointCloud& point_cloud,
                                   const double resolution)
    : resolution(resolution) {
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  // 计算一帧扫描数据的最远数据点的距离 max_scan_range
  float max_scan_range = 3.f * resolution;
  for (const Eigen::Vector3f& point : point_cloud) {
    const float range = point.head<2>().norm();
    max_scan_range = std::max(range, max_scan_range);
  }
  // 计算角度搜索步长 angular_perturbation_step_size，并通过输入参数的角度搜索窗口 angular_search_window，
  // 计算在一个方向上的栅格的角度搜索数量 num_angular_perturbations，最后再计算总的角度搜索数量 num_scans。
  // 这里是通过余弦定理计算的角度步长，三角形为等腰三角形，其中两条边为 max_scan_range，
  // 角度步长对应的边长为分辨率 resolution。
  const double kSafetyMargin = 1. - 1e-3;
  angular_perturbation_step_size =
      kSafetyMargin * std::acos(1. - common::Pow2(resolution) /
                                         (2. * common::Pow2(max_scan_range)));
  num_angular_perturbations =
      std::ceil(angular_search_window / angular_perturbation_step_size);
  num_scans = 2 * num_angular_perturbations + 1;

  // 通过输入参数的线性搜索窗口 linear_search_window，计算栅格的线性搜索数量 num_linear_perturbations
  const int num_linear_perturbations =
      std::ceil(linear_search_window / resolution);
  // 为 linear_bounds 分配总的角度搜索数量的空间。
  // 实际上是为每一个搜索角度确定线性搜索空间的像素范围。
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

std::vector<PointCloud> GenerateRotatedScans(
    const PointCloud& point_cloud,
    const SearchParameters& search_parameters) {
  // 临时变量 rotated_scans 保存扫描数据在每一个搜索角度下的变换结果，
  // 所以这样就完成了搜索角度的遍历。
  std::vector<PointCloud> rotated_scans;
  rotated_scans.reserve(search_parameters.num_scans);

  // 计算最小搜索角度 delta_theta，然后根据搜索步长遍历所有的搜索角度，再根据搜索角度对扫描数据进行旋转变换，
  // 最后保存在容器 rotated_scans 中并返回。
  double delta_theta = -search_parameters.num_angular_perturbations *
                       search_parameters.angular_perturbation_step_size;
  for (int scan_index = 0; scan_index < search_parameters.num_scans;
       ++scan_index,
           delta_theta += search_parameters.angular_perturbation_step_size) {
    rotated_scans.push_back(TransformPointCloud(
        point_cloud, transform::Rigid3f::Rotation(Eigen::AngleAxisf(
                         delta_theta, Eigen::Vector3f::UnitZ()))));
  }
  return rotated_scans;
}

std::vector<DiscreteScan2D> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<PointCloud>& scans,
    const Eigen::Translation2f& initial_translation) {
  // 临时容器 discrete_scans 保存返回结果，并为它分配总的角度搜索数量的大小。
  std::vector<DiscreteScan2D> discrete_scans;
  discrete_scans.reserve(scans.size());
  // for 循环获取扫描数据在每一个搜索角度下经过旋转变换后的数据，
  // 然后通过机器人初始位置估计 initial_translation，把经过旋转变换后的扫描数据通过平移变换到局部地图坐标系下。
  // 接下来对扫描数据点进行离散化处理，离散化主要是计算扫描数据点在栅格地图中的像素坐标，
  // 这样最后就可以得到一帧扫描数据在栅格地图（子图）中的像素坐标。
  for (const PointCloud& scan : scans) {
    discrete_scans.emplace_back();
    discrete_scans.back().reserve(scan.size());
    for (const Eigen::Vector3f& point : scan) {
      const Eigen::Vector2f translated_point =
          Eigen::Affine2f(initial_translation) * point.head<2>();
      discrete_scans.back().push_back(
          map_limits.GetCellIndex(translated_point));
    }
  }
  // 最后返回离散化的扫描数据
  return discrete_scans;
}
