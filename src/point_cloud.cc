#include "point_cloud.h"

PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform) {
  PointCloud result;
  result.reserve(point_cloud.size());
  for (const Eigen::Vector3f& point : point_cloud) {
    result.emplace_back(transform * point);
  }
  return result;
}