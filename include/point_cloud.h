#ifndef POINT_CLOUD_H_
#define POINT_CLOUD_H_

#include <vector>

#include "Eigen/Core"

#include "rigid_transform.h"

typedef std::vector<Eigen::Vector3f> PointCloud;

PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform);

#endif  // POINT_CLOUD_H_
