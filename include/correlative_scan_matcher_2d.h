#ifndef CORRELATIVE_SCAN_MATCHER_2D_H_
#define CORRELATIVE_SCAN_MATCHER_2D_H_

#include <vector>

#include "Eigen/Core"

#include "map_limits.h"
#include "xy_index.h"
#include "point_cloud.h"

typedef std::vector<Eigen::Array2i> DiscreteScan2D;

struct SearchParameters {
  struct LinearBounds {
    int min_x;
    int max_x;
    int min_y;
    int max_y;
  };

  SearchParameters(double linear_search_window, double angular_search_window,
                   const PointCloud& point_cloud, double resolution);

  int num_angular_perturbations;
  double angular_perturbation_step_size;
  double resolution;
  int num_scans;
  std::vector<LinearBounds> linear_bounds;  // Per rotated scans.
};

// Generates a collection of rotated scans.
std::vector<PointCloud> GenerateRotatedScans(
    const PointCloud& point_cloud,
    const SearchParameters& search_parameters);

// Translates and discretizes the rotated scans into a vector of integer
// indices.
std::vector<DiscreteScan2D> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<PointCloud>& scans,
    const Eigen::Translation2f& initial_translation);

// A possible solution.
struct Candidate2D {
  Candidate2D(const int init_scan_index, const int init_x_index_offset,
              const int init_y_index_offset,
              const SearchParameters& search_parameters)
      : scan_index(init_scan_index),
        x_index_offset(init_x_index_offset),
        y_index_offset(init_y_index_offset),
        x(-y_index_offset * search_parameters.resolution),
        y(-x_index_offset * search_parameters.resolution),
        orientation((scan_index - search_parameters.num_angular_perturbations) *
                    search_parameters.angular_perturbation_step_size) {}

  // Index into the rotated scans vector.
  int scan_index = 0;  // 角度的搜索索引 jθ

  // Linear offset from the initial pose.
  int x_index_offset = 0;  // x轴的搜索索引 jx
  int y_index_offset = 0;  // y轴的搜索索引 jy

  // Pose of this Candidate2D relative to the initial pose.
  double x = 0.;  // 相对于初始位姿的x偏移量 rjx
  double y = 0.;  // 相对于初始位姿的y偏移量 rjy
  double orientation = 0.;  // 相对于初始位姿的角度偏移量 δθjθ

  // Score, higher is better.
  float score = 0.f;  // 候选点的评分，越高越好

  // 定义了两个比较操作符的重载用于方便比较候选点的优劣
  bool operator<(const Candidate2D& other) const { return score < other.score; }
  bool operator>(const Candidate2D& other) const { return score > other.score; }
};

#endif  // CORRELATIVE_SCAN_MATCHER_2D_H_
