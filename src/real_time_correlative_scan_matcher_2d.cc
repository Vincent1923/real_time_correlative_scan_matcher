#include "real_time_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "Eigen/Geometry"

#include "common/math.h"
#include "probability_grid.h"
#include "point_cloud.h"

#include "glog/logging.h"

std::vector<Candidate2D>
RealTimeCorrelativeScanMatcher2D::GenerateExhaustiveSearchCandidates(
    const SearchParameters& search_parameters) const {
  // 计算总的候选数量 num_candidates
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + 1);
    const int num_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + 1);
    num_candidates += num_linear_x_candidates * num_linear_y_candidates;
  }
  // 临时容器 candidates 为返回的全部候选
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         ++x_index_offset) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           ++y_index_offset) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

double RealTimeCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const PointCloud& point_cloud,
    const ProbabilityGrid& probability_grid,
    transform::Rigid2d* pose_estimate) const {
  // 获取原始位姿估计的 2D 旋转，并首先对输入的扫描数据进行旋转变换
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const PointCloud rotated_point_cloud = TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
  // 定义 SearchParameters 对象描述线性搜索窗口的像素偏移
  const SearchParameters search_parameters(0.1, 20, rotated_point_cloud, 0.05);

  // 函数 GenerateRotatedScans() 根据角度搜索窗口，生成每一个搜索角度下经过旋转变换的扫描数据 rotated_scans，
  // 返回容器中扫描数据的数量为 search_parameters.num_scans。
  // 然后再利用函数 DiscretizeScans() 对经过旋转后的扫描数据进行离散化，得到每一个搜索角度下的扫描数据在栅格地图中的像素坐标 discrete_scans。
  const std::vector<PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      probability_grid.limits(), rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  // 生成详细的搜索候选 candidates，搜索候选由角度搜索索引，x 轴和 y 轴搜索索引共同确定。
  // 然后计算每一个搜索候选的得分。
  std::vector<Candidate2D> candidates =
      GenerateExhaustiveSearchCandidates(search_parameters);
  ScoreCandidates(probability_grid, discrete_scans, search_parameters,
                  &candidates);

  // 最后选出得分最高的候选，对初始位姿进行优化，并返回最高候选得分。
  const Candidate2D& best_candidate =
      *std::max_element(candidates.begin(), candidates.end());
  *pose_estimate = transform::Rigid2d(
      {initial_pose_estimate.translation().x() + best_candidate.x,
       initial_pose_estimate.translation().y() + best_candidate.y},
      initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
  return best_candidate.score;
}

void RealTimeCorrelativeScanMatcher2D::ScoreCandidates(
    const ProbabilityGrid& probability_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  for (Candidate2D& candidate : *candidates) {
    candidate.score = 0.f;
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      const float probability =
          probability_grid.GetProbability(proposed_xy_index);
      candidate.score += probability;
    }
    candidate.score /=
        static_cast<float>(discrete_scans[candidate.scan_index].size());
    candidate.score *=
        std::exp(-common::Pow2(std::hypot(candidate.x, candidate.y) * 0.1 +
                               std::abs(candidate.orientation) * 0.1));
    CHECK_GT(candidate.score, 0.f);
  }
}
