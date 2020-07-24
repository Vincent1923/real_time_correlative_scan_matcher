#ifndef REAL_TIME_CORRELATIVE_SCAN_MATCHER_2D_H_
#define REAL_TIME_CORRELATIVE_SCAN_MATCHER_2D_H_

#include <iostream>
#include <memory>
#include <vector>

#include "Eigen/Core"

#include "correlative_scan_matcher_2d.h"
#include "probability_grid.h"

class RealTimeCorrelativeScanMatcher2D {
 public:
  RealTimeCorrelativeScanMatcher2D() {};

  double Match(const transform::Rigid2d& initial_pose_estimate,
               const PointCloud& point_cloud,
               const ProbabilityGrid& probability_grid,
               transform::Rigid2d* pose_estimate) const;

  void ScoreCandidates(const ProbabilityGrid& probability_grid,
                       const std::vector<DiscreteScan2D>& discrete_scans,
                       const SearchParameters& search_parameters,
                       std::vector<Candidate2D>* candidates) const;

 private:
  std::vector<Candidate2D> GenerateExhaustiveSearchCandidates(
      const SearchParameters& search_parameters) const;


};

#endif  // REAL_TIME_CORRELATIVE_SCAN_MATCHER_2D_H_