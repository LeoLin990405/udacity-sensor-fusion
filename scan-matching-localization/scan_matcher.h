#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include <Eigen/Core>

#include <cstddef>
#include <limits>

using ScanPoint = pcl::PointXYZ;
using ScanCloud = pcl::PointCloud<ScanPoint>;

struct MatchResult {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  ScanCloud::Ptr aligned_cloud{new ScanCloud};
  bool converged = false;
  bool accepted = false;
  double fitness_score = std::numeric_limits<double>::infinity();
  std::size_t filtered_point_count = 0;
};

class ScanMatcher {
 public:
  explicit ScanMatcher(const ScanCloud::ConstPtr& map_cloud);

  MatchResult Match(const ScanCloud::ConstPtr& scan_cloud,
                    const Eigen::Matrix4f& initial_guess);

 private:
  ScanCloud::ConstPtr map_cloud_;
  pcl::NormalDistributionsTransform<ScanPoint, ScanPoint> ndt_;
};
