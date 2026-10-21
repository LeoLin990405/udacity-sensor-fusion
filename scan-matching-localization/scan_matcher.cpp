#include "scan_matcher.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#include <cmath>

namespace {

constexpr float kVoxelLeafSize = 0.5f;
constexpr float kTransformationEpsilon = 0.0001f;
constexpr float kNdtStepSize = 1.0f;
constexpr float kNdtResolution = 1.0f;
constexpr int kMaximumIterations = 35;
constexpr float kMaximumFrameTranslation = 5.0f;
constexpr float kMaximumFrameRotation = 0.5f;
constexpr std::size_t kMinimumFilteredPoints = 100;

bool IsPlausibleUpdate(const Eigen::Matrix4f& initial_guess,
                       const Eigen::Matrix4f& candidate) {
  if (!candidate.allFinite()) {
    return false;
  }

  const Eigen::Matrix4f delta = initial_guess.inverse() * candidate;
  const float translation = delta.block<3, 1>(0, 3).norm();
  const float yaw = std::abs(std::atan2(delta(1, 0), delta(0, 0)));
  return translation <= kMaximumFrameTranslation &&
         yaw <= kMaximumFrameRotation;
}

}  // namespace

ScanMatcher::ScanMatcher(const ScanCloud::ConstPtr& map_cloud)
    : map_cloud_(map_cloud) {
  ndt_.setTransformationEpsilon(kTransformationEpsilon);
  ndt_.setStepSize(kNdtStepSize);
  ndt_.setResolution(kNdtResolution);
  ndt_.setMaximumIterations(kMaximumIterations);
  if (map_cloud_) {
    ndt_.setInputTarget(map_cloud_);
  }
}

MatchResult ScanMatcher::Match(const ScanCloud::ConstPtr& scan_cloud,
                               const Eigen::Matrix4f& initial_guess) {
  MatchResult result;
  result.transform = initial_guess;

  ScanCloud::Ptr filtered_cloud(new ScanCloud);
  pcl::VoxelGrid<ScanPoint> voxel_filter;
  voxel_filter.setInputCloud(scan_cloud);
  voxel_filter.setLeafSize(kVoxelLeafSize, kVoxelLeafSize, kVoxelLeafSize);
  voxel_filter.filter(*filtered_cloud);
  result.filtered_point_count = filtered_cloud->points.size();

  if (!map_cloud_ || map_cloud_->empty() ||
      result.filtered_point_count < kMinimumFilteredPoints) {
    pcl::transformPointCloud(
        *filtered_cloud, *result.aligned_cloud, initial_guess);
    return result;
  }

  ndt_.setInputSource(filtered_cloud);

  ScanCloud aligned_candidate;
  ndt_.align(aligned_candidate, initial_guess);
  result.converged = ndt_.hasConverged();
  result.fitness_score = ndt_.getFitnessScore();

  const Eigen::Matrix4f candidate = ndt_.getFinalTransformation();
  result.accepted = result.converged &&
                    IsPlausibleUpdate(initial_guess, candidate);
  if (result.accepted) {
    result.transform = candidate;
    *result.aligned_cloud = aligned_candidate;
  } else {
    pcl::transformPointCloud(
        *filtered_cloud, *result.aligned_cloud, initial_guess);
  }

  return result;
}
