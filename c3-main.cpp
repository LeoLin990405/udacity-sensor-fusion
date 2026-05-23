/*
 * Udacity Self Driving Car Engineer
 * Scan Matching Localization project implementation.
 *
 * This file contains the project TODO solution that can replace the TODO block
 * in the Udacity /home/workspace/c3-project/c3-main.cpp starter file.  The
 * approach uses a voxel filter for each incoming lidar scan, then applies NDT
 * with the previous pose as an initial guess.  If NDT does not converge well,
 * a short ICP refinement is used as a fallback.  The resulting transform is
 * converted into the estimated ego pose and is also used to render the aligned
 * scan in the map frame.
 */

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace {

Eigen::Matrix4f pose_to_transform(const Pose& pose) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  const float roll = static_cast<float>(pose.rotation.x);
  const float pitch = static_cast<float>(pose.rotation.y);
  const float yaw = static_cast<float>(pose.rotation.z);

  Eigen::AngleAxisf roll_angle(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yaw_angle(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Matrix3f rotation = (yaw_angle * pitch_angle * roll_angle).matrix();

  transform.block<3, 3>(0, 0) = rotation;
  transform(0, 3) = static_cast<float>(pose.position.x);
  transform(1, 3) = static_cast<float>(pose.position.y);
  transform(2, 3) = static_cast<float>(pose.position.z);

  return transform;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr filter_scan(
    const typename pcl::PointCloud<PointT>::ConstPtr& scan_cloud,
    float leaf_size) {
  auto filtered_scan = typename pcl::PointCloud<PointT>::Ptr(
      new pcl::PointCloud<PointT>());

  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud(scan_cloud);
  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid.filter(*filtered_scan);

  return filtered_scan;
}

template <typename PointT>
Eigen::Matrix4f estimate_pose_transform(
    const typename pcl::PointCloud<PointT>::ConstPtr& filtered_scan,
    const typename pcl::PointCloud<PointT>::ConstPtr& map_cloud,
    const Eigen::Matrix4f& initial_guess) {
  auto ndt_aligned = typename pcl::PointCloud<PointT>::Ptr(
      new pcl::PointCloud<PointT>());

  pcl::NormalDistributionsTransform<PointT, PointT> ndt;
  ndt.setTransformationEpsilon(0.01);
  ndt.setStepSize(0.12);
  ndt.setResolution(1.0);
  ndt.setMaximumIterations(35);
  ndt.setInputSource(filtered_scan);
  ndt.setInputTarget(map_cloud);
  ndt.align(*ndt_aligned, initial_guess);

  Eigen::Matrix4f best_transform = ndt.getFinalTransformation();
  bool use_icp_refinement = !ndt.hasConverged() || ndt.getFitnessScore() > 1.5;

  if (use_icp_refinement) {
    auto icp_aligned = typename pcl::PointCloud<PointT>::Ptr(
        new pcl::PointCloud<PointT>());

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(30);
    icp.setMaxCorrespondenceDistance(2.0);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-3);
    icp.setInputSource(filtered_scan);
    icp.setInputTarget(map_cloud);
    icp.align(*icp_aligned, best_transform);

    if (icp.hasConverged()) {
      best_transform = icp.getFinalTransformation();
    }
  }

  return best_transform;
}

}  // namespace

/*
 * Paste the body of this function into the TODO section near the bottom of the
 * starter c3-main.cpp.  It assumes the starter variable names used in the
 * Udacity workspace: scanCloud, cloudFiltered, mapCloud, viewer, egoEstimate,
 * and renderPointCloud.  If your starter uses slightly different names, only
 * the variable references need to be adjusted; the algorithm is self-contained.
 */
void apply_scan_matching_localization_step() {
  // Step 1: filter scan using voxel filter.
  // A 0.5m voxel size preserves enough geometry for matching while keeping the
  // NDT/ICP solve fast enough for medium-speed driving.
  cloudFiltered = filter_scan<CarlaPoint>(scanCloud, 0.5f);

  // Step 2: estimate pose transform from lidar scan matching.
  // The previous estimated pose is used as the initial guess, so ground truth is
  // only needed for the initial egoEstimate supplied by the starter code.
  Eigen::Matrix4f init_guess = pose_to_transform(egoEstimate);
  Eigen::Matrix4f map_to_scan_transform =
      estimate_pose_transform<CarlaPoint>(cloudFiltered, mapCloud, init_guess);

  // Convert the resulting rigid-body transform back into the Pose object
  // expected by the simulator and evaluator.
  egoEstimate = getPose(map_to_scan_transform);

  // Step 3: transform the scan into the map frame and render the aligned scan.
  pcl::PointCloud<CarlaPoint>::Ptr transformedScan(
      new pcl::PointCloud<CarlaPoint>());
  pcl::transformPointCloud(*cloudFiltered, *transformedScan,
                           map_to_scan_transform);
  renderPointCloud(viewer, transformedScan, "scan", Color(1, 0, 0));
}
