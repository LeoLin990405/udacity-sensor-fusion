#include "scan_matcher.h"

#include <pcl/io/pcd_io.h>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

namespace {

Eigen::Matrix4f MakeTransform(float x, float y, float yaw) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).matrix();
  transform(0, 3) = x;
  transform(1, 3) = y;
  return transform;
}

ScanCloud::Ptr MakeSyntheticScan(const ScanCloud::ConstPtr& map_cloud,
                                 const Eigen::Matrix4f& pose) {
  ScanCloud::Ptr scan(new ScanCloud);
  const Eigen::Matrix4f inverse_pose = pose.inverse();

  for (std::size_t index = 0; index < map_cloud->points.size(); index += 2) {
    const auto& map_point = map_cloud->points[index];
    Eigen::Vector4f point(map_point.x, map_point.y, map_point.z, 1.0f);
    const Eigen::Vector4f local = inverse_pose * point;
    const float range_squared = local.head<3>().squaredNorm();
    if (range_squared > 8.0f && range_squared < 900.0f) {
      const float noise = 0.01f * std::sin(static_cast<float>(index));
      scan->points.emplace_back(local.x() + noise, local.y() - noise,
                                local.z());
    }
  }

  scan->width = static_cast<std::uint32_t>(scan->points.size());
  scan->height = 1;
  return scan;
}

bool CheckPose(const Eigen::Matrix4f& estimated,
               const Eigen::Matrix4f& expected,
               float maximum_position_error,
               float maximum_yaw_error) {
  const Eigen::Matrix4f error = expected.inverse() * estimated;
  const float position_error = error.block<3, 1>(0, 3).norm();
  const float yaw_error = std::abs(std::atan2(error(1, 0), error(0, 0)));
  std::cout << "position_error=" << position_error
            << " yaw_error=" << yaw_error << std::endl;
  return position_error <= maximum_position_error &&
         yaw_error <= maximum_yaw_error;
}

}  // namespace

int main(int argc, char** argv) {
  const char* map_path = argc > 1 ? argv[1] : "map.pcd";
  ScanCloud::Ptr map_cloud(new ScanCloud);
  if (pcl::io::loadPCDFile(map_path, *map_cloud) == -1) {
    std::cerr << "Unable to load map: " << map_path << std::endl;
    return EXIT_FAILURE;
  }

  ScanMatcher matcher(map_cloud);
  Eigen::Matrix4f estimate = Eigen::Matrix4f::Identity();

  for (int frame = 0; frame <= 10; ++frame) {
    const Eigen::Matrix4f truth = MakeTransform(0.8f * frame, 0.0f, 0.0f);
    const MatchResult result = matcher.Match(
        MakeSyntheticScan(map_cloud, truth), estimate);
    if (!result.accepted || !CheckPose(result.transform, truth, 0.35f, 0.05f)) {
      std::cerr << "Continuous-motion match failed at frame " << frame
                << std::endl;
      return EXIT_FAILURE;
    }
    estimate = result.transform;
  }

  const Eigen::Matrix4f far_truth = MakeTransform(170.0f, 0.0f, 0.04f);
  const Eigen::Matrix4f far_initial = MakeTransform(169.4f, 0.2f, 0.02f);
  const MatchResult far_result = matcher.Match(
      MakeSyntheticScan(map_cloud, far_truth), far_initial);
  if (!far_result.accepted ||
      !CheckPose(far_result.transform, far_truth, 0.35f, 0.05f)) {
    std::cerr << "170-meter-region match failed" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "All scan-matching checks passed" << std::endl;
  return EXIT_SUCCESS;
}

