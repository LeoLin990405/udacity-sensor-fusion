#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/Vehicle.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/sensor/data/LidarMeasurement.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

#include "helper.h"
#include "scan_matcher.h"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;

namespace {

cc::Vehicle::Control control;
std::vector<ControlState> control_events;
bool refresh_view = false;

void KeyboardEventOccurred(
    const pcl::visualization::KeyboardEvent& event, void*) {
  if (!event.keyDown()) {
    return;
  }

  if (event.getKeySym() == "Right") {
    control_events.emplace_back(0.0f, -0.02f, 0.0f);
  } else if (event.getKeySym() == "Left") {
    control_events.emplace_back(0.0f, 0.02f, 0.0f);
  } else if (event.getKeySym() == "Up") {
    control_events.emplace_back(0.1f, 0.0f, 0.0f);
  } else if (event.getKeySym() == "Down") {
    control_events.emplace_back(-0.1f, 0.0f, 0.0f);
  } else if (event.getKeySym() == "a") {
    refresh_view = true;
  }
}

void Actuate(ControlState response, cc::Vehicle::Control& state) {
  if (response.t > 0.0f) {
    if (state.reverse) {
      state.reverse = false;
      state.throttle = std::min(response.t, 1.0f);
    } else {
      state.throttle = std::min(state.throttle + response.t, 1.0f);
    }
  } else if (response.t < 0.0f) {
    response.t = -response.t;
    if (state.reverse) {
      state.throttle = std::min(state.throttle + response.t, 1.0f);
    } else {
      state.reverse = true;
      state.throttle = std::min(response.t, 1.0f);
    }
  }

  state.steer = std::min(std::max(state.steer + response.s, -1.0f), 1.0f);
  state.brake = response.b;
}

void DrawCar(Pose pose, int id, Color color, double alpha,
             pcl::visualization::PCLVisualizer::Ptr& viewer) {
  BoxQ box;
  box.bboxTransform = Eigen::Vector3f(
      pose.position.x, pose.position.y, pose.position.z);
  box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
  box.cube_length = 4.0f;
  box.cube_width = 2.0f;
  box.cube_height = 2.0f;
  renderBox(viewer, box, id, color, alpha);
}

Eigen::Matrix4f PoseTransform(const Pose& pose) {
  return transform3D(
             pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll,
             pose.position.x, pose.position.y, pose.position.z)
      .cast<float>();
}

}  // namespace

int main() {
  auto client = cc::Client("localhost", 2000);
  client.SetTimeout(2s);
  auto world = client.GetWorld();

  auto blueprint_library = world.GetBlueprintLibrary();
  auto vehicles = blueprint_library->Filter("vehicle");
  auto map = world.GetMap();
  auto spawn_transform = map->GetRecommendedSpawnPoints()[1];
  auto ego_actor = world.SpawnActor((*vehicles)[12], spawn_transform);
  auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);

  auto lidar_blueprint = *(blueprint_library->Find("sensor.lidar.ray_cast"));
  lidar_blueprint.SetAttribute("upper_fov", "15");
  lidar_blueprint.SetAttribute("lower_fov", "-25");
  lidar_blueprint.SetAttribute("channels", "32");
  lidar_blueprint.SetAttribute("range", "30");
  lidar_blueprint.SetAttribute("rotation_frequency", "60");
  lidar_blueprint.SetAttribute("points_per_second", "500000");

  const auto lidar_transform =
      cg::Transform(cg::Location(-0.5, 0.0, 1.8));
  auto lidar_actor =
      world.SpawnActor(lidar_blueprint, lidar_transform, ego_actor.get());
  auto lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0.0, 0.0, 0.0);
  viewer->registerKeyboardCallback(KeyboardEventOccurred, nullptr);
  viewer->setCameraPosition(0.0, 0.0, 60.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0);

  PointCloudT::Ptr map_cloud(new PointCloudT);
  if (pcl::io::loadPCDFile("map.pcd", *map_cloud) == -1) {
    PCL_ERROR("Could not read map.pcd\n");
    return 1;
  }
  std::cout << "Loaded " << map_cloud->points.size()
            << " map points" << std::endl;
  renderPointCloud(viewer, map_cloud, "map", Color(0.0f, 0.0f, 1.0f), 1);

  ScanMatcher scan_matcher(map_cloud);
  PointCloudT pending_cloud;
  PointCloudT::Ptr scan_cloud(new PointCloudT);
  std::atomic<bool> collecting_scan(true);

  lidar->Listen([&collecting_scan, &pending_cloud, &scan_cloud](auto data) {
    if (!collecting_scan.load(std::memory_order_acquire)) {
      return;
    }

    auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
    for (const auto& detection : *scan) {
      const double range_squared = detection.x * detection.x +
                                   detection.y * detection.y +
                                   detection.z * detection.z;
      if (range_squared > 8.0) {
        // Convert CARLA's sensor frame to the PCL/course map frame used by the
        // current Udacity virtual environment.
        pending_cloud.points.push_back(
            PointT(-detection.y, detection.x, -detection.z));
      }
    }

    if (pending_cloud.points.size() > 5000) {
      *scan_cloud = pending_cloud;
      scan_cloud->width = static_cast<std::uint32_t>(scan_cloud->points.size());
      scan_cloud->height = 1;
      collecting_scan.store(false, std::memory_order_release);
    }
  });

  Pose pose(Point(0.0, 0.0, 0.0), Rotate(0.0, 0.0, 0.0));
  const auto initial_transform = vehicle->GetTransform();
  const Pose pose_reference(
      Point(initial_transform.location.x, initial_transform.location.y,
            initial_transform.location.z),
      Rotate(initial_transform.rotation.yaw * pi / 180.0,
             initial_transform.rotation.pitch * pi / 180.0,
             initial_transform.rotation.roll * pi / 180.0));
  double max_error = 0.0;

  while (!viewer->wasStopped()) {
    while (collecting_scan.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(100ms);
      world.Tick(1s);
    }

    if (refresh_view) {
      viewer->setCameraPosition(
          pose.position.x, pose.position.y, 60.0,
          pose.position.x + 1.0, pose.position.y + 1.0, 0.0,
          0.0, 0.0, 1.0);
      refresh_view = false;
    }

    const auto ground_truth_transform = vehicle->GetTransform();
    Pose true_pose(
        Point(ground_truth_transform.location.x,
              ground_truth_transform.location.y,
              ground_truth_transform.location.z),
        Rotate(ground_truth_transform.rotation.yaw * pi / 180.0,
               ground_truth_transform.rotation.pitch * pi / 180.0,
               ground_truth_transform.rotation.roll * pi / 180.0));
    true_pose = true_pose - pose_reference;

    viewer->removeShape("box0");
    viewer->removeShape("boxFill0");
    DrawCar(true_pose, 0, Color(1.0f, 0.0f, 0.0f), 0.7, viewer);

    const double heading = true_pose.rotation.yaw;
    const double steering_heading = control.steer * pi / 4.0 + heading;
    viewer->removeShape("steer");
    renderRay(
        viewer,
        Point(true_pose.position.x + 2.0 * std::cos(heading),
              true_pose.position.y + 2.0 * std::sin(heading),
              true_pose.position.z),
        Point(true_pose.position.x + 4.0 * std::cos(steering_heading),
              true_pose.position.y + 4.0 * std::sin(steering_heading),
              true_pose.position.z),
        "steer", Color(0.0f, 1.0f, 0.0f));

    if (!control_events.empty()) {
      Actuate(control_events.back(), control);
      control_events.clear();
      vehicle->ApplyControl(control);
    }

    viewer->spinOnce();

    const MatchResult match = scan_matcher.Match(scan_cloud, PoseTransform(pose));
    if (match.accepted) {
      pose = getPose(match.transform.cast<double>());
    } else {
      std::cerr << "Scan match rejected; holding previous pose. NDT converged: "
                << std::boolalpha << match.converged
                << ", fitness: " << match.fitness_score << std::endl;
    }

    viewer->removePointCloud("scan");
    renderPointCloud(
        viewer, match.aligned_cloud, "scan", Color(1.0f, 0.0f, 0.0f), 2);
    viewer->removeShape("box1");
    viewer->removeShape("boxFill1");
    DrawCar(pose, 1, Color(0.0f, 1.0f, 0.0f), 0.35, viewer);

    const double x_error = true_pose.position.x - pose.position.x;
    const double y_error = true_pose.position.y - pose.position.y;
    const double pose_error = std::sqrt(x_error * x_error + y_error * y_error);
    max_error = std::max(max_error, pose_error);
    const double distance_driven = std::sqrt(
        true_pose.position.x * true_pose.position.x +
        true_pose.position.y * true_pose.position.y);

    viewer->removeShape("maxE");
    viewer->addText("Max Error: " + std::to_string(max_error) + " m",
                    200, 100, 32, 1.0, 1.0, 1.0, "maxE", 0);
    viewer->removeShape("derror");
    viewer->addText("Pose error: " + std::to_string(pose_error) + " m",
                    200, 150, 32, 1.0, 1.0, 1.0, "derror", 0);
    viewer->removeShape("dist");
    viewer->addText("Distance: " + std::to_string(distance_driven) + " m",
                    200, 200, 32, 1.0, 1.0, 1.0, "dist", 0);

    if (max_error > 1.2 || distance_driven >= 170.0) {
      viewer->removeShape("eval");
      if (max_error > 1.2) {
        viewer->addText("Try Again", 200, 50, 32,
                        1.0, 0.0, 0.0, "eval", 0);
      } else {
        viewer->addText("Passed!", 200, 50, 32,
                        0.0, 1.0, 0.0, "eval", 0);
      }
    }

    pending_cloud.points.clear();
    collecting_scan.store(true, std::memory_order_release);
  }

  lidar->Stop();
  lidar_actor->Destroy();
  ego_actor->Destroy();
  return 0;
}
