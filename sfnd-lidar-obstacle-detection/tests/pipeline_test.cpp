#include <cmath>
#include <iostream>
#include <string>

#include "processPointClouds.h"
#include "processPointClouds.cpp"

namespace
{
bool require(bool condition, const std::string& message)
{
    if (!condition)
    {
        std::cerr << "Pipeline test failed: " << message << std::endl;
    }
    return condition;
}
}

int main()
{
    ProcessPointClouds<pcl::PointXYZI> processor;
    const std::string pcdFile =
        std::string(PROJECT_SOURCE_DIR) +
        "/src/sensors/data/pcd/data_1/0000000000.pcd";
    const pcl::PointCloud<pcl::PointXYZI>::Ptr input =
        processor.loadPcd(pcdFile);

    if (!require(!input->empty(), "the sample PCD frame could not be loaded"))
    {
        return 1;
    }

    const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered =
        processor.FilterCloud(
            input,
            0.2f,
            Eigen::Vector4f(-10.0f, -6.0f, -2.0f, 1.0f),
            Eigen::Vector4f(30.0f, 7.0f, 2.0f, 1.0f));
    const std::pair<
        pcl::PointCloud<pcl::PointXYZI>::Ptr,
        pcl::PointCloud<pcl::PointXYZI>::Ptr> segmented =
        processor.SegmentPlane(filtered, 100, 0.2f);
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters =
        processor.Clustering(segmented.first, 0.5f, 10, 500);

    bool valid = true;
    valid &= require(filtered->size() < input->size(),
                     "filtering did not reduce the point cloud");
    valid &= require(!segmented.first->empty(),
                     "RANSAC did not retain any obstacle points");
    valid &= require(!segmented.second->empty(),
                     "RANSAC did not identify the road plane");
    valid &= require(clusters.size() >= 3,
                     "fewer than three obstacle clusters were detected");

    for (const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster : clusters)
    {
        valid &= require(cluster->size() >= 10 && cluster->size() <= 500,
                         "a cluster is outside the configured size bounds");
        const Box box = processor.BoundingBox(cluster);
        valid &= require(
            std::isfinite(box.x_min) && std::isfinite(box.x_max) &&
            std::isfinite(box.y_min) && std::isfinite(box.y_max) &&
            std::isfinite(box.z_min) && std::isfinite(box.z_max) &&
            box.x_min <= box.x_max && box.y_min <= box.y_max &&
            box.z_min <= box.z_max,
            "a cluster produced an invalid bounding box");
    }

    if (!valid)
    {
        return 1;
    }

    std::cout << "Pipeline test passed with " << input->size()
              << " input points, " << filtered->size()
              << " filtered points, and " << clusters.size()
              << " obstacle clusters" << std::endl;
    return 0;
}
