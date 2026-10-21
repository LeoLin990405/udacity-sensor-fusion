/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

void cityBlock(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    ProcessPointClouds<pcl::PointXYZI>& pointProcessor,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    const pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud =
        pointProcessor.FilterCloud(
            inputCloud,
            0.2f,
            Eigen::Vector4f(-10.0f, -6.0f, -2.0f, 1.0f),
            Eigen::Vector4f(30.0f, 7.0f, 2.0f, 1.0f));

    const std::pair<
        pcl::PointCloud<pcl::PointXYZI>::Ptr,
        pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud =
        pointProcessor.SegmentPlane(filteredCloud, 100, 0.2f);

    renderPointCloud(viewer, segmentedCloud.second, "road", Color(0, 1, 0));

    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters =
        pointProcessor.Clustering(segmentedCloud.first, 0.5f, 10, 500);
    const std::vector<Color> colors = {
        Color(1, 0, 0), Color(0, 0, 1), Color(1, 1, 0)};

    int clusterId = 0;
    for (const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster : clusters)
    {
        const Color& color = colors[clusterId % colors.size()];
        renderPointCloud(
            viewer, cluster, "obstacle" + std::to_string(clusterId), color);
        renderBox(
            viewer, pointProcessor.BoundingBox(cluster), clusterId, color, 0.8f);
        ++clusterId;
    }
}


// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main(int argc, char** argv)
{
    std::cout << "starting environment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    const std::string dataPath = argc > 1
        ? argv[1]
        : "../src/sensors/data/pcd/data_1";
    const std::vector<boost::filesystem::path> stream =
        pointProcessor.streamPcd(dataPath);
    if (stream.empty())
    {
        std::cerr << "No PCD files found in " << dataPath << std::endl;
        return 1;
    }

    std::vector<boost::filesystem::path>::const_iterator streamIterator =
        stream.begin();

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        const pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud =
            pointProcessor.loadPcd(streamIterator->string());
        cityBlock(viewer, pointProcessor, inputCloud);

        ++streamIterator;
        if (streamIterator == stream.end())
        {
            streamIterator = stream.begin();
        }
        viewer->spinOnce ();
    } 
}
