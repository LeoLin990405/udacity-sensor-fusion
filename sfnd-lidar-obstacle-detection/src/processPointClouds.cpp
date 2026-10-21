// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);
    voxelGrid.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> roofIndices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1.0, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(roofIndices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (int index : roofIndices) {
        inliers->indices.push_back(index);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    std::unordered_set<int> inlierSet(inliers->indices.begin(), inliers->indices.end());
    for (std::size_t index = 0; index < cloud->points.size(); ++index) {
        const PointT& point = cloud->points[index];
        if (inlierSet.count(static_cast<int>(index))) {
            planeCloud->points.push_back(point);
        } else {
            obstCloud->points.push_back(point);
        }
    }

    planeCloud->width = planeCloud->points.size();
    planeCloud->height = 1;
    planeCloud->is_dense = cloud->is_dense;
    obstCloud->width = obstCloud->points.size();
    obstCloud->height = 1;
    obstCloud->is_dense = cloud->is_dense;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    std::unordered_set<int> bestInliers;

    if (cloud->points.size() < 3) {
        return SeparateClouds(inliers, cloud);
    }

    static std::mt19937 generator(42);
    std::uniform_int_distribution<int> distribution(
        0, static_cast<int>(cloud->points.size()) - 1);
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        std::unordered_set<int> sample;
        while (sample.size() < 3) {
            sample.insert(distribution(generator));
        }

        auto iterator = sample.begin();
        const PointT p1 = cloud->points[*iterator++];
        const PointT p2 = cloud->points[*iterator++];
        const PointT p3 = cloud->points[*iterator];

        const float a = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
        const float b = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
        const float c = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
        const float d = -(a * p1.x + b * p1.y + c * p1.z);
        const float norm = std::sqrt(a * a + b * b + c * c);
        if (norm == 0.0f) {
            continue;
        }

        std::unordered_set<int> currentInliers;
        for (std::size_t index = 0; index < cloud->points.size(); ++index) {
            const PointT& point = cloud->points[index];
            const float distance = std::fabs(a * point.x + b * point.y + c * point.z + d) / norm;
            if (distance <= distanceThreshold) {
                currentInliers.insert(static_cast<int>(index));
            }
        }
        if (currentInliers.size() > bestInliers.size()) {
            bestInliers = currentInliers;
        }
    }
    for (int index : bestInliers) {
        inliers->indices.push_back(index);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree3D tree;
    std::vector<KdTree3D::Point> points;
    points.reserve(cloud->points.size());
    for (std::size_t index = 0; index < cloud->points.size(); ++index) {
        const PointT& point = cloud->points[index];
        points.push_back({{point.x, point.y, point.z}});
        tree.insert(points.back(), static_cast<int>(index));
    }

    const std::vector<std::vector<int>> clusterIndexSets =
        euclideanCluster3D(points, tree, clusterTolerance);
    for (const std::vector<int>& clusterIndices : clusterIndexSets) {
        if (clusterIndices.size() < static_cast<std::size_t>(minSize) ||
            clusterIndices.size() > static_cast<std::size_t>(maxSize)) {
            continue;
        }

        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
        for (int index : clusterIndices) {
            cluster->points.push_back(cloud->points[index]);
        }
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = cloud->is_dense;
        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    std::sort(paths.begin(), paths.end());

    return paths;

}
