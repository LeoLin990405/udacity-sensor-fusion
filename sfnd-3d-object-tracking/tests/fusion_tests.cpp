#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>
#include <vector>

#include "camFusion.hpp"
#include "dataStructures.h"

namespace
{
void require(bool condition, const std::string &message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void requireNear(double actual, double expected, double tolerance, const std::string &message)
{
    require(std::isfinite(actual) && std::fabs(actual - expected) <= tolerance, message);
}

cv::DMatch match(int query, int train)
{
    return cv::DMatch(query, train, 0.0f);
}

BoundingBox boundingBox(int id, const cv::Rect &roi)
{
    BoundingBox box{};
    box.boxID = id;
    box.roi = roi;
    return box;
}

void testBoundingBoxMatching()
{
    DataFrame previous{};
    DataFrame current{};
    previous.boundingBoxes = {boundingBox(10, cv::Rect(0, 0, 100, 100)), boundingBox(11, cv::Rect(100, 0, 100, 100))};
    current.boundingBoxes = {boundingBox(20, cv::Rect(0, 0, 100, 100)), boundingBox(21, cv::Rect(100, 0, 100, 100))};
    previous.keypoints = {cv::KeyPoint(10.0f, 10.0f, 1.0f), cv::KeyPoint(20.0f, 20.0f, 1.0f), cv::KeyPoint(120.0f, 10.0f, 1.0f)};
    current.keypoints = {cv::KeyPoint(15.0f, 10.0f, 1.0f), cv::KeyPoint(25.0f, 20.0f, 1.0f), cv::KeyPoint(125.0f, 10.0f, 1.0f)};
    std::vector<cv::DMatch> matches = {match(0, 0), match(1, 1), match(2, 2)};
    std::map<int, int> best;

    matchBoundingBoxes(matches, best, previous, current);
    require(best.size() == 2, "Expected two bounding-box matches");
    require(best.at(10) == 20 && best.at(11) == 21, "Bounding-box voting selected the wrong boxes");
}

void testKeypointOutlierFiltering()
{
    BoundingBox box{};
    box.roi = cv::Rect(0, 0, 200, 200);
    std::vector<cv::KeyPoint> previous = {
        cv::KeyPoint(10.0f, 10.0f, 1.0f), cv::KeyPoint(30.0f, 10.0f, 1.0f),
        cv::KeyPoint(50.0f, 10.0f, 1.0f), cv::KeyPoint(70.0f, 10.0f, 1.0f)};
    std::vector<cv::KeyPoint> current = {
        cv::KeyPoint(15.0f, 10.0f, 1.0f), cv::KeyPoint(36.0f, 10.0f, 1.0f),
        cv::KeyPoint(55.5f, 10.0f, 1.0f), cv::KeyPoint(170.0f, 10.0f, 1.0f)};
    std::vector<cv::DMatch> matches = {match(0, 0), match(1, 1), match(2, 2), match(3, 3)};

    clusterKptMatchesWithROI(box, previous, current, matches);
    require(box.kptMatches.size() == 3, "MAD filter did not remove the displacement outlier");
    require(box.keypoints.size() == box.kptMatches.size(), "Bounding-box keypoints and matches are inconsistent");
}

void testLidarTtc()
{
    std::vector<LidarPoint> previous = {{10.0, 0.0, 0.0, 0.0}, {10.0, 0.0, 0.0, 0.0}, {10.0, 0.0, 0.0, 0.0}, {100.0, 0.0, 0.0, 0.0}, {-5.0, 0.0, 0.0, 0.0}};
    std::vector<LidarPoint> current = {{9.0, 0.0, 0.0, 0.0}, {9.0, 0.0, 0.0, 0.0}, {9.0, 0.0, 0.0, 0.0}, {0.1, 0.0, 0.0, 0.0}, {-3.0, 0.0, 0.0, 0.0}};
    double ttc = 0.0;
    computeTTCLidar(previous, current, 10.0, ttc);
    requireNear(ttc, 0.9, 1e-9, "Median Lidar TTC is incorrect");
}

void testCameraTtc()
{
    std::vector<cv::KeyPoint> previous = {cv::KeyPoint(0.0f, 0.0f, 1.0f), cv::KeyPoint(100.0f, 0.0f, 1.0f)};
    std::vector<cv::KeyPoint> current = {cv::KeyPoint(0.0f, 0.0f, 1.0f), cv::KeyPoint(110.0f, 0.0f, 1.0f)};
    std::vector<cv::DMatch> matches = {match(0, 0), match(1, 1)};
    double ttc = 0.0;
    computeTTCCamera(previous, current, matches, 10.0, ttc);
    requireNear(ttc, 1.0, 1e-9, "Median camera TTC is incorrect");
}
} // namespace

int main()
{
    try
    {
        testBoundingBoxMatching();
        testKeypointOutlierFiltering();
        testLidarTtc();
        testCameraTtc();
        std::cout << "All fusion tests passed" << std::endl;
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Test failure: " << error.what() << std::endl;
        return 1;
    }
}
