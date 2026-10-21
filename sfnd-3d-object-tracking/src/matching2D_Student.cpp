
#include <numeric>
#include <stdexcept>
#include <opencv2/core/version.hpp>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    (void)kPtsSource;
    (void)kPtsRef;
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = descriptorType.compare("DES_HOG") == 0 ? cv::NORM_L2 : cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        cv::Mat source = descSource;
        cv::Mat reference = descRef;
        if (source.type() != CV_32F)
        {
            source.convertTo(source, CV_32F);
            reference.convertTo(reference, CV_32F);
        }
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        if (selectorType.compare("SEL_NN") == 0)
        {
            matcher->match(source, reference, matches);
            return;
        }

        vector<vector<cv::DMatch>> knnMatches;
        matcher->knnMatch(source, reference, knnMatches, 2);
        const double distanceRatio = 0.8;
        for (const auto &candidate : knnMatches)
        {
            if (candidate.size() == 2 && candidate[0].distance < distanceRatio * candidate[1].distance)
            {
                matches.push_back(candidate[0]);
            }
        }
        return;
    }
    else
    {
        throw invalid_argument("Unknown matcher type: " + matcherType);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        vector<vector<cv::DMatch>> knnMatches;
        matcher->knnMatch(descSource, descRef, knnMatches, 2);
        const double distanceRatio = 0.8;
        for (const auto &candidate : knnMatches)
        {
            if (candidate.size() == 2 && candidate[0].distance < distanceRatio * candidate[1].distance)
            {
                matches.push_back(candidate[0]);
            }
        }
    }
    else
    {
        throw invalid_argument("Unknown selector type: " + selectorType);
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 4)
        extractor = cv::SIFT::create();
#else
        extractor = cv::xfeatures2d::SIFT::create();
#endif
    }
    else
    {
        throw invalid_argument("Unknown descriptor type: " + descriptorType);
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints using one of the modern OpenCV detectors.
void detKeypointsModern(vector<cv::KeyPoint> &keypoints, cv::Mat &img, string detectorType, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector;
    if (detectorType.compare("FAST") == 0)
    {
        detector = cv::FastFeatureDetector::create();
    }
    else if (detectorType.compare("BRISK") == 0)
    {
        detector = cv::BRISK::create();
    }
    else if (detectorType.compare("ORB") == 0)
    {
        detector = cv::ORB::create();
    }
    else if (detectorType.compare("AKAZE") == 0)
    {
        detector = cv::AKAZE::create();
    }
    else if (detectorType.compare("SIFT") == 0)
    {
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 4)
        detector = cv::SIFT::create();
#else
        detector = cv::xfeatures2d::SIFT::create();
#endif
    }
    else
    {
        throw invalid_argument("Unknown detector type: " + detectorType);
    }

    double t = static_cast<double>(cv::getTickCount());
    detector->detect(img, keypoints);
    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
    cout << detectorType << " detection with n=" << keypoints.size() << " keypoints in " << 1000.0 * t << " ms" << endl;

    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        imshow(detectorType + " detector results", visImage);
        cv::waitKey(0);
    }
}

// Detect keypoints using the Harris detector and non-maximum suppression.
void detKeypointsHarris(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    const int blockSize = 2;
    const int apertureSize = 3;
    const int minResponse = 100;
    const double k = 0.04;
    const double maxOverlap = 0.0;

    cv::Mat response, normalized;
    response = cv::Mat::zeros(img.size(), CV_32FC1);
    double t = static_cast<double>(cv::getTickCount());
    cv::cornerHarris(img, response, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(response, normalized, 0, 255, cv::NORM_MINMAX, CV_32FC1);

    for (int row = 0; row < normalized.rows; ++row)
    {
        for (int col = 0; col < normalized.cols; ++col)
        {
            const int score = static_cast<int>(normalized.at<float>(row, col));
            if (score <= minResponse)
            {
                continue;
            }

            cv::KeyPoint candidate(cv::Point2f(col, row), 2 * apertureSize, -1, score);
            bool overlaps = false;
            for (auto &existing : keypoints)
            {
                if (cv::KeyPoint::overlap(candidate, existing) > maxOverlap)
                {
                    overlaps = true;
                    if (candidate.response > existing.response)
                    {
                        existing = candidate;
                    }
                    break;
                }
            }
            if (!overlaps)
            {
                keypoints.push_back(candidate);
            }
        }
    }

    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
    cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000.0 * t << " ms" << endl;
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        imshow("Harris detector results", visImage);
        cv::waitKey(0);
    }
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}
