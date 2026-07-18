#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"
#include "lidarData.hpp"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"

using namespace std;

namespace
{
struct Options
{
    string detector = "SHITOMASI";
    string descriptor = "BRISK";
    string matcher = "MAT_BF";
    string selector = "SEL_KNN";
    string dataPath = "../";
    string outputPath;
    int startFrame = 0;
    int endFrame = 18;
    int stepWidth = 1;
    bool visualize = false;
};

string uppercase(string value)
{
    transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(toupper(character));
    });
    return value;
}

string withTrailingSlash(string path)
{
    if (!path.empty() && path.back() != '/')
    {
        path.push_back('/');
    }
    return path;
}

void printUsage(const char *program)
{
    cout << "Usage: " << program << " [DETECTOR] [DESCRIPTOR] [options]\n"
         << "Detectors: SHITOMASI HARRIS FAST BRISK ORB AKAZE SIFT\n"
         << "Descriptors: BRISK BRIEF ORB FREAK AKAZE SIFT\n"
         << "Options:\n"
         << "  --matcher MAT_BF|MAT_FLANN\n"
         << "  --selector SEL_NN|SEL_KNN\n"
         << "  --data PATH\n"
         << "  --output CSV_PATH\n"
         << "  --start FRAME --end FRAME --step WIDTH\n"
         << "  --visualize\n";
}

Options parseOptions(int argc, const char *argv[])
{
    Options options;
    int positional = 0;
    for (int index = 1; index < argc; ++index)
    {
        const string argument = argv[index];
        auto requireValue = [&](const string &name) -> string {
            if (++index >= argc)
            {
                throw invalid_argument("Missing value for " + name);
            }
            return argv[index];
        };

        if (argument == "--help" || argument == "-h")
        {
            printUsage(argv[0]);
            exit(0);
        }
        else if (argument == "--matcher")
        {
            options.matcher = uppercase(requireValue(argument));
        }
        else if (argument == "--selector")
        {
            options.selector = uppercase(requireValue(argument));
        }
        else if (argument == "--data")
        {
            options.dataPath = withTrailingSlash(requireValue(argument));
        }
        else if (argument == "--output")
        {
            options.outputPath = requireValue(argument);
        }
        else if (argument == "--start")
        {
            options.startFrame = stoi(requireValue(argument));
        }
        else if (argument == "--end")
        {
            options.endFrame = stoi(requireValue(argument));
        }
        else if (argument == "--step")
        {
            options.stepWidth = stoi(requireValue(argument));
        }
        else if (argument == "--visualize")
        {
            options.visualize = true;
        }
        else if (!argument.empty() && argument.front() == '-')
        {
            throw invalid_argument("Unknown option: " + argument);
        }
        else if (positional == 0)
        {
            options.detector = uppercase(argument);
            ++positional;
        }
        else if (positional == 1)
        {
            options.descriptor = uppercase(argument);
            ++positional;
        }
        else
        {
            throw invalid_argument("Unexpected positional argument: " + argument);
        }
    }
    return options;
}

bool contains(const vector<string> &values, const string &value)
{
    return find(values.begin(), values.end(), value) != values.end();
}

void validateOptions(const Options &options)
{
    const vector<string> detectors = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    const vector<string> descriptors = {"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};
    if (!contains(detectors, options.detector))
    {
        throw invalid_argument("Unknown detector: " + options.detector);
    }
    if (!contains(descriptors, options.descriptor))
    {
        throw invalid_argument("Unknown descriptor: " + options.descriptor);
    }
    if (options.descriptor == "AKAZE" && options.detector != "AKAZE")
    {
        throw invalid_argument("AKAZE descriptors require AKAZE keypoints");
    }
    if (options.detector == "SIFT" && options.descriptor == "ORB")
    {
        throw invalid_argument("SIFT keypoints are incompatible with the ORB descriptor");
    }
    if (options.startFrame < 0 || options.endFrame < options.startFrame || options.stepWidth <= 0)
    {
        throw invalid_argument("Invalid frame range");
    }
}

BoundingBox *findBoundingBox(vector<BoundingBox> &boxes, int boxId)
{
    for (auto &box : boxes)
    {
        if (box.boxID == boxId)
        {
            return &box;
        }
    }
    return nullptr;
}

double medianLidarDistance(const vector<LidarPoint> &points)
{
    vector<double> distances;
    for (const auto &point : points)
    {
        if (isfinite(point.x) && point.x > 0.0)
        {
            distances.push_back(point.x);
        }
    }
    if (distances.empty())
    {
        return numeric_limits<double>::quiet_NaN();
    }
    const size_t middle = distances.size() / 2;
    nth_element(distances.begin(), distances.begin() + middle, distances.end());
    const double upper = distances[middle];
    if (distances.size() % 2 != 0)
    {
        return upper;
    }
    nth_element(distances.begin(), distances.begin() + middle - 1, distances.begin() + middle);
    return 0.5 * (distances[middle - 1] + upper);
}

void detectKeypoints(const string &detector, vector<cv::KeyPoint> &keypoints, cv::Mat &image)
{
    if (detector == "SHITOMASI")
    {
        detKeypointsShiTomasi(keypoints, image, false);
    }
    else if (detector == "HARRIS")
    {
        detKeypointsHarris(keypoints, image, false);
    }
    else
    {
        detKeypointsModern(keypoints, image, detector, false);
    }
}
} // namespace

int main(int argc, const char *argv[])
{
    try
    {
        const Options options = parseOptions(argc, argv);
        validateOptions(options);
        cout << "Running " << options.detector << "+" << options.descriptor
             << " on frames " << options.startFrame << "-" << options.endFrame << endl;

        const string dataPath = withTrailingSlash(options.dataPath);
        const string imageBasePath = dataPath + "images/";
        const string imagePrefix = "KITTI/2011_09_26/image_02/data/000000";
        const string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
        const string yoloBasePath = dataPath + "dat/yolo/";
        const string yoloClassesFile = yoloBasePath + "coco.names";
        const string yoloConfiguration = yoloBasePath + "yolov3.cfg";
        const string yoloWeights = yoloBasePath + "yolov3.weights";

        ofstream csv;
        if (!options.outputPath.empty())
        {
            csv.open(options.outputPath);
            if (!csv)
            {
                throw runtime_error("Unable to open output CSV: " + options.outputPath);
            }
            csv << "detector,descriptor,previous_frame,current_frame,previous_box,current_box,lead_object,"
                   "keypoint_matches,lidar_points_previous,lidar_points_current,lidar_distance_previous,"
                   "lidar_distance_current,ttc_lidar,ttc_camera\n";
            csv << setprecision(10);
        }

        cv::Mat PRect(3, 4, cv::DataType<double>::type);
        cv::Mat RRect(4, 4, cv::DataType<double>::type);
        cv::Mat RT(4, 4, cv::DataType<double>::type);

        RT.at<double>(0, 0) = 7.533745e-03; RT.at<double>(0, 1) = -9.999714e-01; RT.at<double>(0, 2) = -6.166020e-04; RT.at<double>(0, 3) = -4.069766e-03;
        RT.at<double>(1, 0) = 1.480249e-02; RT.at<double>(1, 1) = 7.280733e-04; RT.at<double>(1, 2) = -9.998902e-01; RT.at<double>(1, 3) = -7.631618e-02;
        RT.at<double>(2, 0) = 9.998621e-01; RT.at<double>(2, 1) = 7.523790e-03; RT.at<double>(2, 2) = 1.480755e-02; RT.at<double>(2, 3) = -2.717806e-01;
        RT.at<double>(3, 0) = 0.0; RT.at<double>(3, 1) = 0.0; RT.at<double>(3, 2) = 0.0; RT.at<double>(3, 3) = 1.0;

        RRect.at<double>(0, 0) = 9.999239e-01; RRect.at<double>(0, 1) = 9.837760e-03; RRect.at<double>(0, 2) = -7.445048e-03; RRect.at<double>(0, 3) = 0.0;
        RRect.at<double>(1, 0) = -9.869795e-03; RRect.at<double>(1, 1) = 9.999421e-01; RRect.at<double>(1, 2) = -4.278459e-03; RRect.at<double>(1, 3) = 0.0;
        RRect.at<double>(2, 0) = 7.402527e-03; RRect.at<double>(2, 1) = 4.351614e-03; RRect.at<double>(2, 2) = 9.999631e-01; RRect.at<double>(2, 3) = 0.0;
        RRect.at<double>(3, 0) = 0.0; RRect.at<double>(3, 1) = 0.0; RRect.at<double>(3, 2) = 0.0; RRect.at<double>(3, 3) = 1.0;

        PRect.at<double>(0, 0) = 7.215377e+02; PRect.at<double>(0, 1) = 0.0; PRect.at<double>(0, 2) = 6.095593e+02; PRect.at<double>(0, 3) = 0.0;
        PRect.at<double>(1, 0) = 0.0; PRect.at<double>(1, 1) = 7.215377e+02; PRect.at<double>(1, 2) = 1.728540e+02; PRect.at<double>(1, 3) = 0.0;
        PRect.at<double>(2, 0) = 0.0; PRect.at<double>(2, 1) = 0.0; PRect.at<double>(2, 2) = 1.0; PRect.at<double>(2, 3) = 0.0;

        const double sensorFrameRate = 10.0 / options.stepWidth;
        vector<DataFrame> dataBuffer;
        const size_t dataBufferSize = 2;
        size_t resultRows = 0;

        for (int frameIndex = options.startFrame; frameIndex <= options.endFrame; frameIndex += options.stepWidth)
        {
            ostringstream frameNumber;
            frameNumber << setfill('0') << setw(4) << frameIndex;
            const string imageFile = imageBasePath + imagePrefix + frameNumber.str() + ".png";
            cv::Mat image = cv::imread(imageFile);
            if (image.empty())
            {
                throw runtime_error("Unable to load image: " + imageFile);
            }

            DataFrame frame{};
            frame.cameraImg = image;
            dataBuffer.push_back(frame);
            if (dataBuffer.size() > dataBufferSize)
            {
                dataBuffer.erase(dataBuffer.begin());
            }
            DataFrame &currentFrame = dataBuffer.back();

            detectObjects(currentFrame.cameraImg, currentFrame.boundingBoxes, 0.2f, 0.4f,
                          yoloBasePath, yoloClassesFile, yoloConfiguration, yoloWeights, false);

            const string lidarFile = imageBasePath + lidarPrefix + frameNumber.str() + ".bin";
            vector<LidarPoint> lidarPoints;
            loadLidarFromFile(lidarPoints, lidarFile);
            cropLidarPoints(lidarPoints, 2.0f, 20.0f, 2.0f, -1.5f, -0.9f, 0.1f);
            currentFrame.lidarPoints = lidarPoints;
            clusterLidarWithROI(currentFrame.boundingBoxes, currentFrame.lidarPoints, 0.10f, PRect, RRect, RT);

            cv::Mat gray;
            cv::cvtColor(currentFrame.cameraImg, gray, cv::COLOR_BGR2GRAY);
            detectKeypoints(options.detector, currentFrame.keypoints, gray);
            descKeypoints(currentFrame.keypoints, currentFrame.cameraImg, currentFrame.descriptors, options.descriptor);

            if (dataBuffer.size() < 2)
            {
                continue;
            }

            DataFrame &previousFrame = dataBuffer.front();
            vector<cv::DMatch> matches;
            const string descriptorCategory = options.descriptor == "SIFT" ? "DES_HOG" : "DES_BINARY";
            matchDescriptors(previousFrame.keypoints, currentFrame.keypoints,
                             previousFrame.descriptors, currentFrame.descriptors,
                             matches, descriptorCategory, options.matcher, options.selector);
            currentFrame.kptMatches = matches;
            matchBoundingBoxes(matches, currentFrame.bbMatches, previousFrame, currentFrame);

            int leadPreviousBox = -1;
            int leadCurrentBox = -1;
            double closestDistance = numeric_limits<double>::infinity();
            for (const auto &pair : currentFrame.bbMatches)
            {
                BoundingBox *previousBox = findBoundingBox(previousFrame.boundingBoxes, pair.first);
                BoundingBox *currentBox = findBoundingBox(currentFrame.boundingBoxes, pair.second);
                if (previousBox == nullptr || currentBox == nullptr || previousBox->lidarPoints.empty() || currentBox->lidarPoints.empty())
                {
                    continue;
                }
                const double distance = medianLidarDistance(currentBox->lidarPoints);
                if (isfinite(distance) && distance < closestDistance)
                {
                    closestDistance = distance;
                    leadPreviousBox = previousBox->boxID;
                    leadCurrentBox = currentBox->boxID;
                }
            }

            for (const auto &pair : currentFrame.bbMatches)
            {
                BoundingBox *previousBox = findBoundingBox(previousFrame.boundingBoxes, pair.first);
                BoundingBox *currentBox = findBoundingBox(currentFrame.boundingBoxes, pair.second);
                if (previousBox == nullptr || currentBox == nullptr)
                {
                    continue;
                }

                double ttcLidar;
                double ttcCamera;
                computeTTCLidar(previousBox->lidarPoints, currentBox->lidarPoints, sensorFrameRate, ttcLidar);
                clusterKptMatchesWithROI(*currentBox, previousFrame.keypoints, currentFrame.keypoints, currentFrame.kptMatches);
                computeTTCCamera(previousFrame.keypoints, currentFrame.keypoints, currentBox->kptMatches,
                                 sensorFrameRate, ttcCamera);

                const double previousDistance = medianLidarDistance(previousBox->lidarPoints);
                const double currentDistance = medianLidarDistance(currentBox->lidarPoints);
                cout << "frames " << frameIndex - options.stepWidth << "->" << frameIndex
                     << " boxes " << previousBox->boxID << "->" << currentBox->boxID
                     << " TTC lidar=" << ttcLidar << " camera=" << ttcCamera << endl;

                if (csv)
                {
                    csv << options.detector << ',' << options.descriptor << ','
                        << frameIndex - options.stepWidth << ',' << frameIndex << ','
                        << previousBox->boxID << ',' << currentBox->boxID << ','
                        << (previousBox->boxID == leadPreviousBox && currentBox->boxID == leadCurrentBox ? 1 : 0) << ','
                        << currentBox->kptMatches.size() << ','
                        << previousBox->lidarPoints.size() << ',' << currentBox->lidarPoints.size() << ','
                        << previousDistance << ',' << currentDistance << ',' << ttcLidar << ',' << ttcCamera << '\n';
                }
                ++resultRows;

                if (options.visualize)
                {
                    cv::Mat visualization = currentFrame.cameraImg.clone();
                    showLidarImgOverlay(visualization, currentBox->lidarPoints, PRect, RRect, RT, &visualization);
                    cv::rectangle(visualization, currentBox->roi, cv::Scalar(0, 255, 0), 2);
                    ostringstream label;
                    label << fixed << setprecision(2) << "TTC Lidar: " << ttcLidar << " s, Camera: " << ttcCamera << " s";
                    cv::putText(visualization, label.str(), cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255));
                    cv::imshow("Final TTC results", visualization);
                    cv::waitKey(0);
                }
            }
        }

        cout << "Completed " << options.detector << "+" << options.descriptor
             << " with " << resultRows << " TTC rows" << endl;
        return 0;
    }
    catch (const exception &error)
    {
        cerr << "Error: " << error.what() << endl;
        return 2;
    }
}
