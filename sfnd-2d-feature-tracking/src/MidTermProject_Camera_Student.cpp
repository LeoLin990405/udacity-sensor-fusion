/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    const vector<string> detectorTypes = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    const vector<string> descriptorTypes = {"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};
    const string detectorType = argc > 1 ? argv[1] : "SHITOMASI";
    const string descriptorType = argc > 2 ? argv[2] : "BRISK";
    const string dataPath = argc > 3 ? argv[3] : "../";
    const bool bVis = argc > 4 && string(argv[4]) == "--visualize";

    const auto isSupported = [](const vector<string> &choices, const string &value) {
        return find(choices.begin(), choices.end(), value) != choices.end();
    };
    if (!isSupported(detectorTypes, detectorType) || !isSupported(descriptorTypes, descriptorType) || argc > 5)
    {
        cerr << "Usage: " << argv[0] << " [DETECTOR] [DESCRIPTOR] [DATA_ROOT] [--visualize]" << endl;
        return 1;
    }
    if (descriptorType == "AKAZE" && detectorType != "AKAZE")
    {
        cerr << "AKAZE descriptors require AKAZE keypoints." << endl;
        return 2;
    }
    if (detectorType == "SIFT" && descriptorType == "ORB")
    {
        cerr << "SIFT keypoints are incompatible with the ORB descriptor in this OpenCV build." << endl;
        return 2;
    }

    // data location
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    double totalDetectorMs = 0.0;
    double totalDescriptorMs = 0.0;
    size_t totalKeypoints = 0;
    size_t totalMatches = 0;

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        if (img.empty())
        {
            cerr << "Unable to load image: " << imgFullFilename << endl;
            return 3;
        }
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        DataFrame frame;
        frame.cameraImg = imgGray;
        if (dataBuffer.size() >= dataBufferSize)
        {
            dataBuffer.erase(dataBuffer.begin());
        }
        dataBuffer.push_back(frame);

        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        double detectorStart = static_cast<double>(cv::getTickCount());

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray, false);
        }
        else
        {
            detKeypointsModern(keypoints, imgGray, detectorType, false);
        }
        double detectorMs = 1000.0 * (static_cast<double>(cv::getTickCount()) - detectorStart) / cv::getTickFrequency();
        totalDetectorMs += detectorMs;
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            vector<cv::KeyPoint> vehicleKeypoints;
            for (auto keypoint : keypoints)
            {
                if (vehicleRect.contains(keypoint.pt))
                {
                    vehicleKeypoints.push_back(keypoint);
                }
            }
            keypoints = vehicleKeypoints;
        }

        double meanKeypointSize = 0.0;
        for (const auto &keypoint : keypoints)
        {
            meanKeypointSize += keypoint.size;
        }
        if (!keypoints.empty())
        {
            meanKeypointSize /= keypoints.size();
        }
        totalKeypoints += keypoints.size();

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        double descriptorStart = static_cast<double>(cv::getTickCount());
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        double descriptorMs = 1000.0 * (static_cast<double>(cv::getTickCount()) - descriptorStart) / cv::getTickFrequency();
        totalDescriptorMs += descriptorMs;
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        size_t matchCount = 0;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorCategory = descriptorType == "SIFT" ? "DES_HOG" : "DES_BINARY";
            string selectorType = "SEL_KNN";      // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorCategory, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;
            matchCount = matches.size();
            totalMatches += matchCount;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
        }

        cout << fixed << setprecision(3)
             << "METRIC," << imgStartIndex + imgIndex << "," << detectorType << "," << descriptorType << ","
             << keypoints.size() << "," << meanKeypointSize << "," << matchCount << ","
             << detectorMs << "," << descriptorMs << endl;

    } // eof loop over all images

    const size_t imageCount = imgEndIndex - imgStartIndex + 1;
    const size_t matchedFrameCount = imageCount > 0 ? imageCount - 1 : 0;
    cout << fixed << setprecision(3)
         << "SUMMARY," << detectorType << "," << descriptorType << ","
         << static_cast<double>(totalKeypoints) / imageCount << ","
         << (matchedFrameCount > 0 ? static_cast<double>(totalMatches) / matchedFrameCount : 0.0) << ","
         << totalDetectorMs << "," << totalDescriptorMs << ","
         << totalDetectorMs + totalDescriptorMs << endl;

    return 0;
}
