#include "util.h"
#include <memory> // unique_ptr
#include <vector>


#include "matching2D.hpp" // KPDetector

using namespace std;

const bool bLimitKpts = false;
bool bVis = true;            // visualize results


void LimitKeyPoints(vector<cv::KeyPoint>& keypoints, const Params& p)
{
    int maxKeypoints = 50;

    if (p.detectorType.compare("SHITOMASI") == 0)
    { // there is no response info, so keep the first 50 as they are sorted in descending quality order
        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
    }
    cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
    cout << " NOTE: Keypoints have been limited!" << endl;
}

void LimitKeyPointsRect(vector<cv::KeyPoint>& keypoints)
{
    cv::Rect vehicleRect(535, 180, 180, 150);
    for (auto it = keypoints.begin(); it != keypoints.end(); )
    {
        // delete keypoint if not inside rectangle
        if (vehicleRect.contains(it->pt))
            it++;
        else
            it = keypoints.erase(it);
    }
}


DataFrame DetectAndDescribeFeatures(const cv::Mat& imgGray,
                            const std::unique_ptr<KPDetector>& _detector,
                            const cv::Ptr<cv::DescriptorExtractor>& _descriptor,
                            const Params& params)
{
    cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;
    // extract 2D keypoints from current image
    vector<cv::KeyPoint> keypoints; // create empty feature list for current image        
    keypoints = _detector->DetectKeypoints(imgGray, false);

    // optional : limit number of keypoints (helpful for debugging and learning)
    if (bLimitKpts) LimitKeyPoints(keypoints, params);

    //// TASK MP.3 -> only keep keypoints on the preceding vehicle
    if (params.bFocusOnVehicle) LimitKeyPointsRect(keypoints);
    
    cout << "#2 : DETECT KEYPOINTS done" << endl;
    cv::Mat descriptors = descKeypoints(keypoints, imgGray, _descriptor, params);
    // push descriptors for current frame to end of data buffer
    DataFrame newFrame(imgGray, keypoints, descriptors);
    cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

    return newFrame;
    
}

std::unique_ptr<KPDetector> CreateDetector(std::string _detectorType)
{
    std::unique_ptr<double> d = std::make_unique<double>();
    
    cout << "Creating detector with type: " << _detectorType << "\n";
    std::unique_ptr<KPDetector> detector;
    if (_detectorType.compare("SHITOMASI") == 0)
        detector = std::make_unique<DetectorShiTomasi>();
    else if (_detectorType.compare("HARRIS") == 0)
        detector = std::make_unique<DetectorHarris>();
    else if (_detectorType.compare("FAST") == 0)
        detector = std::make_unique<DetectorFast>();
    else if (_detectorType.compare("BRISK") == 0)
        detector = std::make_unique<DetectorBrisk>();
    else if (_detectorType.compare("ORB") == 0)
        detector = std::make_unique<DetectorOrb>();
    else if (_detectorType.compare("AKAZE") == 0)
        detector = std::make_unique<DetectorAkaze>();
    else if (_detectorType.compare("SIFT") == 0)
        detector = std::make_unique<DetectorSift>();
    else
    {
        cout << "Error: " << _detectorType  << " is not a valid detector type!"<< "\n";
        exit(0);
    }
    return detector;
}

cv::Ptr<cv::DescriptorExtractor> CreateDescriptor(std::string _descriptorType)
{
    cout << "Creating descriptor of type: " << _descriptorType << "\n";
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (_descriptorType.compare("BRISK") == 0)
    {
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (_descriptorType.compare("BRIEF") == 0)
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    else if (_descriptorType.compare("ORB") == 0)
        extractor = cv::ORB::create();
    else if (_descriptorType.compare("FREAK") == 0)
        extractor = cv::xfeatures2d::FREAK::create();
    else if (_descriptorType.compare("AKAZE") == 0)
        extractor = cv::AKAZE::create();
    else if (_descriptorType.compare("SIFT") == 0)
        extractor = cv::SIFT::create();
    else
    {
        cout << _descriptorType  << " is not a valid descriptor type!"<< "\n";
        exit(0);
    }

    return extractor;
}
