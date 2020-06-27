/* INCLUDES FOR THIS PROJECT */
#include <iostream>
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
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"
#include "SettingsParser.h" // LoadParamsFromFile, Params
#include "util.h" // CreateDetector, CreateDescriptor
#include "FeatureTracker.h" // FeatureTracker
#include "ObjectTracker3D.h"
#include "TTCCalculator.h"
#include "constants.h"

using namespace std;

int dataBufferSize_g = 2;       // no. of images which are held in memory (ring buffer) at the same time
vector<DataFrame> dataBuffer_g; // list of data frames which are held in memory at the same time

void AddToRingBuffer(const DataFrame& frame)
{
    dataBuffer_g.push_back(frame);
    while (dataBuffer_g.size() > dataBufferSize_g)
        dataBuffer_g.erase(dataBuffer_g.begin());
}


void UpdateDisplay(cv::Mat visImg,
                   const BoundingBox& currBB,
                   const Calibration& cal,
                   const TTCResults& ttcResults)
{
    
    showLidarImgOverlay(visImg,
                        currBB.lidarPoints,
                        cal.P_rect_00,
                        cal.R_rect_00,
                        cal.RT,
                        &visImg);
    cv::rectangle(visImg,
                  cv::Point(currBB.roi.x, currBB.roi.y),
                  cv::Point(currBB.roi.x + currBB.roi.width,currBB.roi.y + currBB.roi.height),
                  cv::Scalar(0, 255, 0), 2);
                        
    char str[200];
    sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcResults.lidar, ttcResults.camera);
    putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255));

    string windowName = "Final Results : TTC";
    cv::namedWindow(windowName, 4);
    cv::imshow(windowName, visImg);
    cout << "Press key to continue to next frame" << endl;
    cv::waitKey(0);
}


Calibration InitializeCalibration()
{
// calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;
    
    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;
    
    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;

    Calibration cal;
    cal.P_rect_00 = P_rect_00;
    cal.RT = RT;
    cal.R_rect_00 = R_rect_00;

    return cal;
}

std::vector<BoundingBox> GetClassifiedBoundingBoxesFromImage(cv::Mat img)
{
    static float confThreshold = 0.2;
    //static float nmsThreshold = 0.4;
    static float nmsThreshold = 0.1;
    bool bVis = true;
    auto boundingBoxes = detectObjects(img,
                                       confThreshold,
                                       nmsThreshold,
                                       yoloBasePath,
                                       yoloClassesFile,
                                       yoloModelConfiguration,
                                       yoloModelWeights, bVis);

    cout << "#2 : DETECT & CLASSIFY OBJECTS done" << endl;
    
    return boundingBoxes;
}

std::vector<LidarPoint> GetCroppedLidarPoints(const size_t _index)
{
    ostringstream imgNumber;
    imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + _index;
    // load 3D Lidar points from file
    string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
    std::vector<LidarPoint> lidarPoints;
    loadLidarFromFile(lidarPoints, lidarFullFilename);

    // remove Lidar points based on distance properties
    static float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
    cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);
    
    cout << "#3 : CROP LIDAR POINTS done" << endl;

    return lidarPoints;
}

cv::Mat GetNextImage(const size_t _index)
{
    ostringstream imgNumber;
    imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + _index;
    string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

    return cv::imread(imgFullFilename);
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    Calibration cal = InitializeCalibration();    
    
    auto params = LoadParamsFromFile("../src/settings.txt");
    auto detector = CreateDetector(params.detectorType);
    auto descriptor = CreateDescriptor(params.descriptorType);

    FeatureTracker featureTracker(params);
    ObjectTracker3D objectTracker(featureTracker);
    TTCCalculator ttcCalculator;

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex+=imgStepWidth)
    {
        // assemble filenames for current index
        cv::Mat img = GetNextImage(imgIndex);        

        auto boundingBoxes = GetClassifiedBoundingBoxesFromImage(img);
        auto lidarPoints = GetCroppedLidarPoints(imgIndex); 
        AddLidarPointsToBoxes(boundingBoxes, lidarPoints, shrinkFactor, cal);
        //if(true) show3DObjects(boundingBoxes, cv::Size(4.0, 20.0), cv::Size(900,900), true);
        if(true) show3DObjects(boundingBoxes, cv::Size(10.0, 20.0), cv::Size(900,900), true);        
        DataFrame newFrame = DetectAndDescribeFeatures(img, detector, descriptor, params);
        newFrame.boundingBoxes = boundingBoxes;
        AddToRingBuffer(newFrame);

        if (dataBuffer_g.size() > 1) // wait until at least two images have been processed
        {
            auto lastFrame = dataBuffer_g.end() - 2;
            auto currentFrame = dataBuffer_g.end() - 1;
            BBoxPair bboxPair = objectTracker.ProcessImagePair(*lastFrame, *currentFrame);

            if (bboxPair.curr != nullptr && bboxPair.prev != nullptr)
            {
                // compute lidar and camera TTC for the bounding box
                // corresponding to vehicle in ego lane
                TTCResults results = ttcCalculator.ComputeTTC(*bboxPair.prev,
                                                              *bboxPair.curr,
                                                              sensorFrameRate,
                                                              *lastFrame,
                                                              *currentFrame);

                cv::Mat visImg = (dataBuffer_g.end() - 1)->cameraImg.clone();
                UpdateDisplay(visImg, *bboxPair.curr, cal, results);
            }
        }
    }
    return 0;
}
