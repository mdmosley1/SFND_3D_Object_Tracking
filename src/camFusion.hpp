
#ifndef camFusion_hpp
#define camFusion_hpp

#include <stdio.h>
#include <vector>
#include <opencv2/core.hpp>
#include "dataStructures.h"


void AddLidarPointsToBoxes(std::vector<BoundingBox> &boundingBoxes,
                           const std::vector<LidarPoint> &lidarPoints,
                           const float shrinkFactor,
                           const Calibration& cal);
                         
void clusterKptMatchesWithROI(const BoundingBox &boundingBox,
                              const std::vector<cv::KeyPoint> &kptsPrev,
                              const std::vector<cv::KeyPoint> &kptsCurr,
                              const std::vector<cv::DMatch> &kptMatches);
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame);

void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait=true);

void computeTTCCamera(const std::vector<cv::KeyPoint> &kptsPrev,
                      const std::vector<cv::KeyPoint> &kptsCurr,
                      const std::vector<cv::DMatch> kptMatches,
                      const double frameRate,
                      double &TTC,
                      cv::Mat *visImg=nullptr);
void computeTTCLidar(const std::vector<LidarPoint> &lidarPointsPrev,
                     const std::vector<LidarPoint> &lidarPointsCurr,
                     const double frameRate,
                     double &TTC);                  
#endif /* camFusion_hpp */
