
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
                         
void clusterKptMatchesWithROI(BoundingBox &boundingBox,
                              const std::vector<cv::KeyPoint> &kptsPrev,
                              const std::vector<cv::KeyPoint> &kptsCurr,
                              const std::vector<cv::DMatch> &kptMatches);

std::map<int, int> matchBoundingBoxes(const DataFrame& prevFrame,
                                      const DataFrame& currFrame);
    

void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait=true);

double computeTTCCamera(const std::vector<cv::KeyPoint> &kptsPrev,
                        const std::vector<cv::KeyPoint> &kptsCurr,
                        const std::vector<cv::DMatch> kptMatches,
                        const double frameRate);
                      

double computeTTCLidar(const std::vector<LidarPoint> &lidarPointsPrev,
                       const std::vector<LidarPoint> &lidarPointsCurr,
                       const double frameRate);

                     
#endif /* camFusion_hpp */
