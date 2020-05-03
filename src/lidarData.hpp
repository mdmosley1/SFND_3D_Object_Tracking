
#ifndef lidarData_hpp
#define lidarData_hpp

#include <stdio.h>
#include <fstream>
#include <string>

#include "dataStructures.h"

void cropLidarPoints(std::vector<LidarPoint> &lidarPoints, float minX, float maxX, float maxY, float minZ, float maxZ, float minR);
void loadLidarFromFile(std::vector<LidarPoint> &lidarPoints, std::string filename);

void showLidarTopview(std::vector<LidarPoint> &lidarPoints, cv::Size worldSize, cv::Size imageSize, bool bWait=true);
void showLidarImgOverlay(const cv::Mat &img,
                         const std::vector<LidarPoint> &lidarPoints,
                         const cv::Mat &P_rect_xx,
                         const cv::Mat &R_rect_xx,
                         const cv::Mat &RT,
                         const cv::Mat *extVisImg=nullptr);
#endif /* lidarData_hpp */
