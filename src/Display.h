#ifndef DISPLAY_H
#define DISPLAY_H

#include <opencv2/core.hpp>
#include "dataStructures.h" // LidarPoint
#include <vector>

void showLidarTopview(std::vector<LidarPoint> &lidarPoints, cv::Size worldSize, cv::Size imageSize, bool bWait=true);
void showLidarImgOverlay(const cv::Mat &img,
                         const std::vector<LidarPoint> &lidarPoints,
                         const cv::Mat &P_rect_xx,
                         const cv::Mat &R_rect_xx,
                         const cv::Mat &RT,
                         const cv::Mat *extVisImg=nullptr);
cv::Mat DrawLidarTopviewMat(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait=true);

#endif /* DISPLAY_H */
