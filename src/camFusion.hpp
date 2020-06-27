
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
    

                     
#endif /* camFusion_hpp */
