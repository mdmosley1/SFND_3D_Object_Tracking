#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <string>

// data location
const std::string dataPath = "../";

// camera
const std::string imgBasePath = dataPath + "images/";
const std::string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
const std::string imgFileType = ".png";
const int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
const int imgEndIndex = 45;   // last file index to load
const int imgStepWidth = 1;
const double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
const int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

// object detection
const std::string yoloBasePath = dataPath + "dat/yolo/";
const std::string yoloClassesFile = yoloBasePath + "coco.names";
const std::string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
const std::string yoloModelWeights = yoloBasePath + "yolov3.weights";

// Lidar
const std::string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
const std::string lidarFileType = ".bin";

// associate Lidar points with camera-based ROI
const float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI

#endif /* CONSTANTS_H */
