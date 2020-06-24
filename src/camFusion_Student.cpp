
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h" // Calibration
#include <map>

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void AddLidarPointsToBoxes(std::vector<BoundingBox> &boundingBoxes,
                           const std::vector<LidarPoint> &lidarPoints,
                           const float shrinkFactor,
                           const Calibration& cal)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = cal.P_rect_00 * cal.R_rect_00 * cal.RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }
    } // eof loop over all Lidar points
    cout << "#4 : CLUSTER LIDAR POINT CLOUD done" << endl;
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left, bottom+50), cv::FONT_ITALIC, 0.8, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left, bottom+100), cv::FONT_ITALIC, 0.8, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox,
                              const std::vector<cv::KeyPoint> &kptsPrev,
                              const std::vector<cv::KeyPoint> &kptsCurr,
                              const std::vector<cv::DMatch> &kptMatches)
{
    unsigned int numKeypointsAdded = 0;
    // iterate over all kptMatches. Check if both prevKpt and currKpt are contained within bounding box
    for (auto match :  kptMatches)
    {
        auto currKpt = kptsCurr[match.trainIdx];
        auto prevKpt = kptsPrev[match.queryIdx];
        
        // check if both keypoints are inside bounding box
        if (boundingBox.roi.contains(currKpt.pt)
            && boundingBox.roi.contains(prevKpt.pt))
        {
            boundingBox.kptMatches.push_back(match);
            numKeypointsAdded++;
        }
    }
    cout << "Number of keypoint matches added to bounding box " << boundingBox.boxID <<  ": " << numKeypointsAdded << endl;
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
double computeTTCCamera(const std::vector<cv::KeyPoint> &kptsPrev,
                        const std::vector<cv::KeyPoint> &kptsCurr,
                        const std::vector<cv::DMatch> kptMatches,
                        const double frameRate)
{
    assert(!kptsPrev.empty());
    assert(!kptsCurr.empty());
    assert(!kptMatches.empty());
    double TTC = 0.0;
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        cout << __FUNCTION__ << "Warning: list of distance ratios is empty!" << endl;
        return TTC;
    }

    // compute camera-based TTC from distance ratios
    double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();
    // compute median distance ratio
    std::sort(distRatios.begin(), distRatios.end());
    double medianDistRatio = distRatios[distRatios.size()/2];

    // cout << "mean is : " << meanDistRatio << "\n";
    // cout << "median is : " << medianDistRatio << "\n";

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medianDistRatio);
    return TTC;
}

double GetMedianX(const std::vector<LidarPoint> &lidarPoints)
{
    std::vector<double> values;
    for (auto& pt : lidarPoints) values.push_back(pt.x);
        
    std::sort(values.begin(), values.end());
    return values[values.size()/2];
}

double computeTTCLidar(const std::vector<LidarPoint> &lidarPointsPrev,
                       const std::vector<LidarPoint> &lidarPointsCurr,
                       const double frameRate)
{
    double TTC = 0.0;
    // auxiliary variables
    double dT = 1.0 / frameRate ; // time between two measurements in seconds

    // find closest distance to Lidar points 
    double medXCurr = GetMedianX(lidarPointsCurr);
    double medXPrev = GetMedianX(lidarPointsPrev);

    // compute TTC from both measurements
    TTC = medXCurr * dT / (medXPrev - medXCurr);
    return TTC;
}

// return index of first bounding box that contains the keypoint
int FindBboxId(const cv::KeyPoint& currKpt, const std::vector<BoundingBox>& bboxes)
{
    // check which bounding box contains the given keypoint
    for (int i = 0; i < bboxes.size(); ++i)
    {
        if (bboxes[i].roi.contains(currKpt.pt))
            return bboxes[i].boxID;
    }
    return -1;
}

// Function tp find the Entry with largest Value in a Map 
int FindBoxIdWithMostMatches( std::map<int, int> sampleMap) 
{ 
    // Reference variable to help find 
    // the entry with the highest value 
    std::pair<int, int> entryWithMaxValue = std::make_pair(-1, 0);
  
    // Iterate in the map to find the required entry 
    std::map<int, int>::iterator currentEntry; 
    for (currentEntry = sampleMap.begin(); currentEntry != sampleMap.end(); ++currentEntry)
        if (currentEntry->second > entryWithMaxValue.second)
            entryWithMaxValue = make_pair( currentEntry->first, currentEntry->second); 
  
    return entryWithMaxValue.first; 
}



// show the bounding box indices for each frame so I know if they are matched correctly
void VisualizeBboxMatches(const DataFrame& frame, std::string name)
{

    cv::Mat visImg = frame.cameraImg.clone();
    int idx = 0;
    for(auto it = frame.boundingBoxes.begin(); it != frame.boundingBoxes.end(); ++it)
    {
        // Draw rectangle displaying the bounding box
        int top, left, width, height;
        top = (*it).roi.y;
        left = (*it).roi.x;
        width = (*it).roi.width;
        height = (*it).roi.height;
        cv::rectangle(visImg, cv::Point(left, top), cv::Point(left+width, top+height),cv::Scalar(0, 255, 0), 2);

        std::string label = "bbox: " + std::to_string(idx++);
        //string label = cv::format("%.2f", (*it).confidence);
        //label = classes[((*it).classID)] + ":" + label;
        
        // Display label at the top of the bounding box
        int baseLine;
        cv::Size labelSize = getTextSize(label, cv::FONT_ITALIC, 0.5, 1, &baseLine);
        top = max(top, labelSize.height);
        rectangle(visImg, cv::Point(left, top - round(1.5*labelSize.height)), cv::Point(left + round(1.5*labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
        cv::putText(visImg, label, cv::Point(left, top), cv::FONT_ITALIC, 0.75, cv::Scalar(0,0,0),1);
    }

    cv::imshow(name, visImg);
}


// match bounding boxes between current dataframe and last dataframe
// based on keypoint correspondences
std::map<int, int> matchBoundingBoxes(const DataFrame& prevFrame,
                                       const DataFrame& currFrame)
{
    std::map<int,int> bboxBestMatches;
    std::multimap<int, int> bboxMatches;
    // for each match in keypoint match vector of current frame
    for (auto match :  currFrame.kptMatches)
    {
        auto currKpt = currFrame.keypoints[match.trainIdx];
        auto prevKpt = prevFrame.keypoints[match.queryIdx];
        
        // find index of enclosing bbox (if any) in current frame for current frame keypoint
        int bboxIdCurr = FindBboxId(currKpt, currFrame.boundingBoxes);

        // find index of enclosing bbox (if any) in prev frame for prev frame keypoint
        int bboxIdPrev = FindBboxId(prevKpt, prevFrame.boundingBoxes);

        // if either keypoint is not enclosed by bounding box, then skip 
        if (bboxIdCurr == -1 || bboxIdPrev == -1)
            continue;

        // increment number of matches for currentBbox -> prevBbox
        bboxMatches.emplace(bboxIdCurr, bboxIdPrev);
    }

    // process the multimap for each bounding box in currFrame
    cout << "Printing out bbox matches:" << "\n";
    // iterate over each bounding box from current frame and find the best match to bounding box in previous frame
    //for (int currBoxId = 0; currBoxId < currFrame.boundingBoxes.size(); ++currBoxIdx)
    for (auto& bbox : currFrame.boundingBoxes)
    {
        // create new std::map
        std::map<int,int> prevBboxIdToNumMatches;
        // for each entry in bboxMatches multimap
        for (auto match : bboxMatches)
        {
            //  increment number of matches for prevBboxIdx
            if (match.first == bbox.boxID)
                prevBboxIdToNumMatches[match.second]++;
        }
        int bestPrevBboxId = FindBoxIdWithMostMatches(prevBboxIdToNumMatches);
        bboxBestMatches.emplace(bestPrevBboxId, bbox.boxID);

        cout << "BBox from current frame: " << bbox.boxID << "\n";
        cout << "BBox from prev frame: " << bestPrevBboxId << "\n";
        cout << "\n";
    }

    cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << endl;
    // VisualizeBboxMatches(prevFrame, "Previous image bboxes");
    // VisualizeBboxMatches(currFrame, "Current image bboxes");

    return bboxBestMatches;
}
