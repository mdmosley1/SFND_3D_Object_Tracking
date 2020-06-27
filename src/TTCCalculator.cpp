#include "TTCCalculator.h"
#include <vector>
#include <iostream>

using namespace std;


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

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox,
                              const std::vector<cv::KeyPoint> &kptsPrev,
                              const std::vector<cv::KeyPoint> &kptsCurr,
                              const std::vector<cv::DMatch> &kptMatches)
{
    assert(!kptMatches.empty());
    assert(!kptsPrev.empty());
    assert(!kptsCurr.empty());
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


TTCResults TTCCalculator::ComputeTTC(const BoundingBox& prevBB,
                      const BoundingBox& _currBB,
                      const double sensorFrameRate,
                      const DataFrame& lastFrame,
                      const DataFrame& currentFrame)
{
    // make copy since this gets modified to add kpts 
    BoundingBox currBB = _currBB;
    
    double ttcLidar = computeTTCLidar(prevBB.lidarPoints,
                                      currBB.lidarPoints,
                                      sensorFrameRate);
    cout << "TTC lidar = " << ttcLidar << "\n";

    clusterKptMatchesWithROI(currBB,
                             lastFrame.keypoints,
                             currentFrame.keypoints,
                             currentFrame.kptMatches);

    double ttcCamera = computeTTCCamera(lastFrame.keypoints,
                                        currentFrame.keypoints,
                                        currBB.kptMatches,
                                        sensorFrameRate);
    cout << "TTC camera = " << ttcCamera << "\n";

    TTCResults results;
    results.camera = ttcCamera;
    results.lidar = ttcLidar;
    return results;
}
