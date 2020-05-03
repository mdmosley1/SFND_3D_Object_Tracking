#ifndef FEATURETRACKER_H
#define FEATURETRACKER_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <vector>

#include "dataStructures.h" // DataFrame
#include "SettingsParser.h" // Params

class FeatureTracker
{
public:
    FeatureTracker(const Params& params);
    
    std::vector<cv::DMatch> TrackFeatures(const DataFrame& lastFrame, const DataFrame& newFrame);

private:
    void VisualizeMatches(const DataFrame& lastFrame,
                          const DataFrame& newFrame,
                          const std::vector<cv::DMatch> matches);
    std::vector<cv::DMatch> matchDescriptors(const std::vector<cv::KeyPoint> &kPtsSource,
                                             const std::vector<cv::KeyPoint> &kPtsRef,
                                             const cv::Mat &descSource,
                                             const cv::Mat &descRef);


    Params params_m;
};



#endif /* FEATURETRACKER_H */
