#include "FeatureTracker.h"

#include <opencv2/highgui/highgui.hpp> // imshow
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <iostream>

using namespace std;


FeatureTracker::FeatureTracker(const Params& params)
{
    params_m = params;
}


std::vector<cv::DMatch> ConvertMatches(std::vector<std::vector<cv::DMatch>>& knnMatches)
{
    std::vector<cv::DMatch> matches;
    for (auto& kMatches : knnMatches)
        matches.push_back(kMatches[0]);

    return matches;
}

void FilterMatches(std::vector<std::vector<cv::DMatch>>& knnMatches)
{
    cout << "FilterMatches" << "\n";
    const double minThresh = 0.8;
    // for each set of matches, compare best match with second best match
    cout << "Filtering Matches..." << "\n";

    int numMatches = knnMatches.size();
    int matchesFiltered = 0;
    //for (auto& kMatches : knnMatches)
    for (auto it = knnMatches.begin(); it != knnMatches.end();)
    {
        // delete this match if match is ambiguous (top two distances are too close)
        auto& matches = *it;
        double ratio = matches[0].distance / matches[1].distance;
        if (ratio > minThresh)
        {
            matchesFiltered++;
            it = knnMatches.erase(it);
        }
        else
            it++;
    }
    cout << "Filtered out " << matchesFiltered << " ambiguous matches out of " << numMatches << " total." << endl;
}



vector<cv::DMatch> FeatureTracker::TrackFeatures(const DataFrame& lastFrame, const DataFrame& newFrame)
{
    vector<cv::DMatch> matches;

    matches = matchDescriptors(lastFrame.keypoints, newFrame.keypoints,
                               lastFrame.descriptors, newFrame.descriptors);
        
    cout << "#7 : MATCH KEYPOINT DESCRIPTORS done" << endl;
    // visualize matches between current and previous image
    if (params_m.visualizeMatches) VisualizeMatches2(lastFrame, newFrame, matches);
    
    return matches;
}


// Find best matches for keypoints in two camera images based on several matching methods
std::vector<cv::DMatch> FeatureTracker::matchDescriptors(const std::vector<cv::KeyPoint> &kPtsSource,
                                                         const std::vector<cv::KeyPoint> &kPtsRef,
                                                         const cv::Mat &descSource,
                                                         const cv::Mat &descRef)
                                                         
{
    cout << "MatchDescriptors: " << endl;
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (params_m.matcherType.compare("MAT_BF") == 0)
        matcher = cv::BFMatcher::create(params_m.normType, crossCheck);
    else if (params_m.matcherType.compare("MAT_FLANN") == 0)
    {
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        // convert descriptors to correct datatype if using flann
        descSource.convertTo(descSource, CV_32F);
        descRef.convertTo(descRef, CV_32F);
    }

    std::vector<cv::DMatch> matches;
    // perform matching task
    if (params_m.selectorType.compare("SEL_NN") == 0) // nearest neighbor (best match)
    {
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (params_m.selectorType.compare("SEL_KNN") == 0)
    {
        cout << "about to run knn matching: " << endl;
        double t = (double)cv::getTickCount();
        int k = 2;
        std::vector<std::vector<cv::DMatch>> knnMatches;
        cout << "descSource size: " << descSource.size() << "\n";
        cout << "descRef size: " << descRef.size() << "\n";
        matcher->knnMatch(descSource, descRef, knnMatches, k);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (KNN) with n=" << knnMatches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
        // filter matches using descriptor distance ratio test
        FilterMatches(knnMatches);
        matches = ConvertMatches(knnMatches);
    }
    return matches;
}

// Better visualize matches function that draw one image and draws
// line from previous location to current location for each feature
void FeatureTracker::VisualizeMatches2(const DataFrame& lastFrame, const DataFrame& newFrame, const vector<cv::DMatch> matches)
{
    cv::Mat matchImg = newFrame.cameraImg.clone();

    for (auto match : matches)
    {
        auto pt1 = lastFrame.keypoints[match.queryIdx].pt;
        auto pt2 = newFrame.keypoints[match.trainIdx].pt;
        cv::line(matchImg, pt1, pt2, cv::Scalar(0,0,255));
    }

    string windowName = "Matching keypoints between two camera images";
    cv::namedWindow(windowName, 7);
    cv::imshow(windowName, matchImg);
    cout << "Press key to continue to next image" << endl;
    //cv::waitKey(params_m.cvWaitTime); // wait for key to be pressed
}

void FeatureTracker::VisualizeMatches(const DataFrame& lastFrame, const DataFrame& newFrame, const vector<cv::DMatch> matches)
{
    cv::Mat matchImg = newFrame.cameraImg.clone();
    cv::drawMatches(lastFrame.cameraImg, lastFrame.keypoints,
                    newFrame.cameraImg, newFrame.keypoints,
                    matches, matchImg,
                    cv::Scalar::all(-1), cv::Scalar::all(-1),
                    vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    string windowName = "Matching keypoints between two camera images";
    cv::namedWindow(windowName, 7);
    cv::imshow(windowName, matchImg);
    cout << "Press key to continue to next image" << endl;
    //cv::waitKey(params_m.cvWaitTime); // wait for key to be pressed
}

