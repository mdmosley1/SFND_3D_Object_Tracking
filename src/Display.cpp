#include "Display.h"
#include <opencv2/highgui/highgui.hpp> // imshow
#include <opencv2/imgproc.hpp> // circle, line

using namespace std;


void showLidarTopview(std::vector<LidarPoint> &lidarPoints, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

    // plot Lidar points into image
    for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
    {
        float xw = (*it).x; // world position in m with x facing forward from sensor
        float yw = (*it).y; // world position in m with y facing left from sensor

        int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
        int x = (-yw * imageSize.height / worldSize.height) + imageSize.width / 2;

        cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
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
    string windowName = "Top-View Perspective of LiDAR data";
    cv::namedWindow(windowName, 2);
    cv::imshow(windowName, topviewImg);
    if(bWait)
    {
        //cv::waitKey(0); // wait for key to be pressed
    }
}

void showLidarImgOverlay(const cv::Mat &img,
                         const std::vector<LidarPoint> &lidarPoints,
                         const cv::Mat &P_rect_xx,
                         const cv::Mat &R_rect_xx,
                         const cv::Mat &RT,
                         const cv::Mat *extVisImg)
{
    // init image for visualization
    cv::Mat visImg; 
    if(extVisImg==nullptr)
    {
        visImg = img.clone();
    } else 
    {
        visImg = *extVisImg;
    }

    cv::Mat overlay = visImg.clone();

    // find max. x-value
    double maxVal = 0.0; 
    for(auto it=lidarPoints.begin(); it!=lidarPoints.end(); ++it)
    {
        maxVal = maxVal<it->x ? it->x : maxVal;
    }

    cv::Mat X(4,1,cv::DataType<double>::type);
    cv::Mat Y(3,1,cv::DataType<double>::type);
    for(auto it=lidarPoints.begin(); it!=lidarPoints.end(); ++it) {

            X.at<double>(0, 0) = it->x;
            X.at<double>(1, 0) = it->y;
            X.at<double>(2, 0) = it->z;
            X.at<double>(3, 0) = 1;

            Y = P_rect_xx * R_rect_xx * RT * X;
            cv::Point pt;
            pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2);
            pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

            float val = it->x;
            int red = min(255, (int)(255 * abs((val - maxVal) / maxVal)));
            int green = min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
            cv::circle(overlay, pt, 5, cv::Scalar(0, green, red), -1);
    }

    float opacity = 0.6;
    cv::addWeighted(overlay, opacity, visImg, 1 - opacity, 0, visImg);

    // return augmented image or wait if no image has been provided
    if (extVisImg == nullptr)
    {
        string windowName = "LiDAR data on image overlay";
        cv::namedWindow( windowName, 3 );
        cv::imshow( windowName, visImg );
        //cv::waitKey(0); // wait for key to be pressed
    }
    else
    {
        extVisImg = &visImg;
    }
}

cv::Mat DrawLidarTopviewMat(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
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
    // float lineSpacing = 2.0; // gap between distance markers
    // int nMarkers = floor(worldSize.height / lineSpacing);
    // for (size_t i = 0; i < nMarkers; ++i)
    // {
    //     int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
    //     cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    // }

    return topviewImg;
}
