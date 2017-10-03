#include <cv_bridge/cv_bridge.h>
#include "detect_color.h"
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

colorDetector::colorDetector()
    : it_(nh_)
{
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &colorDetector::image_cb, this);
    lowH_ = 25;
    cv::namedWindow("thresh_img", CV_WINDOW_AUTOSIZE); 
    cv::createTrackbar("LowH", "thresh_img", &lowH_, 179);
}

void colorDetector::image_cb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));
    cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2HSV);

    std::vector<int> low_hsv = {25, 83, 113};
    std::vector<int> high_hsv = {86, 255, 255};

    cv::inRange(cv_ptr->image, low_hsv, high_hsv, cv_ptr->image);

    cv::imshow("Image Window", cv_ptr->image);
    cv::waitKey(1);
}

int colorDetector::add(int a, int b)
{
    std::cout << a + b << std::endl;
    return a + b;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ColorDetector");
    colorDetector cd;
    ros::spin();
    return 0;
}