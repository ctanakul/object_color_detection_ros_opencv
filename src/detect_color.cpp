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
    cv::namedWindow("Image Window");
}

void colorDetector::image_cb(const sensor_msgs::ImageConstPtr &msg)
{
    std::cout << "agjdsh" << std::endl;
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

    cv::imshow("Image Window", cv_ptr->image);
    cv::waitKey(3);
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