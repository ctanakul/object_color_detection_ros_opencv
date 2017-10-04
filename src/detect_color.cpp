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
    low_h_ = 33;
    high_h_ = 81;
    low_s_ = 83;
    high_s_ = 255;
    low_v_ = 113;
    high_v_ = 255;
    blur_k_size_ = 1;
    dil_k_size_ = 1;
    cv::namedWindow("thresh_img", CV_WINDOW_AUTOSIZE);
    cv::createTrackbar("low_h_", "thresh_img", &low_h_, 179);
    cv::createTrackbar("high_h_", "thresh_img", &high_h_, 179);
    cv::createTrackbar("low_s_", "thresh_img", &low_s_, 255);
    cv::createTrackbar("high_s_", "thresh_img", &high_s_, 255);
    cv::createTrackbar("low_v_", "thresh_img", &low_v_, 255);
    cv::createTrackbar("high_v_", "thresh_img", &high_v_, 255);
    cv::createTrackbar("blur_k_size_", "thresh_img", &blur_k_size_, 15);
    cv::createTrackbar("dil_k_size_", "thresh_img", &dil_k_size_, 15);
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

    std::vector<int> low_hsv = {low_h_, low_s_, low_v_};
    std::vector<int> high_hsv = {high_h_, high_s_, high_v_};

    cv::inRange(cv_ptr->image, low_hsv, high_hsv, cv_ptr->image);
    cv::blur(cv_ptr->image, cv_ptr->image, cv::Size(blur_k_size_, blur_k_size_));
    // if ((blur_k_size_ % 2) != 1)
    // {
    //     blur_k_size_++;
    // }
    // cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(blur_k_size_, blur_k_size_), 0);
    cv::imshow("Before", cv_ptr->image);
    cv::dilate(cv_ptr->image, cv_ptr->image, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * dil_k_size_ + 1, 2 * dil_k_size_ + 1), cv::Point(dil_k_size_, dil_k_size_)));
    // cv::erode(cv_ptr->image, cv_ptr->image, 5);
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