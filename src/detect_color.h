#ifndef DETECT_COLOR_H
#define DETECT_COLOR_H

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

class colorDetector
{
    public:
        colorDetector();
        int add(int a, int b);
        void image_cb(const sensor_msgs::ImageConstPtr& msg);

    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        int lowH_;
};

#endif