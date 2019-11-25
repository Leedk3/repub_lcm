#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include "std_msgs/Float64MultiArray.h"
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm.h>
#include "repub_lcm/lcm/eurecar/gga_ros_t.hpp"

class ROSToLCM_GGA_T
{
    public:
        ROSToLCM_GGA_T(ros::NodeHandle& n, lcm::LCM* lh);
        ~ROSToLCM_GGA_T();
        void ros_gga_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        
    private:        
        lcm::LCM* lcm_handle;
        ros::NodeHandle nh;
        ros::Subscriber ros_gga_sub;
};

ROSToLCM_GGA_T::ROSToLCM_GGA_T(ros::NodeHandle& n, lcm::LCM* lh) 
{
    nh = n;
    lcm_handle = lh;
   
    // Subscribers
    ros_gga_sub = nh.subscribe<std_msgs::Float64MultiArray>("GpsData", 10, &ROSToLCM_GGA_T::ros_gga_callback, this);
};

ROSToLCM_GGA_T::~ROSToLCM_GGA_T() 
{    
    ROS_INFO("NAVIGATION ROSToLCM_GGA_T destructor.");
}

void ROSToLCM_GGA_T::ros_gga_callback(const std_msgs::Float64MultiArray::ConstPtr& ros_msg)
{
    eurecar::gga_ros_t* gga_msg = new eurecar::gga_ros_t();
    gga_msg->utime = 0;
    gga_msg-> time = ros_msg->data.at(0)*3600+ros_msg->data.at(1)*60+ros_msg->data.at(2);
    gga_msg-> x = 0;
    gga_msg-> y = 0;
    gga_msg-> h = ros_msg->data.at(10);
    gga_msg-> numSV = (int)ros_msg->data.at(8);
    gga_msg-> postype = (int)ros_msg->data.at(7);
    gga_msg-> lag = (int)ros_msg->data.at(0);
    gga_msg-> hordev = ros_msg->data.at(9);
    gga_msg-> lat = ros_msg->data.at(22);
    gga_msg-> lon = ros_msg->data.at(23);
    gga_msg-> hgt = ros_msg->data.at(11);
    // std::cout<<"test"<<std::endl;
    lcm_handle->publish("GGA_ros_T", gga_msg);
};