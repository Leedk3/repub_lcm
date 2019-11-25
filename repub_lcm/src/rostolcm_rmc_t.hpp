#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include "std_msgs/Float64MultiArray.h"
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm.h>

#include "repub_lcm/lcm/eurecar/rmc_ros_t.hpp"

class ROSToLCM_RMC_T
{
    public:
        ROSToLCM_RMC_T(ros::NodeHandle& n, lcm::LCM* lh);
        ~ROSToLCM_RMC_T();
        void ros_rmc_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        
    private:        
        lcm::LCM* lcm_handle;
        ros::NodeHandle nh;
        ros::Subscriber ros_rmc_sub;
};

ROSToLCM_RMC_T::ROSToLCM_RMC_T(ros::NodeHandle& n, lcm::LCM* lh) 
{
    nh = n;
    lcm_handle = lh;
   
    // Subscribers
    ros_rmc_sub = nh.subscribe<std_msgs::Float64MultiArray>("GpsData", 10, &ROSToLCM_RMC_T::ros_rmc_callback, this);
};

ROSToLCM_RMC_T::~ROSToLCM_RMC_T() 
{    
    ROS_INFO("NAVIGATION ROSToLCM_RMC_T destructor.");
}

void ROSToLCM_RMC_T::ros_rmc_callback(const std_msgs::Float64MultiArray::ConstPtr& ros_msg)
{
    eurecar::rmc_ros_t* rmc_msg = new eurecar::rmc_ros_t();
    rmc_msg->utime = 0;
    rmc_msg-> time = ros_msg->data.at(12)*3600+ros_msg->data.at(13)*60+ros_msg->data.at(14);
    rmc_msg-> speed = ros_msg->data.at(20);
    rmc_msg-> heading = ros_msg->data.at(21);
    rmc_msg-> lat = ros_msg->data.at(22);
    rmc_msg-> lon = ros_msg->data.at(23);
    // std::cout<<"test"<<std::endl;
    lcm_handle->publish("RMC_ros_T", rmc_msg);
};     