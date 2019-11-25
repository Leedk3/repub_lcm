#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include "std_msgs/Float64MultiArray.h"
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm.h>

#include "repub_lcm/lcm/eurecar/pos_ros_t.hpp"

class ROSToLCM_POS_T
{
    public:
        ROSToLCM_POS_T(ros::NodeHandle& n, lcm::LCM* lh);
        ~ROSToLCM_POS_T();
        void ros_rmc_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        
    private:        
        lcm::LCM* lcm_handle;
        ros::NodeHandle nh;
        ros::Subscriber ros_rmc_sub;
};

ROSToLCM_POS_T::ROSToLCM_POS_T(ros::NodeHandle& n, lcm::LCM* lh) 
{
    nh = n;
    lcm_handle = lh;
   
    // Subscribers
    ros_rmc_sub = nh.subscribe<std_msgs::Float64MultiArray>("POS_T", 10, &ROSToLCM_POS_T::ros_rmc_callback, this);
};

ROSToLCM_POS_T::~ROSToLCM_POS_T() 
{    
    ROS_INFO("NAVIGATION ROSToLCM_POS_T destructor.");
}

void ROSToLCM_POS_T::ros_rmc_callback(const std_msgs::Float64MultiArray::ConstPtr& ros_msg)
{
    eurecar::pos_ros_t* pos_msg = new eurecar::pos_ros_t();
    pos_msg->utime = 0;
    pos_msg-> x_est = ros_msg->data.at(0);
    pos_msg-> y_est = ros_msg->data.at(1);
    pos_msg-> h_est = ros_msg->data.at(2);
    pos_msg-> x_raw = ros_msg->data.at(3);
    pos_msg-> y_raw = ros_msg->data.at(4);
    pos_msg-> h_raw = ros_msg->data.at(5);
    pos_msg-> v = ros_msg->data.at(10);
    pos_msg-> get_head = ros_msg->data.at(7);
    pos_msg-> get_pose = ros_msg->data.at(6);
    pos_msg-> pos_init = ros_msg->data.at(8);
    pos_msg-> gps_init = ros_msg->data.at(9);
    // std::cout<<"test"<<std::endl;
    lcm_handle->publish("POS_ros_T", pos_msg);
};     