#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm.h>

#include "sensor_msgs/Imu.h"
#include "repub_lcm/lcm/eurecar/imu_ros_t.hpp"

class ROSToLCM_IMU_T
{
    public:
        ROSToLCM_IMU_T(ros::NodeHandle& n, lcm::LCM* lh);
        ~ROSToLCM_IMU_T();
        void ros_imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
        
    private:        
        lcm::LCM* lcm_handle;
        ros::NodeHandle nh;
        ros::Subscriber ros_imu_sub;
};

ROSToLCM_IMU_T::ROSToLCM_IMU_T(ros::NodeHandle& n, lcm::LCM* lh) 
{
    nh = n;
    lcm_handle = lh;
   
    // Subscribers
    ros_imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 10, &ROSToLCM_IMU_T::ros_imu_callback, this);
};

ROSToLCM_IMU_T::~ROSToLCM_IMU_T() 
{    
    ROS_INFO("NAVIGATION ROSToLCM_IMU_T destructor.");
}

void ROSToLCM_IMU_T::ros_imu_callback(const sensor_msgs::Imu::ConstPtr& ros_msg)
{
    eurecar::imu_ros_t* imu_msg = new eurecar::imu_ros_t();
    imu_msg->utime = 0;
    imu_msg-> x_acc = ros_msg->linear_acceleration.x;
    imu_msg-> y_acc = ros_msg->linear_acceleration.y;
    imu_msg-> z_acc = ros_msg->linear_acceleration.z;
    imu_msg-> rollrate = ros_msg->angular_velocity.x;
    imu_msg-> pitchrate = ros_msg->angular_velocity.y;
    imu_msg-> yawrate = ros_msg->angular_velocity.z;
    // std::cout<<"test"<<std::endl;

    lcm_handle->publish("IMU_ros_T", imu_msg);
};     