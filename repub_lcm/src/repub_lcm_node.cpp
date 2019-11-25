///////////////////////////////////////////////////////////////////////
// This source is re-publisher from ROS topic to LCM 
// 
///////////////////////////////////////////////////////////////////////
//
// Creation: Lee Daegyu//
///////////////////////////////////////////////////////////////////////
#include "rostolcm_gga_t.hpp"
#include "rostolcm_rmc_t.hpp"
#include "rostolcm_pos_t.hpp"
#include "rostolcm_imu_t.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "repub_lcm_node");
    ros::NodeHandle nh;   
    lcm::LCM* lcm_handle = new lcm::LCM("udpm://239.255.76.67:7667?ttl=1");
    if(!lcm_handle->good())
        return 1;
        
    ROSToLCM_IMU_T imu(nh, lcm_handle);
    ROSToLCM_GGA_T gga(nh, lcm_handle);
    ROSToLCM_RMC_T rmc(nh, lcm_handle);
    ROSToLCM_POS_T pos(nh, lcm_handle);
    ros::spin();

    delete lcm_handle;
    return 0;
}

