#include <ros/ros.h>
#include <static_laser_tools/static_laser_tools.hpp>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"laser_scan_cp");
    ros::NodeHandle nh_priv("~");
    ClosestPoint cp(nh_priv);
    ros::spin();
    return 0;
}