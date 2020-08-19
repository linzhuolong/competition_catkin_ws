#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>

using namespace std;
using namespace ros;

void callback(const sensor_msgs::LaserScanConstPtr& laser)
{
    ROS_INFO_STREAM("laser data: "<<laser->ranges[540]*1000);
}

int main(int argc, char**argv)
{
    init(argc, argv, "laser");
    NodeHandle nh;
    Subscriber sub = nh.subscribe("/scan", 1000, &callback);
    spin();
    return 0;
}