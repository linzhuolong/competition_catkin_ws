#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Int32.h>

using namespace std;

int main(int argc, char**argv)
{
    ros::init(argc, argv, "pub_order2");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("target_type_2", 1000);
    ros::Rate r(10);
    std_msgs::Int32 msg;
    msg.data = 9;
    while(ros::ok())
    {
        pub.publish(msg);
        r.sleep();
    }
    return 0;
}