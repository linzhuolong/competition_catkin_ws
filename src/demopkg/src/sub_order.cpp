#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Int32.h>

using namespace std;

void callback_1(const std_msgs::Int32ConstPtr& msg)
{
    ROS_INFO_STREAM("30hz: "<<msg->data);
}

void callback_2(const std_msgs::Int32ConstPtr& msg)
{
    ROS_INFO_STREAM("10hz: "<<msg->data);
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "sub_order");
    ros::NodeHandle nh;
    ros::Subscriber sub_1 = nh.subscribe("/target_type", 1000, &callback_1);
    ros::Subscriber sub_2 = nh.subscribe("/target_type_2", 1000, &callback_2);
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}