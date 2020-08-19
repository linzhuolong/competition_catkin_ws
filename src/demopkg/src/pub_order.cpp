#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
    init(argc, argv, "pub_order");
    NodeHandle nh;
    Publisher pub = nh.advertise<std_msgs::Int32>("/target_type", 1000);
    Rate r(30);
    std_msgs::Int32 order_msgs;

    while (ok())
    {
        /*for (size_t i = 1; i < 12; i++)
        {
            order_msgs.data = i;
            pub.publish(order_msgs);
            r.sleep();
        }*/
        order_msgs.data = 1;
        pub.publish(order_msgs);
        r.sleep();
    }
    return 0;
}