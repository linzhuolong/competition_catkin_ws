#include <ros/ros.h>
#include <iostream>
#include <serial/serial.h>
#include <const_msg/pc_to_dsp.h>

using namespace std;
using namespace ros;

int main(int argc, char**argv)
{
    init(argc, argv, "pub_control");
    NodeHandle nh;
    Publisher pub = nh.advertise<const_msg::pc_to_dsp>("/pc_to_dsp", 1000);
    const_msg::pc_to_dsp ctrl_msg;
    Rate r(30);
    while (ok())
    {
        ctrl_msg.flag_start_stop = 1;
        ctrl_msg.SendData_State = 0;
        for (size_t i = 300; i < 1000; i++)
        {
            ctrl_msg.V1 = ctrl_msg.V2 = ctrl_msg.V3 = ctrl_msg.V4 = i;
            pub.publish(ctrl_msg);
            r.sleep();
        }
        for (size_t i = 1000; i > 300; i--)
        {
            ctrl_msg.V1 = ctrl_msg.V2 = ctrl_msg.V3 = ctrl_msg.V4 = i;
            pub.publish(ctrl_msg);
            r.sleep();
        }
    }
}