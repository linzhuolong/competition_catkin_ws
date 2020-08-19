#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include <serial/serial.h>
#include "dsp/serial_communication.h"


using namespace std;
using namespace ros;

int main(int argc, char**argv)
{
    init(argc, argv, "serial_node");
    serial_communication ser_process;//定义一个串口通信类
    unsigned char* pbuffer;
    while(ser_process.serial_port_init(ser) && ok())
    {
        pbuffer = ser_process.DSPStatus_read(ser);
        ser_process.m_DSPStatus = ser_process.translate(pbuffer, ser_process.m_DSPStatus);
        ser_process.DSPStatus_pub(ser_process.m_DSPStatus);
        spinOnce();
    }
    return 0;
}