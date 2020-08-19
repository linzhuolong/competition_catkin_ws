//serial_port.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <time.h>
#include <const_msg/globe.h>

bool OnDSPStartCommunication(serial::Serial &serialport)
{
    unsigned char InitStart[12] = {0xfa, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, '0', 0xfb};
    unsigned char InitRecv[16] = {0xfa, 0xfb, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b};
    unsigned char r[16];
    int count = 0;
    for (int i = 0; i < 3; i++)
        serialport.write(InitStart, 16);
    //sleep(5);
    serialport.read(r, 16);
    for (size_t i = 0; i < 16; i++)
    {
        if(r[i] == InitRecv[i])
            count++;
    }
    if (count != 16)
    {
        ROS_ERROR_STREAM("DSP start error");
        return false;
    }
    else
    {
        return true;
    }
}

bool serial_port_init(serial::Serial &serialport)
{
    static int serial_flag = 0;
    while (!serial_flag && ros::ok())
    {
        try
        {
            serialport.setPort("/dev/ttyUSB0");
            serialport.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(45);
            serialport.setTimeout(to);
            serialport.open();
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("unable to open port");
            serial_flag = 0;
        }
        if (serialport.isOpen())
        {
            serial_flag = 1;
            while (!OnDSPStartCommunication(serialport) && ros::ok())
            {
                
            }
            ROS_INFO_STREAM("serial port initialized, DSP start right");
        }
        else
        {
            ROS_ERROR_STREAM("unable to initialize port");
            serial_flag = 0;
        }
    }
    ros::Time::init();
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;

    //创建一个serial类
    serial::Serial ser;
    unsigned char rbuffer[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    clock_t start, end;
    ros::Rate loop_rate(10);   

    while (serial_port_init(ser) && ros::ok())
    {
        //serial_port_init(ser);
        //获取缓冲区内的字节数
        start = clock();
        int s = ser.read(rbuffer, 16);
        end = clock();
        double delay = (double)(end-start)/CLOCKS_PER_SEC;
        for (size_t i = 0; i < 16; i++)
        {
            std::cout << std::hex << " " << (unsigned int)rbuffer[i];
        }

        std::cout <<"delay: "<<delay<< std::endl;

        loop_rate.sleep();
    }

    //关闭串口
    ser.close();

    return 0;
}