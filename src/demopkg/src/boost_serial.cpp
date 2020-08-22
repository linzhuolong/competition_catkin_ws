#include <ros/ros.h>
#include <stdio.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace boost::asio;
using namespace std;

void sendThreadRun(boost::asio::serial_port *serialPort)
{
    unsigned char InitStart[12] = {0xfa, 0x10, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, '0', 0xfb};

    boost::system::error_code errorCode;
    
    //serialPort->write_some(boost::asio::buffer(InitStart,sizeof(InitStart)));
    
}

void receiveThreadRun(boost::asio::serial_port *serialPort)
{
    unsigned char receiveBuffer[16];
    ros::NodeHandle nh_;
    ros::Rate r(200);

    while(ros::ok())
    {

        //serialPort->read_some(boost::asio::buffer(receiveBuffer,sizeof(receiveBuffer)));
        for (size_t i = 0; i < 16; i++)
        {
            std::cout << std::hex << " " << (unsigned int)receiveBuffer[i];
        }
        std::cout<<std::endl;
        r.sleep();
    }
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "boost_serial");
    ros::NodeHandle nh;
    io_service io_ser;
    serial_port ser(io_ser, "/dev/ttyUSB0");
    ser.set_option(serial_port::baud_rate(115200));
    ser.set_option(serial_port::flow_control(serial_port::flow_control::none));
    ser.set_option(serial_port::parity(serial_port::parity::none));
    ser.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    ser.set_option(serial_port::character_size(8));
    unsigned char InitStart[12] = {0xfa, 0x10, 0x01, 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, '0', 0xfb};
    write(ser, buffer(InitStart));
    ros::Rate r(22);
    
    while (ros::ok())
    {
        unsigned char rbuffer[16];
        boost::asio::read(ser, buffer(rbuffer));//TODO:读操作出现问题
        for (size_t i = 0; i < 16; i++)
        {
            std::cout << std::hex << " " << (unsigned int)rbuffer[i];
        }
        std::cout<<std::endl;
        //ROS_INFO_STREAM("rbuffer is: "<<std::hex<<(unsigned char*)(unsigned int*)rbuffer);
        r.sleep();
    }
    
    //std::thread sendThread(sendThreadRun,&ser);

    //std::thread receiveThread(receiveThreadRun,&ser);
    
    //sendThread.join();
    //receiveThread.join();
    return 0;
}