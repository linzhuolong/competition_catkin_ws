#include <ros/ros.h>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace boost::asio;

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
    unsigned char InitStart[12] = {0xfa, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, '0', 0xfb};
    write(ser, buffer(InitStart));
    ros::Rate r(22);
    while (ros::ok())
    {
        unsigned char rbuffer[16];
        read(ser, buffer(rbuffer));
        for (size_t i = 0; i < 16; i++)
        {
            std::cout << std::hex << " " << (unsigned int)rbuffer[i];
        }
        std::cout<<std::endl;
        //ROS_INFO_STREAM("rbuffer is: "<<std::hex<<(unsigned char*)(unsigned int*)rbuffer);
        r.sleep();
    }
    return 0;
}