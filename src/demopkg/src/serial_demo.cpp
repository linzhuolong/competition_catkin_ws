//serial_port.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <time.h>
#include <const_msg/globe.h>

#define wbuffersize 12

typedef struct tagDSPSend
{
    unsigned char flag_start_stop;
    short V1, V2, V3, V4;
    unsigned char SendData_state;
} DSPControl; //发送数据结构

enum ArmState
{
    donothing = 0,
    arm_up = 1,
    push_ball_high = 2,
    push_ball = 3,
    push_ball_low = 4,
    pick_ball = 5,
    turning_off = 6,
    turning_on = 7,
    armup_gameover = 8
}; //抬杆　弹球等控制指令

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
    if (r[0] != 0xfa && r[1] != 0xfb)
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

unsigned short checksum(unsigned char *ptr)
{
    int n;
    unsigned short c = 0;
    n = *(ptr++);
    c += n;
    while (n--)
    {
        c += *(ptr++);
        c = c & 0xffff;
    }
    return c;
}

unsigned char* DSPControl_write(DSPControl &pDSPControl)
{
    static unsigned char databuff[wbuffersize];
    memset(databuff, '0', sizeof(databuff));
    databuff[0] = 0xfa;
    databuff[1] = 0x10;
    databuff[2] = (pDSPControl.V1 >> 8) & 0xff;
    databuff[3] = pDSPControl.V1 & 0xff;
    databuff[4] = (pDSPControl.V2 >> 8) & 0xff;
    databuff[5] = pDSPControl.V2 & 0xff;
    databuff[6] = (pDSPControl.V3 >> 8) & 0xff;
    databuff[7] = pDSPControl.V3 & 0xff;
    databuff[8] = (pDSPControl.V4 >> 8) & 0xff;
    databuff[9] = pDSPControl.V4 & 0xff;
    switch (pDSPControl.SendData_state)
    {
    case donothing:
        databuff[10] = '0';
        break;
    case arm_up:
        databuff[10] = 'a';
        break;
    case push_ball_high:
        databuff[10] = 'h';
        break;
    case push_ball:
        databuff[10] = 'c';
        break;
    case push_ball_low:
        databuff[10] = 'l';
        break;
    case pick_ball:
        databuff[10] = 'e';
        break;
    case turning_off:
        databuff[10] = 'k';
        break;
    case turning_on:
        databuff[10] = 'j';
        break;
    case armup_gameover:
        databuff[10] = 'g';
        break;

    default:
        databuff[10] = '0';
        break;
    }
    databuff[11] = 0xfb;
    //ROS_INFO_STREAM("databuff[2] is: "<<std::hex<<(unsigned int)databuff[2]<<"databuff[3] is: "<<(unsigned int)databuff[3]);
    return databuff;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;

    //创建一个serial类
    serial::Serial ser;
    unsigned char rbuffer[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    unsigned char pbuffer[12] = {0xfa, 0x10, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0x00, 0x64, 0x00, 0xfb};
    unsigned char *wbuffer;
    clock_t start, end;
    ros::Rate loop_rate(30);
    int count = 0;
    DSPControl m_ctrl;
    m_ctrl.flag_start_stop = 1;
    m_ctrl.SendData_state = 0;   

    /*while (serial_port_init(ser) && ros::ok())
    {
        m_ctrl.V1 = m_ctrl.V2 = m_ctrl.V3 = m_ctrl.V4 = 100;
        wbuffer = DSPControl_write(m_ctrl);
        for (size_t i = 0; i < wbuffersize; i++)
        {
            rbuffer[i] = wbuffer[i];
        }    
        std::cout<<"rbuffer : ";
        for (size_t i = 0; i < wbuffersize; i++)
        {
            std::cout << std::hex << " " << (unsigned int)rbuffer[i];
        }
        std::cout<<std::endl;   
        ser.write(rbuffer, wbuffersize);
        loop_rate.sleep();
    }*/
    

    while (serial_port_init(ser) && ros::ok())
    {
        //serial_port_init(ser);
        //获取缓冲区内的字节数
        //ser.flush();
        start = clock();
        int s = ser.read(rbuffer, 16);
        ROS_INFO_STREAM("!!!!!!!!!!!!!!!!!!!!XDist is: "<<(unsigned int)rbuffer[3]<<" "<<(unsigned int)rbuffer[4]);
        end = clock();
        double delay = (double)(end-start)/CLOCKS_PER_SEC;
        if(rbuffer[0]!=0xfa && rbuffer[1] != 0xfb)
        {
            count++;
        }
        for (size_t i = 0; i < 16; i++)
        {
            //std::cout << std::hex << " " << (unsigned int)rbuffer[i];
        }

        //std::cout <<" "<<"delay: "<<delay<<" "<<"count is: "<<count<< std::endl;
        /*
        for (size_t i = 400; i < 1500; i++)
        {
            m_ctrl.V1 = m_ctrl.V2 = m_ctrl.V3 = m_ctrl.V4 = i;
            wbuffer = DSPControl_write(m_ctrl);
            ser.write(wbuffer, wbuffersize);
            loop_rate.sleep();
        }
        for (size_t i = 1500; i > 400; i--)
        {
            m_ctrl.V1 = m_ctrl.V2 = m_ctrl.V3 = m_ctrl.V4 = i;
            wbuffer = DSPControl_write(m_ctrl);
            ser.write(wbuffer, wbuffersize);
            loop_rate.sleep();
        }
        */

        loop_rate.sleep();
    }

    //关闭串口
    ser.close();

    return 0;
}