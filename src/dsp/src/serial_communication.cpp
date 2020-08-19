#include "dsp/serial_communication.h"
#include "const_msg/globe.h"

//其他包头文件　https://blog.csdn.net/qq_16775293/article/details/80597763
//消息文件　https://blog.csdn.net/u013453604/article/details/72903398
using namespace std;
using namespace ros;
//同时收发　http://zhaoxuhui.top/blog/2019/10/20/ros-note-7.html

serial::Serial ser;//定义一个串口对象

serial_communication::serial_communication()
{
    control_sub = nh.subscribe("/pc_to_dsp", 1000, &serial_communication::controlCallback, this);
    status_pub = nh.advertise<const_msg::dsp_to_pc>("/dsp_to_pc", 1000);
}

serial_communication::~serial_communication()
{
}

void serial_communication::controlCallback(const const_msg::pc_to_dspConstPtr &control_msg)
{
    //TODO: 失败帧count
    if (control_msg->flag_start_stop)
    {
        m_DSPControl.flag_start_stop = control_msg->flag_start_stop;
        m_DSPControl.SendData_state = control_msg->SendData_State;
        m_DSPControl.V1 = control_msg->V1;
        m_DSPControl.V2 = control_msg->V2;
        m_DSPControl.V3 = control_msg->V3;
        m_DSPControl.V4 = control_msg->V4;
        DSPControl_write(ser, m_DSPControl);
    }
    else
    {
        OnDSPStopCommunication(ser);
    }
}

void serial_communication::DSPStatus_pub(DSPStatus &pDSPStatus)
{
    static const_msg::dsp_to_pc DSPStatus_msg;

    DSPStatus_msg.dw = pDSPStatus.DW;
    DSPStatus_msg.Vx = pDSPStatus.Vx;
    DSPStatus_msg.Vy = pDSPStatus.Vy;
    DSPStatus_msg.XDist = pDSPStatus.XDist;
    DSPStatus_msg.YDist = pDSPStatus.YDist;
    DSPStatus_msg.fAngle = pDSPStatus.fAngle;
    DSPStatus_msg.RecData_State = pDSPStatus.RecData_state;

    status_pub.publish(DSPStatus_msg);
}

bool serial_communication::OnDSPStartCommunication(serial::Serial &serialport)
{
    unsigned char InitStart[12] = {0xfa, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, '0', 0xfb};
    unsigned char InitRecv[16] = {0xfa, 0xfb, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b};
    unsigned char r[16];
    int count = 0;
    for (int i = 0; i < 3; i++)
        serialport.write(InitStart, 16);
    serialport.read(r, 16);
    for (size_t i = 0; i < 16; i++)
    {
        if (r[i] == InitRecv[i])
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

bool serial_communication::OnDSPStopCommunication(serial::Serial &serialport)
{
    unsigned char InitStop[wbuffersize] = {0xfa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, '0', 0xfb};
    int len;
    for (int i = 0; i < 3; i++)
        len = serialport.write(InitStop, wbuffersize);
    if (len != wbuffersize)
    {
        ROS_ERROR_STREAM("DSP stop error");
        return false;
    }
    else
    {
        return true;
    }
}

bool serial_communication::serial_port_init(serial::Serial &serialport)
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
            while (!OnDSPStartCommunication(serialport) && ros::ok()){            }
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

unsigned char *serial_communication::DSPStatus_read(serial::Serial &serialport)
{
    //定义为static类型，当出现数据包不完整的时候，可以发送上一帧的数据
    //todo 增加失败帧的count
    static unsigned char rbuffer[rbuffersize] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    unsigned char tempbuffer[rbuffersize];
    int s = serialport.read(tempbuffer, rbuffersize);

    if (0xfa != tempbuffer[0] && 0xfb != tempbuffer[1])
    {
        ROS_ERROR_STREAM("data head wrong");
    }
    else
    {
        ROS_INFO_STREAM("data head right");
        unsigned short chksum = checksum(tempbuffer + 2);
        unsigned short temp = (unsigned short)(tempbuffer[14] * 0x100 + tempbuffer[15]);
        if (chksum == temp)
        {
            ROS_INFO_STREAM("Receive DSPdata right");
            for (size_t i = 0; i < rbuffersize; i++)
            {
                rbuffer[i] == tempbuffer[i];
            }
        }
        else
        {
            ROS_ERROR_STREAM("Receive DSPdata checksum error");
        }
        return rbuffer;
    }
}

unsigned short serial_communication::checksum(unsigned char *ptr)
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

DSPStatus &serial_communication::translate(unsigned char *databuff, DSPStatus &pDSPStatus)
{
    static double old_angle = 0;

    pDSPStatus.XDist = (short)(databuff[3] * 0x100 + databuff[4]);
    pDSPStatus.YDist = (short)(databuff[5] * 0x100 + databuff[6]);
    pDSPStatus.fAngle = (short)(databuff[7] * 0x100 + databuff[8]) / 32768 * 180;
    pDSPStatus.fAngle = (float)mth_ChangeAngle(pDSPStatus.fAngle);
    pDSPStatus.DW = (old_angle - pDSPStatus.fAngle) / 0.045; //45ms通信一次
    old_angle = pDSPStatus.fAngle;
    pDSPStatus.Vx = (short)(databuff[9] << 8 | databuff[10]);
    pDSPStatus.Vy = (short)(databuff[11] << 8 | databuff[12]);
    pDSPStatus.RecData_state = (short)(databuff[13]);
    return pDSPStatus;
}

bool serial_communication::DSPControl_write(serial::Serial &serialport, DSPControl &pDSPControl)
{
    unsigned char databuff[wbuffersize];
    memset(databuff, '0', sizeof(databuff));
    databuff[0] = 0xfa;
    databuff[1] = 0x10;
    databuff[2] = (pDSPControl.V1 >> 8) & 0xff;
    databuff[3] = pDSPControl.V1 & 0xff;
    databuff[4] = (pDSPControl.V2 >> 8) & 0xff;
    databuff[5] = pDSPControl.V2 & 0xff;
    databuff[6] = (pDSPControl.V3 >> 8) & 0xff;
    databuff[7] = pDSPControl.V3 & 0xff;
    databuff[8] = (pDSPControl.V4) & 0xff;
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
    int len = serialport.write(databuff, sizeof(databuff));
    if (len != sizeof(databuff))
    {
        ROS_ERROR_STREAM("Write control data to serialport error");
        return false;
    }
    else
    {
        return true;
    }
}

double serial_communication::mth_ChangeAngle(double theta)
{
    while(theta>180)
        theta-=360;
    while(theta<=180)
        theta+=180;
    return theta;
}