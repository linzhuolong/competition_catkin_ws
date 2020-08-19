#ifndef _VISION_H_
#define _VISION_H_

#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>
#include <opencv2/opencv.hpp>
#include <const_msg/object_param.h>

using namespace std;
using namespace ros;
using namespace cv;

#define angleDist 10
#define threshValue 4000

typedef enum
{
    unkownObject,             //0
    calibration,              //1标定柱
    reddish_brown_basketball, //2红棕色
    basketball,               //3
    blue_gray_basketball,     //4蓝灰色
    calibration_plus,         //5标定柱+
    orange_red_volleyball,    //6橙红色
    red_blue_volleyball,      //7红蓝色
    volleyball,               //8
    laser_ball,               //9
    three_round_ball,         //10
    round_1,                  //11
} objectType;

typedef struct
{
    objectType whatObject;
    double objectAngle;
    long distance;
} objectParameter;

extern objectType object_type_target;               //目标类型
extern Mat image_original0;                         //上面的摄像头数据
extern Mat image_original1;                         //下面的摄像头数据
extern vector<long> laser_data_range;               //激光数据
extern vector<long> laser_data_intensity;           //激光反射强度
extern objectParameter object_detect_result;        //目标角度距离信息


class Vision
{
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    //image_transport::Publisher image_pub;
    image_transport::Subscriber image_sub;
    ros::Publisher detect_data_pub;
    ros::Subscriber laser_data_sub;
    ros::Subscriber object_target_sub;
    //时间戳结构体
    /*typedef struct
    {
        uint32_t seq;
        Time stamp;
        string fram_id;
    } Header;
    //原始图像消息结构体
    /*typedef struct original_image_data
    {
        Mat image_original;
        Header image_header;
    } Original_Image_Data;
    //原始雷达数据结构体
    typedef struct original_laser_data
    {
        vector<long> laser_data_range;
        vector<long> laser_data_intensity;
        Header laser_header;
    } Original_Laser_Data;

    Original_Image_Data image_data_original;//通过usb_cam包读取的图像数据
    Original_Laser_Data laser_data_original;*/
public:
    Vision();
    ~Vision(){};
    /**************************************回调函数->数据接受**************************************************/
    void img_callback(const sensor_msgs::ImageConstPtr &img);
    void laser_callback(const sensor_msgs::LaserScanConstPtr &laser);
    void object_target_callback(const std_msgs::Int32ConstPtr& nums);
    /****************************************数据发送函数，方便发送数据******************************************/
    void send_final_result(pair<double, long> &final_object_param);
  
    /***************************************色块检测函数、距离检测函数******************************************/
    bool find_calibration_by_LC(Mat &imgOriginal, Point2f &objectCenter);
    bool find_calibration_object_by_laser_data(std::vector<long> &data, std::vector<pair<double, long>> &objectAngleDistance);
    bool find_Reddish_brown_basketball(Mat &imgOriginal, Point2f &objectCenter);
    bool find_blue_gray_basketball(Mat &imgOriginal, Point2f &objectCenter);
    bool find_blue_gray_basketball_plus(Mat &imgOriginal, Point2f &objectCenter);
    bool find_nearest_object_by_laser_data_plus(std::vector<long> &data, std::vector<pair<double, long>> &objectAngleDistance);

    bool find_nearest_object_by_laser_data(std::vector<long> &data, std::vector<pair<double, long>> &objectAngleDistance);
    bool find_nearest_object_by_laser_data_laser_ball(std::vector<long> &data, std::vector<pair<double, long>> &objectAngleDistance);

    bool find_basketball(Mat &imgOriginal, Point2f &centerObject);
    bool find_final_object(vector<pair<double, long>> &objectAngleDistance, Point2f &objectCenter, pair<double, long> &finalobjectAngleDistance);
    bool find_final_object_plus(vector<pair<double, long>> &objectAngleDistance, Point2f &objectCenter, pair<double, long> &finalobjectAngleDistance);

    bool find_orange_red_volleyball(Mat &imgOriginal, Point2f &objectCenter);
    bool find_orange_blue_volleyball(Mat &imgOriginal, Point2f &objectCenter);
    bool find_volleyball(Mat &imgOriginal, Point2f &objectCener);
};

#endif