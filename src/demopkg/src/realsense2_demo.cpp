#include <iostream>
#include <librealsense2/rs.hpp>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace ros;
using namespace cv;

int main(int argc, char** argv)
{
    init(argc, argv, "realsense2_demo");
    rs2::pipeline pipe;
    rs2::pipeline_profile profile;
    rs2::config pipe_config;
    rs2::frameset frameset;
    rs2::frame color_frame, depth_frame, pose_frame;
    rs2::colorizer cz;
    rs2::rates_printer printer;

    //pipe_config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    //pipe_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    //pipe_config.enable_stream(RS2_STREAM_GYRO,  640, 480, 30);
    //pipe_config.enable_stream(RS2_STREAM_ACCEL, 640, 480, 30);

    profile = pipe.start();
    //profile = pipe.start(pipe_config);
    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    //auto gyro_stream  = profile.get_stream(RS2_STREAM_GYRO).as<rs2::video_stream_profile>();
    //uto accel_stream = profile.get_stream(RS2_STREAM_ACCEL).as<rs2::video_stream_profile>();

    auto extrin_depth_2_color = depth_stream.get_extrinsics_to(color_stream);

    while(ok())
    {
        frameset = pipe.wait_for_frames().apply_filter(printer);
        depth_frame = frameset.get_depth_frame();
        color_frame = frameset.get_color_frame();
        pose_frame  = frameset.get_pose_frame();
        
        const int depth_w = depth_frame.as<rs2::video_frame>().get_width();
        const int depth_h = depth_frame.as<rs2::video_frame>().get_height();
        const int color_w = color_frame.as<rs2::video_frame>().get_width();
        const int color_h = color_frame.as<rs2::video_frame>().get_height();

        Mat color_image(Size(color_w, color_h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        imshow("color image", color_image);
        waitKey(1);
    }

    return 0;
}
