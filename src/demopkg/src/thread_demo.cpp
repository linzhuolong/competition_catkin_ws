/*
#include <ros/ros.h>
#include <iostream>
#include <boost/thread.hpp>
#include <sensor_msgs/LaserScan.h>
#include <ros/callback_queue.h>

using namespace std;
using namespace ros;

static vector<long> data;
static ros::CallbackQueue laser_queue;

void threadcallback(const sensor_msgs::LaserScanConstPtr& laser)
{
    ROS_INFO("receive laser data in laser callback thread");
    data.clear();
	for (size_t i = 0; i < laser->ranges.size(); i++)
	{
		data.push_back(laser->ranges[i]*1000);	
	}
}

void laser_thread()
{
    NodeHandle nh_laser;
    while(nh_laser.ok())
    {
        laser_queue.callAvailable(WallDuration(0.01));
    }
}

int main(int argc, char**argv)
{
    init(argc, argv, "thread_demo");
    NodeHandle nh_main;
    
    ros::SubscribeOptions laser_ops = ros::SubscribeOptions::create<sensor_msgs::LaserScan>("/scan", 1000, threadcallback, ros::VoidPtr(), &laser_queue);
    ros::Subscriber laser_sub = nh_main.subscribe(laser_ops);
    boost::thread laser_thread(threadcallback);
    laser_thread.join();
    ros::Rate r(1);
    while (nh_main.ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;

}*/


#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
#define datasize 1081
  //tutorial demonstrates the use of custom separate callback queues that can be processed
  //independently, whether in different threads or just at different times. 
  //演示了自定义独立回调队列的使用，
  //这些回调队列可以在不同的线程中独立处理，也可以在不同的时间进行处理。
 
static std::vector<long> data;
static cv::Mat image_original;
static std::vector<pair<double, long>> object_angle_distance;
static pair<double, long> tempPair;

void laser_callback(const sensor_msgs::LaserScanConstPtr &laser)
{
    //ROS_INFO_STREAM("I heard: [ " << msg->data << "] in thread ["<< boost::this_thread::get_id() << "]");
    //ROS_INFO_STREAM("receive laser data in thread");
    data.clear();
    for (size_t i = 0; i < laser->ranges.size(); i++)
	{
		data.push_back(laser->ranges[i]*1000);	
	}
}

bool find_nearest_object_by_laser_data(std::vector<long> &data, std::vector<pair<double, long>> &object_angle_distance)
{ //第一个有
	std::cout << "得到的激光的大小：" << data.size() << std::endl;
	if (!data.empty())
	{
		for (size_t i = 0; i < data.size(); i++)
		{
			if (data[i] > 3000)
				data[i] = 3000;
		}
		int start = 0;
		int end = 0;
		for (size_t i = 260; i < 821 /*data.size()-1*/; i++)
		{
			long dataSub = data[i + 1] - data[i];
			//if(dataSub<-400  && data[i+2]<2000)
			if (dataSub < -200 && data[i + 2] < 1200)
			//if(dataSub<-300  && data[i+2]<1200)
			{
				start = i + 1;
				//for(int count=1;count<=5;count++)
				for (int count = 1; count <= 10; count++)
				{
					if ((data[i + 1 + count] - data[i - count]) < -200 && data[i + 1 + count] < 1200)
					{
						;
					}
					else
					{
						start = 0;
						break;
					}
				}
				cout << "start:" << start << endl;
			}
			//if(dataSub>400 && start && data[i-1]<2000)
			if (dataSub > 200 && start && data[i - 1] < 1200)
			//if(dataSub>300 && start && data[i-1]<1200)
			{
				end = i;
				//for(int count=1;count<=5;count++)
				for (int count = 1; count <= 10; count++)
				{
					if ((data[i + 1 + count] - data[i - count]) > 200 && data[i - count] < 1200)
					{
						;
					}
					else
					{
						end = 0;
						break;
					}
				}
				cout << "end:" << end << endl;
			}
			if (start && end)
			{
				cout << "end-start" << end - start << endl;
				//if(end-start>=5 && end-start<=350)
				if (end - start > 1 && end - start <= 350)

				{
					long centerDist = data[(int)((start + end) / 2)]; //计算物体中心点的坐标
					/////自己加
					if (centerDist >= 150)
					{
						/////
						
						tempPair.first = 0.25 * ((int)(start + end) / 2);
						tempPair.second = centerDist;
						object_angle_distance.push_back(tempPair);
						cout<<tempPair.first<<" "<<tempPair.second<<endl;
					}
					/*auto minDist = min_element(data.begin()+start,data.begin()+end); 
						long centerDist = data[(int)((start+end)/2)]; 
						if(centerDist-*minDist<=100)
						{
							pair<double,long>temPara;
							temPara.first=0.25*((int)((start+end)/2));
							temPara.second=centerDist;
							object_angle_distance.push_back(temPara);
						}*/
				}
				if (end - start > 1 && end - start <= 550)
				//if(end-start>=5 && end-start<=550)
				{
					long centerDist = data[(int)((start + end) / 2)]; //计算物体中心点的坐标
					/////自己加
					if (centerDist < 150)
					{
						/////
						
						tempPair.first = 0.25 * ((int)(start + end) / 2);
						tempPair.second = centerDist;
						object_angle_distance.push_back(tempPair);
						cout<<tempPair.first<<" "<<tempPair.second<<endl;
					}
				}
				start = 0;
				end = 0;
			}
		}
	}
	if (object_angle_distance.size() == 0)
	{
		cout << "fail to find_nearest_object_by_laser_data" << endl;
		return false;
	}
	else
	{
		cout << "success to find_nearest_object_by_laser_data" << endl;
		return true;
	}
}

void image_callback(const sensor_msgs::ImageConstPtr& img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
	{
		cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge error: s%", e.what());
		return;
	}
    image_original = cv_ptr->image;
}

 
 // The custom queue used for one of the subscription callbacks
 
ros::CallbackQueue laser_queue; //第一步:用于订阅回调的自定义队列
ros::CallbackQueue image_queue;


void ls_thread()
{
    ROS_INFO_STREAM("laser thread id=" << boost::this_thread::get_id());
    
    ros::NodeHandle nh_0;
    //ros::Publisher laser_pub = nh_0.advertise<demopkg::target>("/object_param", 1000);
    while (nh_0.ok())
    {
        //第四步: 执行自定义队列中的回调函数.
        // CallbackQueue类有两种调用内部回调的方法:callAvailable()和callOne()。
        // callAvailable()将获取队列中当前的所有内容并调用它们。callOne()将简单地调用队列上最古老的回调。
        laser_queue.callAvailable(ros::WallDuration(0.01));
    }
}

void img_thread()
{   
    ROS_INFO_STREAM("image thread id=" << boost::this_thread::get_id());
    ros::NodeHandle nh_1;
    
    while(nh_1.ok())
    {
        image_queue.callAvailable(ros::WallDuration(0.01));
        
        
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener_with_custom_callback_processing");
    ros::NodeHandle nh_main;
    data.resize(datasize);
    ros::Rate r(100);
    
    //第二步: 声明订阅或者发布选项, 然后和订阅器/发布器绑定在一起
    ros::SubscribeOptions laser_ops = ros::SubscribeOptions::create<sensor_msgs::LaserScan>("/scan", 1000, laser_callback, ros::VoidPtr(), &laser_queue);
    ros::SubscribeOptions image_ops = ros::SubscribeOptions::create<sensor_msgs::Image>("/usb_cam/image_raw", 1, image_callback, ros::VoidPtr(), &image_queue);
    ros::Subscriber laser_sub = nh_main.subscribe(laser_ops);
    ros::Subscriber image_sub = nh_main.subscribe(image_ops);

    


    //第三步: 声明线程.
    boost::thread image_thread(img_thread);
    boost::thread laser_thread(ls_thread);
    
    ROS_INFO_STREAM("Main thread id=" << boost::this_thread::get_id());
    while(ros::ok())
    {   
        ros::spinOnce();
        find_nearest_object_by_laser_data(data, object_angle_distance);
        //ROS_INFO_STREAM("receive data : "<<data[540]);
        //imshow("image", image_original); 
    }
        
    



    laser_thread.join();
    image_thread.join();

    return 0;
}