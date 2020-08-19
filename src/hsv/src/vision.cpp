#include <ros/ros.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include "hsv/vision.h"

#define dataSize 1081　 //激光硬件为正负135度扫描范围

using namespace std;
using namespace cv;


objectType object_type_target;
//Mat image_original0;				  //上面的摄像头数据
//Mat image_original1;				  //下面的摄像头数据
vector<long> laser_data_range;		  //激光数据
vector<long> laser_data_intensity;	  //激光反射强度
objectParameter object_detect_result; //目标角度距离信息


//全局变量和局部变量　static关键字https://blog.csdn.net/weiyuefei/article/details/51563890

Vision::Vision() : it(nh)
{
	image_sub = it.subscribe("/usb_cam/image_raw", 1, &Vision::img_callback, this);
	laser_data_sub = nh.subscribe("/scan", 1000, &Vision::laser_callback, this);
	object_target_sub = nh.subscribe("/target_type", 1000, &Vision::object_target_callback, this);
	detect_data_pub = nh.advertise<const_msg::object_param>("/detect_result", 1000);
}

/****************************receive image data**********************/
void Vision::img_callback(const sensor_msgs::ImageConstPtr &img)
{
	cv_bridge::CvImagePtr cv_img_ptr;
	//toCvCopy()函数允许修改图像数据，BGR8编码方式CV_8UC3,颜色顺序为BGR
	try
	{
		cv_img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge error: s%", e.what());
		return;
	}

	/*image_data_original.image_original = cv_img_ptr->image;

	image_data_original.image_header.fram_id = cv_img_ptr->header.frame_id;
	image_data_original.image_header.seq = cv_img_ptr->header.seq;
	image_data_original.image_header.stamp = cv_img_ptr->header.stamp;
	image_original = cv_img_ptr->image;*/
	/****************image process function**********/
}

/************************receive laser data**********************/
void Vision::laser_callback(const sensor_msgs::LaserScanConstPtr &laser)
{
	for (size_t i = 0; i < laser->ranges.size(); i++)
	{
		long temp_1 = laser->ranges[i] * 1000; //单位换算，从meter_to_micrometer
		//laser_data_original.laser_data_range.push_back(temp_1);
		laser_data_range.push_back(temp_1);
		long temp_2 = laser->intensities[i] * 1000;
		laser_data_intensity.push_back(temp_2);
	}

	/*laser_data_original.laser_header.fram_id = laser->header.frame_id;
	laser_data_original.laser_header.seq 	 = laser->header.seq;
	laser_data_original.laser_header.stamp 	 = laser->header.stamp;*/
}

/***************************receive object target*******************/
void Vision::object_target_callback(const std_msgs::Int32ConstPtr &nums)
{
	object_type_target = (objectType)(nums->data);
	ROS_INFO_STREAM("target callback is: " << object_type_target);
}

/*************************************send final detecting reasult********************************************/
void Vision::send_final_result(pair<double, long> &final_object_param)
{
	const_msg::object_param param_msg;
	param_msg.obj_angle = final_object_param.first;
	param_msg.obj_distance = final_object_param.second;
	detect_data_pub.publish(param_msg);
}

/******************************************  find_calibration_by_LC *******************************************/
bool Vision::find_calibration_by_LC(Mat &imgOriginal, Point2f &objectCenter) //Point==Point_<int>==Point2i
{																			 //参数二：最大面积的中心位置
	/*int greenLowH=40;
	int greenHighH=77;   
	int greenLowS=218;
	int greenHighS=255;
	int greenLowV=0;
	int greenHighV=255;*/

	int greenLowH = 45;
	int greenHighH = 75;
	int greenLowS = 112;
	int greenHighS = 255;
	int greenLowV = 0;
	int greenHighV = 255;

	/*imshow("原图：",imgOriginal);
	char key=(char)waitKey(30);*/
	Mat imgOriginal_copy;
	imgOriginal.copyTo(imgOriginal_copy);

	Mat imgHSV_1;
	vector<Mat> hsvSplit_1;
	cvtColor(imgOriginal, imgHSV_1, COLOR_BGR2HSV);
	split(imgHSV_1, hsvSplit_1);				//分割成三个通道
	equalizeHist(hsvSplit_1[2], hsvSplit_1[2]); //均衡化
	merge(hsvSplit_1, imgHSV_1);

	Mat imgThresholded_1;
	inRange(imgHSV_1, Scalar(greenLowH, greenLowS, greenLowV),
			Scalar(greenHighH, greenHighS, greenHighV), imgThresholded_1); //二值化

	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5)); //返回指定形状和尺寸的结构元素。
	morphologyEx(imgThresholded_1, imgThresholded_1, MORPH_CLOSE, element);
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded_1, imgThresholded_1, MORPH_OPEN, element);
	//imshow("开操作闭操作之后的图：",imgThresholded_1);//形态学处理

	vector<vector<Point>> contours_1; //轮廓数
	vector<Vec4i> hierarchy_1;
	findContours(imgThresholded_1, contours_1, hierarchy_1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); //轮廓检测，只检测最外围轮廓，仅保存拐点信息。
	Scalar color_1(0, 255, 0);

	if (contours_1.size() > 0)
	{
		vector<double> contourAreaSize_1;
		for (size_t k = 0; k < contours_1.size(); k++)
		{
			contourAreaSize_1.push_back(contourArea(contours_1[k], false));
			cout << "面积：" << contourArea(contours_1[k], false) << endl;
		}
		auto it = max_element(contourAreaSize_1.begin(), contourAreaSize_1.end());
		size_t maxAreaIndex_1 = it - contourAreaSize_1.begin();
		cout << "最大面积的个数：" << maxAreaIndex_1 << endl;
		if ((*it) < 4000)
		{
			cout << "fail to find_calibration_by_LC" << endl;
			return false;
		}
		else
		{
			vector<Point> contourPoly_1;
			Point2f centerCircle_1; //中心点
			float radiusCircle_1;	//半径

			approxPolyDP(Mat(contours_1[maxAreaIndex_1]), contourPoly_1, 3, true);
			minEnclosingCircle((Mat)contourPoly_1, centerCircle_1, radiusCircle_1);

			circle(imgOriginal, centerCircle_1, (int)radiusCircle_1, color_1, 3, 8);
			circle(imgOriginal, centerCircle_1, 5, color_1, CV_FILLED, 8);
			//imshow("[效果图]",imgOriginal);

			objectCenter.x = centerCircle_1.x;
			objectCenter.y = centerCircle_1.y;
			cout << "找到物体的中心点的横坐标：" << objectCenter.x
				 << "找到物体中心点的纵坐标：" << objectCenter.x << endl;
			cout << "success to find_calibration" << endl;
			return true;
		}
	}
	else
	{
		cout << "fail to find_green_contours" << endl;
		return false;
	}
} //返回找到的物体像素中心点坐标
/*********************************find_calibration_object_by_laser_data*************************************/
bool Vision::find_calibration_object_by_laser_data(std::vector<long> &data, std::vector<pair<double, long>> &objectAngleDistance)
{ //参数二有两部分
	if (!data.empty())
	{
		for (size_t i = 0; i < data.size(); i++)
		{
			if (data[i] > 4000)
				data[i] = 4000; //最大的都为4000
		}
		int start = 0;
		int end = 0;
		for (size_t i = 260; i < 821; i++)
		//激光雷达扫描最佳范围为正负70度，换算过来为[260,820]
		{
			long dataSub = data[i + 1] - data[i];
			if (dataSub < -300 && data[i + 1] > 1000 && data[i + 2] < 4000)
			{
				start = i + 1;
				//cout<<start<<endl;
			}
			if (dataSub > 300 && start && data[i] > 1000 && data[i - 1] < 4000)
			{
				end = i;
			}
			if (start && end)
			{
				//if(end-start>=5 && end-start<=350)
				cout << "end-start" << end - start << endl;
				if (end - start > 2 && end - start <= 350)
				{
					//auto minDist = min_element(data.begin()+start,data.begin()+end);
					long centerDist = data[(int)((start + end) / 2)];
					/*if(centerDist-*minDist<=100)
					{*/
					pair<double, long> temPara;
					temPara.first = 0.25 * ((int)((start + end) / 2)); //角度
					temPara.second = centerDist;					   //距离
					objectAngleDistance.push_back(temPara);
					//}
				}
				else if (end - start > 350 && end - start < 600)
				{
					//auto minDist = min_element(data.begin()+start,data.begin()+end);
					long centerDist = data[(int)((start + end) / 2)];
					/*if(centerDist-*minDist<=100)
					{*/
					pair<double, long> temPara;
					temPara.first = 0.25 * ((int)((start + end) / 2)); //角度
					temPara.second = centerDist;					   //距离
					objectAngleDistance.push_back(temPara);
					//}
				}
				start = 0;
				end = 0;
			}
		}
	}
	if (objectAngleDistance.size() == 0)
	{
		cout << "fail to find_object_by_laser_data" << endl;
		return false;
	}
	else
	{
		cout << "success to find_object_by_laser_data" << endl;
		return true;
	}
}
/******************************************find_Reddish_brown_basketball(between two volleyball) *******************************************/
bool Vision::find_Reddish_brown_basketball(Mat &imgOriginal, Point2f &objectCenter) //Point==Point_<int>==Point2i
{																					//1、红棕色
	int Reddish_brownLowH = 0;
	int Reddish_brownHighH = 14;
	int Reddish_brownLowS = 80;
	int Reddish_brownHighS = 141;
	int Reddish_brownLowV = 0;
	int Reddish_brownHighV = 255; //上午

	/*int Reddish_brownLowH=0;
	int Reddish_brownHighH=17;   
	int Reddish_brownLowS=46;
	int Reddish_brownHighS=255;
	int Reddish_brownLowV=0;
	int Reddish_brownHighV=255;*/
	//不能识别红色的红棕色(下午)

	/*imshow("原图：",imgOriginal);
	char key=(char)waitKey(30);*/

	Mat imgHSV;
	vector<Mat> hsvSplit;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);
	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2], hsvSplit[2]);
	merge(hsvSplit, imgHSV);

	Mat imgThresholded;
	inRange(imgHSV, Scalar(Reddish_brownLowH, Reddish_brownLowS, Reddish_brownLowV),
			Scalar(Reddish_brownHighH, Reddish_brownHighS, Reddish_brownHighV), imgThresholded);
	//imshow("二值化后的图：",imgThresholded);

	/*medianBlur(imgThresholded,imgThresholded,5);
	imshow("中值滤波后的图：",imgThresholded);*/
	Mat element = getStructuringElement(MORPH_RECT, Size(8, 8));
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	//imshow("红棕色 开操作闭操作之后的图：",imgThresholded);
	///加canny边缘算子

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	/*Mat imgToDisplay;
	imgThresholded.copyTo(imgToDisplay);*/

	findContours(imgThresholded, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); //只检索最外面的轮廓
	cout << "红棕色轮廓的个数" << contours.size() << endl;
	Scalar color(0, 255, 0);
	//排除黄色
	int orangeLowH = 17;
	int orangeHighH = 33;
	int orangeLowS = 90;
	int orangeHighS = 255;
	int orangeLowV = 57;
	int orangeHighV = 255; //能识别排球黄色，识别不到蓝灰色的黄色

	Mat imgOriginal_copy;
	imgOriginal.copyTo(imgOriginal_copy);
	Mat imgHSV_1;
	vector<Mat> hsvSplit_1;
	cvtColor(imgOriginal_copy, imgHSV_1, COLOR_BGR2HSV);
	split(imgHSV_1, hsvSplit_1);
	equalizeHist(hsvSplit_1[2], hsvSplit_1[2]);
	merge(hsvSplit_1, imgHSV_1);

	Mat imgThresholded_1;
	inRange(imgHSV_1, Scalar(orangeLowH, orangeLowS, orangeLowV),
			Scalar(orangeHighH, orangeHighS, orangeHighV), imgThresholded_1);
	element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded_1, imgThresholded_1, MORPH_CLOSE, element);
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded_1, imgThresholded_1, MORPH_OPEN, element);
	//imshow("黄色 开操作闭操作之后的图：",imgThresholded_1);

	vector<vector<Point>> contours_1;
	vector<Vec4i> hierarchy_1;
	//Mat imgToDisplay;

	//imgThresholded.copyTo(imgToDisplay);

	//Canny(imgThresholded, imgThresholded, 128, 255, 3);
	findContours(imgThresholded_1, contours_1, hierarchy_1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	cout << "黄色轮廓的个数：" << contours_1.size() << endl;
	Scalar color_1(255, 0, 0);

	if (contours.size() > 0)
	{
		vector<double> contourAreaSize;
		vector<double> contourAreaSize_need;
		for (size_t k = 0; k < contours.size(); k++)
		{
			contourAreaSize.push_back(contourArea(contours[k], false));
			cout << "红棕色的面积：" << contourArea(contours[k], false) << endl;
		}

		for (size_t i = 0; i < contourAreaSize.size(); i++)
		{
			//if(contourAreaSize[i]>9000  && contourAreaSize[i]<60000)//红棕色大于9000，小于60000
			if (contourAreaSize[i] > 6000 && contourAreaSize[i] < 60000)
			{
				contourAreaSize_need.push_back(contourAreaSize[i]);
			}
		}
		if (contourAreaSize_need.size() > 0)
		{
			cout << "success to find reddish_brown_basketball" << endl;
			auto it = max_element(contourAreaSize_need.begin(), contourAreaSize_need.end());
			int maxIndex = it - contourAreaSize_need.begin();
			vector<Point> contourPoly;
			Point2f centerCircle;
			float radiusCircle;
			approxPolyDP(Mat(contours[maxIndex]), contourPoly, 3, true);
			minEnclosingCircle((Mat)contourPoly, centerCircle, radiusCircle);
			circle(imgOriginal, centerCircle, (int)radiusCircle, color, 3, 8);
			circle(imgOriginal, centerCircle, 5, color, CV_FILLED, 8);
			//imshow("画圆图；",imgOriginal);
			objectCenter = centerCircle;
			if (contours_1.size() > 0) //黄色不大于3000，就正确
			{
				vector<double> contourAreaSize_1;
				for (size_t k = 0; k < contours_1.size(); k++)
				{
					contourAreaSize_1.push_back(contourArea(contours_1[k], false));
					cout << "黄色的面积：" << contourArea(contours_1[k], false) << endl;
				}
				for (size_t i = 0; i < contourAreaSize_1.size(); i++)
				{
					if (contourAreaSize_1[i] > 4000)
					{
						cout << "fail to find reddish_brown_basketball(find orange)" << endl;
						return false;
					}
				}
			} //黄色轮廓if
		}
		else
		{
			cout << "fail to find_need_reddish_brown_area" << endl;
			return false;
		}
	}
	else
	{
		cout << "fail to find_reddish_brown_contours" << endl;
		return false;
	}
	return true;
}

/*********************************find_nearest_object_by_laser_data*************************************/
bool Vision::find_nearest_object_by_laser_data(std::vector<long> &data, std::vector<pair<double, long>> &objectAngleDistance)
{ //第一个有
	cout << "得到的激光的大小：" << data.size() << endl;
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
						pair<double, long> tempPair;
						tempPair.first = 0.25 * ((int)(start + end) / 2);
						tempPair.second = centerDist;
						objectAngleDistance.push_back(tempPair);
					}
					/*auto minDist = min_element(data.begin()+start,data.begin()+end); 
						long centerDist = data[(int)((start+end)/2)]; 
						if(centerDist-*minDist<=100)
						{
							pair<double,long>temPara;
							temPara.first=0.25*((int)((start+end)/2));
							temPara.second=centerDist;
							objectAngleDistance.push_back(temPara);
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
						pair<double, long> tempPair;
						tempPair.first = 0.25 * ((int)(start + end) / 2);
						tempPair.second = centerDist;
						objectAngleDistance.push_back(tempPair);
					}
				}
				start = 0;
				end = 0;
			}
		}
	}
	//	}
	if (objectAngleDistance.size() == 0)
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
/*********************************find_nearest_object_by_laser_data*************************************/
bool Vision::find_nearest_object_by_laser_data_laser_ball(std::vector<long> &data, std::vector<pair<double, long>> &objectAngleDistance)
{
	cout << "得到的激光的大小：" << data.size() << endl;
	if (!data.empty())
	{
		for (size_t i = 0; i < data.size(); i++)
		{
			if (data[i] > 3000)
				data[i] = 3000;
		}
		int start = 0;
		int end = 0;
		for (size_t i = 246; i < 840 /*data.size()-1*/; i++)
		{
			long dataSub = data[i + 1] - data[i];
			if (dataSub < -400 && data[i + 2] < 3000)
			{
				start = i + 1;
				cout << "start:" << start << endl;
			}
			if (dataSub > 400 && start && data[i - 1] < 3000)
			{
				end = i;
				cout << "end:" << end << endl;
			}
			if (start && end)
			{
				if (end - start >= 10 && end - start <= 350)
				{
					long centerDist = data[(int)((start + end) / 2)]; //计算物体中心点的坐标
					pair<double, long> tempPair;
					tempPair.first = 0.25 * ((int)(start + end) / 2);
					tempPair.second = centerDist;
					objectAngleDistance.push_back(tempPair);
					/*auto minDist = min_element(data.begin()+start,data.begin()+end); 
						long centerDist = data[(int)((start+end)/2)]; 
						if(centerDist-*minDist<=100)
						{
							pair<double,long>temPara;
							temPara.first=0.25*((int)((start+end)/2));
							temPara.second=centerDist;
							objectAngleDistance.push_back(temPara);
						}*/
				}
				start = 0;
				end = 0;
			}
		}
	}
	//	}
	if (objectAngleDistance.size() == 0)
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
/**************************find_blue_gray_basketball**************************************/
bool Vision::find_blue_gray_basketball(Mat &imgOriginal, Point2f &objectCenter)
{
	//1、蓝色
	int blueLowH = 95;
	int blueHighH = 128;
	int blueLowS = 89;
	int blueHighS = 255;
	int blueLowV = 0;
	int blueHighV = 255; //蓝色

	Mat imgOriginal_copy;
	imgOriginal.copyTo(imgOriginal_copy);

	Mat imgHSV;
	vector<Mat> hsvSplit;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);
	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2], hsvSplit[2]);
	merge(hsvSplit, imgHSV);

	Mat imgThresholded;
	inRange(imgHSV, Scalar(blueLowH, blueLowS, blueLowV),
			Scalar(blueHighH, blueHighS, blueHighV), imgThresholded);
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	/*imshow("蓝色 开操作闭操作之后的图：",imgThresholded);
    waitKey(30);*/

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(imgThresholded, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	cout << "蓝色轮廓的个数：" << contours.size() << endl;
	Scalar color(0, 0, 255);

	int orangeLowH = 17;
	int orangeHighH = 33;
	int orangeLowS = 120;
	int orangeHighS = 255;
	int orangeLowV = 57;
	int orangeHighV = 255; //能识别排球黄色，识别不到蓝灰色的黄色

	Mat imgHSV_1;
	vector<Mat> hsvSplit_1;
	cvtColor(imgOriginal_copy, imgHSV_1, COLOR_BGR2HSV);
	split(imgHSV_1, hsvSplit_1);
	equalizeHist(hsvSplit_1[2], hsvSplit_1[2]);
	merge(hsvSplit_1, imgHSV_1);

	Mat imgThresholded_1;
	inRange(imgHSV_1, Scalar(orangeLowH, orangeLowS, orangeLowV),
			Scalar(orangeHighH, orangeHighS, orangeHighV), imgThresholded_1);
	element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded_1, imgThresholded_1, MORPH_CLOSE, element);
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded_1, imgThresholded_1, MORPH_OPEN, element);
	/*imshow("黄色 开操作闭操作之后的图：",imgThresholded_1);
    waitKey(30);*/

	vector<vector<Point>> contours_1;
	vector<Vec4i> hierarchy_1;
	findContours(imgThresholded_1, contours_1, hierarchy_1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	cout << "黄色轮廓的个数：" << contours_1.size() << endl;
	Scalar color_1(255, 0, 0);
	Point2f centerCircle_Blue;
	float radiusCircle_Blue;
	if (contours.size() > 0) //蓝色轮廓
	{
		vector<double> contourAreaSize;
		vector<double> contourAreaSize_need;
		for (size_t k = 0; k < contours.size(); k++)
		{
			contourAreaSize.push_back(contourArea(contours[k], false));
			cout << "每个蓝色轮廓的面积：" << contourArea(contours[k], false) << endl;
		}
		for (size_t k = 0; k < contourAreaSize.size(); k++)
		{
			if (contourAreaSize[k] > 8000 && contourAreaSize[k] < 60000) //蓝色面积需要修改
				contourAreaSize_need.push_back(contourAreaSize[k]);
		}
		if (contourAreaSize_need.size() == 0)
		{
			cout << "没有蓝色面积，不是蓝灰色球" << endl;
			return false;
		}
		else
		{
			cout << "蓝灰球的蓝色面积满足" << endl;
			auto it = max_element(contourAreaSize_need.begin(), contourAreaSize_need.end());
			size_t maxAreaIndex = it - contourAreaSize_need.begin();
			vector<Point> contourPoly;
			Point2f centerCircle;
			float radiusCircle;
			approxPolyDP(Mat(contours[maxAreaIndex]), contourPoly, 3, true);
			minEnclosingCircle((Mat)contourPoly, centerCircle, radiusCircle);
			circle(imgOriginal, centerCircle, (int)radiusCircle, color, 3, 8);
			circle(imgOriginal, centerCircle, 5, color, CV_FILLED, 8);
			centerCircle_Blue = centerCircle;
			radiusCircle_Blue = radiusCircle; //大于1000的都画圈圈
											  //imshow("[蓝色画圆图]",imgOriginal);
			if (contours_1.size() > 0)		  //黄色轮廓
			{
				vector<double> contourAreaSize_1;
				vector<double> contourAreaSize_need_1;
				for (size_t k = 0; k < contours_1.size(); k++)
				{
					contourAreaSize_1.push_back(contourArea(contours_1[k], false));
					cout << "每个黄色轮廓的面积：" << contourArea(contours_1[k], false) << endl;
				}
				for (size_t k = 0; k < contourAreaSize_1.size(); k++)
				{
					if (contourAreaSize_1[k] > 4000) //根据现场环境再测
						contourAreaSize_need_1.push_back(contourAreaSize_1[k]);
				}
				if (contourAreaSize_need_1.size() > 0)
				{
					cout << "有排球的黄色，不是蓝灰色球" << endl;
					return false;
				}
			}
		} //蓝色的else
	}
	else
	{
		cout << "没有蓝色轮廓，不是蓝灰色" << endl;
		return false;
	}
	objectCenter = centerCircle_Blue;
	return true;
}
/**************************find_blue_gray_basketball_plus**************************************/
bool Vision::find_blue_gray_basketball_plus(Mat &imgOriginal, Point2f &objectCenter)
{ //1、黄色
	int orangeLowH = 17;
	int orangeHighH = 33;
	int orangeLowS = 120;
	int orangeHighS = 255;
	int orangeLowV = 0;
	int orangeHighV = 255;

	Mat imgOriginal_copy;
	imgOriginal.copyTo(imgOriginal_copy);
	imshow("黄色 原图", imgOriginal);
	//imshow("复制图：",imgOriginal_copy);

	Mat imgHSV;
	vector<Mat> hsvSplit;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);
	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2], hsvSplit[2]);
	merge(hsvSplit, imgHSV);

	Mat imgThresholded;
	inRange(imgHSV, Scalar(orangeLowH, orangeLowS, orangeLowV),
			Scalar(orangeHighH, orangeHighS, orangeHighV), imgThresholded);

	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	imshow("黄色 开操作闭操作之后的图：", imgThresholded);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	//Mat imgToDisplay;

	//imgThresholded.copyTo(imgToDisplay);

	//Canny(imgThresholded, imgThresholded, 128, 255, 3);
	findContours(imgThresholded, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	cout << "黄色轮廓的个数：" << contours.size() << endl;
	Scalar color(255, 0, 0);

	//2、蓝色
	int blueLowH = 95;
	int blueHighH = 128;
	int blueLowS = 89;
	int blueHighS = 255;
	int blueLowV = 0;
	int blueHighV = 255;
	Mat imgHSV_1;
	vector<Mat> hsvSplit_1;
	cvtColor(imgOriginal_copy, imgHSV_1, COLOR_BGR2HSV);
	split(imgHSV_1, hsvSplit_1);
	equalizeHist(hsvSplit_1[2], hsvSplit_1[2]);
	merge(hsvSplit_1, imgHSV_1);

	Mat imgThresholded_1;
	inRange(imgHSV_1, Scalar(blueLowH, blueLowS, blueLowV),
			Scalar(blueHighH, blueHighS, blueHighV), imgThresholded_1);

	element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded_1, imgThresholded_1, MORPH_CLOSE, element);
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded_1, imgThresholded_1, MORPH_OPEN, element);
	imshow("蓝色 开操作闭操作之后的图：", imgThresholded_1);

	vector<vector<Point>> contours_1;
	vector<Vec4i> hierarchy_1;
	//Mat imgToDisplay_1;
	//imgThresholded_1.copyTo(imgToDisplay_1);

	//Canny(imgThresholded, imgThresholded, 128, 255, 3);
	findContours(imgThresholded_1, contours_1, hierarchy_1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	cout << "蓝色色块轮廓的个数：" << contours_1.size() << endl;
	Scalar color_1(0, 0, 255);

	Point2f centerCircle_Orange;
	float radiusCircle_Orange;
	Point2f centerCircle_Blue;
	float radiusCircle_Blue;

	if (contours.size() > 0) //黄色轮廓
	{
		vector<double> contourAreaSize;
		vector<double> contourAreaSize_need;
		for (size_t k = 0; k < contours.size(); k++)
		{
			contourAreaSize.push_back(contourArea(contours[k], false));
			cout << "每个黄色轮廓的面积：" << contourArea(contours[k], false) << endl;
		}
		for (size_t k = 0; k < contourAreaSize.size(); k++)
		{
			if (contourAreaSize[k] > 200) //需要修改面积值
			{
				contourAreaSize_need.push_back(contourAreaSize[k]);
			}
		}
		if (contourAreaSize_need.size() == 0)
		{
			cout << "没有黄色，不是蓝灰色球" << endl;
			return false;
		}
		else
		{
			cout << "success to find orange_block" << endl;
			auto it = max_element(contourAreaSize.begin(), contourAreaSize.end());
			size_t maxAreaIndex = it - contourAreaSize.begin();
			vector<Point> contourPoly;
			Point2f centerCircle;
			float radiusCircle;
			approxPolyDP(Mat(contours[maxAreaIndex]), contourPoly, 3, true);
			minEnclosingCircle((Mat)contourPoly, centerCircle, radiusCircle);
			circle(imgOriginal, centerCircle, (int)radiusCircle, color, 3, 8);
			circle(imgOriginal, centerCircle, 5, color, CV_FILLED, 8);
			centerCircle_Orange = centerCircle;
			radiusCircle_Orange = radiusCircle;
			imshow("[黄色 画圆图]", imgOriginal);
			/////判断蓝色
			if (contours_1.size() > 0)
			{
				vector<double> contourAreaSize_1;
				vector<double> contourAreaSize_need_1;
				for (size_t k = 0; k < contours_1.size(); k++)
				{
					contourAreaSize_1.push_back(contourArea(contours_1[k], false));
					cout << "每个蓝色轮廓的面积：" << contourArea(contours_1[k], false) << endl;
				}
				for (size_t k = 0; k < contourAreaSize_1.size(); k++)
				{
					if (contourAreaSize_1[k] > 1000 && contourAreaSize_1[k] < 25000) //面积值还需修改
					{
						contourAreaSize_need_1.push_back(contourAreaSize_1[k]);
					}
				}
				if (contourAreaSize_need_1.size() == 0)
				{
					cout << "fail to find blue_gray_basketball(blue block)" << endl;
					return false;
				}
				else
				{
					cout << "success to find blue_gray_basketball(blue block)" << endl;
					auto it = max_element(contourAreaSize_need_1.begin(), contourAreaSize_need_1.end());
					size_t maxAreaIndex_1 = it - contourAreaSize_need_1.begin();
					vector<Point> contourPoly_1;
					Point2f centerCircle_1;
					float radiusCircle_1;
					approxPolyDP(Mat(contours_1[maxAreaIndex_1]), contourPoly_1, 3, true);
					minEnclosingCircle((Mat)contourPoly_1, centerCircle_1, radiusCircle_1);
					circle(imgOriginal_copy, centerCircle_1, (int)radiusCircle_1, color_1, 3, 8);
					circle(imgOriginal_copy, centerCircle_1, 5, color_1, CV_FILLED, 8);
					centerCircle_Blue = centerCircle_1;
					radiusCircle_Blue = radiusCircle_1;
					imshow("[蓝色 画圆图]", imgOriginal_copy);
				}
			}
			else
			{
				cout << "fail to find blue_gray_basketball(blue_block)" << endl;
				return false;
			}
		}
	}
	else
	{
		cout << "fail to find blue_gray_basketball(orange_block)" << endl;
		return false;
	}

	if (centerCircle_Orange.x != 0 && centerCircle_Blue.x == 0)
	{
		objectCenter = centerCircle_Orange;
		return true;
	}
	if (centerCircle_Orange.x != 0 && centerCircle_Blue.x != 0)
	{
		objectCenter.x += centerCircle_Orange.x + centerCircle_Blue.x;
		objectCenter.y += centerCircle_Orange.y + centerCircle_Blue.y;
		objectCenter.x /= 2;
		objectCenter.y /= 2;
		return true;
	}
}
/**************************find_basketball**************************************/
bool Vision::find_basketball(Mat &imgOriginal, Point2f &objectCener)
{
	Mat imgOriginal_copy;
	imgOriginal.copyTo(imgOriginal_copy);
	char key = (char)waitKey(30);

	Point2f objectCenter_1;
	Point2f objectCenter_2;
	if (find_Reddish_brown_basketball(imgOriginal, objectCenter_1))
	{
		cout << "success to find_basketball(reddish_brown)" << endl;
		objectCener = objectCenter_1;
		return true;
	}
	else
	{
		cout << "fail to find_basketball(reddish_brown)" << endl;
		if (find_blue_gray_basketball(imgOriginal_copy, objectCenter_2))
		{
			cout << "success to find_basketball(blue_gray)" << endl;
			objectCener = objectCenter_2;
			return true;
		}
		else
		{
			cout << "fail to find _all_basketball" << endl;
			return false;
		}
	}
}

/********************find_final_object***********************************/
bool Vision::find_final_object(vector<pair<double, long>> &objectAngleDistance, Point2f &objectCenter, pair<double, long> &finalobjectAngleDistance)
{ //第一个有
	if (objectAngleDistance.size() == 0 || objectCenter.x == 0)
	{
		cout << "fail to find_first_basketball" << endl;
		return false;
	}
	else
	{
		cout << "success to  find_first_basketball" << endl;
		cout << "激光物体角度距离的组数：" << objectAngleDistance.size() << endl; //错了？
		for (size_t i = 0; i < objectAngleDistance.size(); i++)
		{
			cout << "激光所测物体的角度：" << objectAngleDistance[i].first
				 << "激光所测物体的距离：" << objectAngleDistance[i].second << endl;
		}
		cout << "物体的中心点的横坐标(摄像头)：" << objectCenter.x
			 << "物体中心点的纵坐标：" << objectCenter.y << endl;
		vector<double> absValue;
		double objectCenterAngle;

		if (objectCenter.x < 320)
		{
			objectCenterAngle = 90 + (320 - objectCenter.x) * ((double)38 / 640);
			cout << "中心点的角度(转换坐标后)：" << objectCenterAngle << endl;
		}
		else
		{
			objectCenterAngle = 90 - (objectCenter.x - 320) * ((double)38 / 640);
			cout << "中心点的角度(转换坐标后)：" << objectCenterAngle << endl;
		}

		/*if(objectCenter.x <640)     
		{
			objectCenterAngle=90+(640-objectCenter.x)*((double)38/1280);
			cout<<"中心点的角度："<<objectCenterAngle<<endl;
		}else{
	         objectCenterAngle=71+(objectCenter.x-640)*((double)38/1280);
			cout<<"中心点的角度："<<objectCenterAngle<<endl;
		}*/

		for (size_t i = 0; i < objectAngleDistance.size(); i++)
		{
			absValue.push_back(abs(objectAngleDistance[i].first - objectCenterAngle));
		}
		for (size_t i = 0; i < absValue.size(); i++)
		{
			cout << "差值角度：" << absValue[i] << endl;
		}
		auto it = min_element(absValue.begin(), absValue.end());
		if (*it < angleDist) //当差值小于10时就找到了
		{
			size_t index = it - absValue.begin();
			finalobjectAngleDistance.first = objectAngleDistance[index].first;
			finalobjectAngleDistance.second = objectAngleDistance[index].second;

			cout << "最后篮球的角度：" << finalobjectAngleDistance.first
				 << "最后篮球的距离：" << finalobjectAngleDistance.second << endl;
			cout << "success to find_final_basketball" << endl;
			return true;
		}
		else
		{
			cout << "fail to find_final_basketball" << endl;
			return false;
		}
	}
}
/********************find_final_object__plus***********************************/
bool Vision::find_final_object_plus(vector<pair<double, long>> &objectAngleDistance, Point2f &objectCenter, pair<double, long> &finalobjectAngleDistance)
{
	if ((objectAngleDistance.size() == 0) || objectCenter.x == 0)
	{
		cout << "fail to find_first_object" << endl;
		return false;
	}
	else
	{
		cout << "success to  find_first_object" << endl;
		cout << "共有物体角度距离：" << objectAngleDistance.size() << "对" << endl;
		for (size_t i = 0; i < objectAngleDistance.size(); i++)
		{
			cout << "物体的角度：" << objectAngleDistance[i].first
				 << "物体的距离：" << objectAngleDistance[i].second << endl;
		}
		cout << "物体的中心点的横坐标：" << objectCenter.x
			 << "物体中心点的纵坐标：" << objectCenter.y << endl;
		vector<double> absValue;
		double objectCenterAngle;
		if (objectCenter.x < 320)
		{
			//objectCenterAngle = 180-((180-38)/2+(objectCenter.x+1)*((double)38/640));
			// objectCenterAngle=(180-38)/2+(objectCenter.x)*((double)38/640);
			objectCenterAngle = 90 + (320 - objectCenter.x) * ((double)38 / 640);
			cout << "中心点的角度：" << objectCenterAngle << endl;
		}
		else
		{
			//objectCenterAngle = 180-(90+(objectCenter.x+1-320)*((double)38/640));
			//objectCenterAngle= 90+(objectCenter.x-320)*((double)38/640);
			objectCenterAngle = 71 + (640 - objectCenter.x) * ((double)38 / 640);
			cout << "中心点的角度：" << objectCenterAngle << endl;
		}
		for (size_t i = 0; i < objectAngleDistance.size(); i++)
		{
			absValue.push_back(abs(objectAngleDistance[i].first - objectCenterAngle));
		}

		for (size_t i = 0; i < absValue.size(); i++)
		{
			cout << "差值角度：" << absValue[i] << endl;
		}
		auto it = min_element(absValue.begin(), absValue.end());
		if (*it < angleDist)
		{
			size_t index_1 = it - absValue.begin();
			finalobjectAngleDistance.first = objectAngleDistance[index_1].first;
			vector<long> distance;
			for (size_t i = 0; i < objectAngleDistance.size(); i++)
			{
				distance.push_back(objectAngleDistance[i].second);
			}
			auto it = min_element(distance.begin(), distance.end());
			size_t index_2 = it - distance.begin();
			finalobjectAngleDistance.second = objectAngleDistance[index_2].second;

			cout << "success to find_final_object__plus" << endl;
			return true;
		}
		else
		{
			cout << "fail to find_final_object__plus" << endl;
			return false;
		}
	}
}
/**************************find_red_orange_volleyball**************************************/
bool Vision::find_orange_red_volleyball(Mat &imgOriginal, Point2f &objectCenter)
{
	//1、黄色
	int orangeLowH = 17;
	int orangeHighH = 33;
	int orangeLowS = 120;
	int orangeHighS = 255;
	int orangeLowV = 0;
	int orangeHighV = 255;

	Mat imgOriginal_copy;
	imgOriginal.copyTo(imgOriginal_copy);
	/*imshow("橘色 原图",imgOriginal);
	char key=(char)waitKey(30);*/

	Mat imgHSV;
	vector<Mat> hsvSplit;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);
	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2], hsvSplit[2]);
	merge(hsvSplit, imgHSV);

	Mat imgThresholded;
	inRange(imgHSV, Scalar(orangeLowH, orangeLowS, orangeLowV),
			Scalar(orangeHighH, orangeHighS, orangeHighV), imgThresholded);

	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	//imshow("橘色 开操作闭操作之后的图：",imgThresholded);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	//Mat imgToDisplay_1;
	//imgThresholded_1.copyTo(imgToDisplay_1);

	//Canny(imgThresholded, imgThresholded, 128, 255, 3);
	findContours(imgThresholded, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	cout << "黄色轮廓的个数：" << contours.size() << endl;
	Scalar color(0, 255, 0);

	//2、蓝色
	int blueLowH = 95;
	int blueHighH = 128;
	int blueLowS = 89;
	int blueHighS = 255;
	int blueLowV = 0;
	int blueHighV = 255;
	// int iLowH=80;
	////int iHighH=130;
	////int iLowS=80;
	////int iHighS=255;
	////int iLowV=0;
	////int iHighV=255;

	Mat imgHSV_1;
	vector<Mat> hsvSplit_1;
	cvtColor(imgOriginal_copy, imgHSV_1, COLOR_BGR2HSV);
	split(imgHSV_1, hsvSplit_1);
	equalizeHist(hsvSplit_1[2], hsvSplit_1[2]);
	merge(hsvSplit_1, imgHSV_1);
	Mat imgThresholded_1;
	inRange(imgHSV_1, Scalar(blueLowH, blueLowS, blueLowV),
			Scalar(blueHighH, blueHighS, blueHighV), imgThresholded_1);
	element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded_1, imgThresholded_1, MORPH_CLOSE, element);
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded_1, imgThresholded_1, MORPH_OPEN, element);
	//imshow("蓝色开操作闭操作之后的图：",imgThresholded_1);
	vector<vector<Point>> contours_1;
	vector<Vec4i> hierarchy_1;

	findContours(imgThresholded_1, contours_1, hierarchy_1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	cout << "蓝色轮廓的个数：" << contours_1.size() << endl;
	Scalar color_1(0, 255, 255);

	//3、红色
	int redLowH = 140;
	int redHighH = 255;
	int redLowS = 84;
	int redHighS = 255;
	int redLowV = 0;
	int redHighV = 255;

	Mat imgOriginal_copy_2;
	imgOriginal.copyTo(imgOriginal_copy_2);
	char key1 = (char)waitKey(30);

	Mat imgHSV_2;
	vector<Mat> hsvSplit_2;
	cvtColor(imgOriginal_copy_2, imgHSV_2, COLOR_BGR2HSV);
	split(imgHSV_2, hsvSplit_2);
	equalizeHist(hsvSplit_2[2], hsvSplit_2[2]);
	merge(hsvSplit_2, imgHSV_2);

	Mat imgThresholded_2;
	inRange(imgHSV_2, Scalar(redLowH, redLowS, redLowV),
			Scalar(redHighH, redHighS, redHighV), imgThresholded_2);
	element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded_2, imgThresholded_2, MORPH_CLOSE, element);
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded_2, imgThresholded_2, MORPH_OPEN, element);
	//imshow("红色 开操作闭操作之后的图：",imgThresholded_2);

	vector<vector<Point>> contours_2;
	vector<Vec4i> hierarchy_2;
	//Mat imgToDisplay_1;
	//imgThresholded_1.copyTo(imgToDisplay_1);

	//Canny(imgThresholded, imgThresholded, 128, 255, 3);
	findContours(imgThresholded_2, contours_2, hierarchy_2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	cout << "红色轮廓的个数：" << contours_2.size() << endl;
	Scalar color_2(0, 255, 0);

	Point2f centerCircle_Orange;
	float radiusCircle_Orange;
	Point2f centerCircle_Blue;
	float radiusCircle_Blue;
	Point2f centerCircle_Red;
	float radiusCircle_Red;

	if (contours.size() > 0) //黄色
	{
		vector<double> contourAreaSize;
		for (size_t k = 0; k < contours.size(); k++)
		{
			contourAreaSize.push_back(contourArea(contours[k], false));
			cout << "黄色的面积：" << contourArea(contours[k], false) << endl;
		}

		vector<double> contourAreaSize_need;
		for (size_t i = 0; i < contourAreaSize.size(); i++)
		{
			if (contourAreaSize[i] > 5000 && contourAreaSize[i] < 25000)
			{
				//cout<<"success to find_orange_block"<<endl;
				contourAreaSize_need.push_back(contourAreaSize[i]);
			}
		}
		if (contourAreaSize_need.size() == 0)
		{
			cout << "fail to find orange_block" << endl;
			return false;
		}
		else
		{
			cout << "success to find_orange_block" << endl;
			/*for(size_t i=0;i<contourAreaSize_1_need.size();i++)
			{*/
			auto it = max_element(contourAreaSize_need.begin(), contourAreaSize_need.end());
			size_t maxAreaIndex = it - contourAreaSize_need.begin();
			vector<Point> contourPoly;
			Point2f centerCircle;
			float radiusCircle;
			approxPolyDP(Mat(contours[maxAreaIndex]), contourPoly, 3, true);
			minEnclosingCircle((Mat)contourPoly, centerCircle, radiusCircle);
			circle(imgOriginal, centerCircle, (int)radiusCircle, color, 3, 8);
			circle(imgOriginal, centerCircle, 5, color, CV_FILLED, 8);
			centerCircle_Orange = centerCircle;
			radiusCircle_Orange = radiusCircle; //大于1000的都画圈圈
			//imshow("[黄色画圆图]",imgOriginal);

			if (contours_1.size() > 0)
			{
				vector<double> contourAreaSize_1;
				for (size_t k = 0; k < contours_1.size(); k++)
				{
					contourAreaSize_1.push_back(contourArea(contours_1[k], false));
					cout << "蓝色的面积：" << contourArea(contours_1[k], false) << endl;
				}
				for (size_t k = 0; k < contourAreaSize_1.size(); k++)
				{
					if (contourAreaSize_1[k] > 1800)
					{
						cout << "success to find blue_block,but it's not orange_red_volleyball" << endl;
						return false;
					}
				}
			}

			if (contours_2.size() > 0)
			{
				vector<double> contourAreaSize_2;
				vector<double> contourAreaSize_2_need;
				for (size_t k = 0; k < contours_2.size(); k++)
				{
					contourAreaSize_2.push_back(contourArea(contours_2[k], false));
					cout << "红色的面积：" << contourArea(contours_2[k], false) << endl;
				}
				for (size_t i = 0; i < contourAreaSize_2.size(); i++)
				{
					if (contourAreaSize_2[i] > 1800 && contourAreaSize_2[i] < 20000) //红色大于1000
					{
						contourAreaSize_2_need.push_back(contourAreaSize_2[i]);
					}
				}
				if (contourAreaSize_2_need.size() == 0)
				{
					cout << "fail to find orange_red_volleyball red_block" << endl;
					return false;
				}
				else
				{
					cout << "success to find red_block,it's orange_red_volleyball " << endl;
					auto it = max_element(contourAreaSize_2_need.begin(), contourAreaSize_2_need.end());
					size_t maxAreaIndex_2 = it - contourAreaSize_2_need.begin();
					vector<Point> contourPoly_2;
					Point2f centerCircle_2;
					float radiusCircle_2;
					approxPolyDP(Mat(contours_2[maxAreaIndex_2]), contourPoly_2, 3, true);
					minEnclosingCircle((Mat)contourPoly_2, centerCircle_2, radiusCircle_2);
					circle(imgOriginal_copy_2, centerCircle_2, (int)radiusCircle_2, color_2, 3, 8); //仅最大的拟合
					//circle(imgOriginal,centerCircle_3,(int)radiusCircle_3,color_3,3,8);

					circle(imgOriginal_copy_2, centerCircle_2, 5, color_2, CV_FILLED, 8);
					//circle(imgOriginal,centerCircle_3,5,color_2,CV_FILLED,8);

					centerCircle_Red = centerCircle_2;
					radiusCircle_Red = radiusCircle_2;
					//imshow("[红色画圆图]",imgOriginal_copy_2);
				}
			}
			else
			{
				cout << "fail to find_red_block" << endl;
				return false;
			}
		}
	}
	else
	{
		cout << "fail to find_orange_block" << endl;
		return false;
	}
	if (centerCircle_Orange.x != 0 && centerCircle_Red.x == 0)
	{
		objectCenter = centerCircle_Orange;
		return true;
	}

	if (centerCircle_Orange.x != 0 && centerCircle_Red.x != 0)
	{
		/*for(size_t i=0;i<centerCircle_Orange_Container.size();i++)
		{*/
		objectCenter.x += centerCircle_Orange.x + centerCircle_Red.x;
		objectCenter.y += centerCircle_Orange.y + centerCircle_Red.y;
		objectCenter.x /= 2;
		objectCenter.y /= 2;
		return true;
	}
}
/*************************find_blue_red_volleyball*************************/
bool Vision::find_orange_blue_volleyball(Mat &imgOriginal, Point2f &objectCenter)
{
	//1、黄色
	int orangeLowH = 17;
	int orangeHighH = 33;
	int orangeLowS = 120;
	int orangeHighS = 255;
	int orangeLowV = 57;
	int orangeHighV = 255; //能识别排球黄色，识别不到蓝灰色的黄色

	Mat imgOriginal_copy; //定义剪切的图像
	imgOriginal.copyTo(imgOriginal_copy);
	Mat imgOriginal_copy_1; //定义剪切的图像
	imgOriginal.copyTo(imgOriginal_copy_1);
	Mat imgHSV;
	vector<Mat> hsvSplit;

	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);
	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2], hsvSplit[2]);
	merge(hsvSplit, imgHSV);

	Mat imgThresholded;

	inRange(imgHSV, Scalar(orangeLowH, orangeLowS, orangeLowV),
			Scalar(orangeHighH, orangeHighS, orangeHighV), imgThresholded);

	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	/*imshow("黄色 开操作闭操作之后的图：",imgThresholded);
    waitKey(30);*/
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(imgThresholded, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	cout << "黄色轮廓的个数：" << contours.size() << endl;
	Scalar color(0, 0, 255);

	int redLowH = 140;
	int redHighH = 255;
	int redLowS = 84;
	int redHighS = 255;
	int redLowV = 0;
	int redHighV = 255;

	Mat imgHSV_1;
	vector<Mat> hsvSplit_1;
	cvtColor(imgOriginal_copy, imgHSV_1, COLOR_BGR2HSV);
	split(imgHSV_1, hsvSplit_1);
	equalizeHist(hsvSplit_1[2], hsvSplit_1[2]);
	merge(hsvSplit_1, imgHSV_1);

	Mat imgThresholded_1;
	inRange(imgHSV_1, Scalar(redLowH, redLowS, redLowV),
			Scalar(redHighH, redHighS, redHighV), imgThresholded_1);
	element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded_1, imgThresholded_1, MORPH_CLOSE, element);
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded_1, imgThresholded_1, MORPH_OPEN, element);
	/*imshow("红色 开操作闭操作之后的图：",imgThresholded_1);
    waitKey(30);*/
	vector<vector<Point>> contours_1;
	vector<Vec4i> hierarchy_1;
	findContours(imgThresholded_1, contours_1, hierarchy_1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	cout << "红色轮廓的个数：" << contours_1.size() << endl;
	Scalar color_1(255, 0, 0);

	int blueLowH = 95;
	int blueHighH = 128;
	int blueLowS = 95;
	int blueHighS = 128;
	int blueLowV = 0;
	int blueHighV = 255; //蓝色

	Mat imgHSV_2;
	vector<Mat> hsvSplit_2;
	cvtColor(imgOriginal_copy_1, imgHSV_2, COLOR_BGR2HSV);
	split(imgHSV_2, hsvSplit_2);
	equalizeHist(hsvSplit_2[2], hsvSplit_2[2]);
	merge(hsvSplit_2, imgHSV_2);

	Mat imgThresholded_2;
	inRange(imgHSV_2, Scalar(blueLowH, blueLowS, blueLowV),
			Scalar(blueHighH, blueHighS, blueHighV), imgThresholded_2);
	element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded_2, imgThresholded_2, MORPH_CLOSE, element);
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded_2, imgThresholded_2, MORPH_OPEN, element);
	//imshow("蓝色 开操作闭操作之后的图：",imgThresholded_2);

	vector<vector<Point>> contours_2;
	vector<Vec4i> hierarchy_2;

	findContours(imgThresholded_2, contours_2, hierarchy_2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	cout << "蓝色轮廓的个数：" << contours_2.size() << endl;
	Scalar color_2(0, 255, 0);
	Point2f centerCircle_Orange;
	float radiusCircle_Orange;
	Point2f centerCircle_Blue;
	float radiusCircle_Blue;
	if (contours.size() > 0) //黄色轮廓
	{
		vector<double> contourAreaSize;
		vector<double> contourAreaSize_need;
		for (size_t k = 0; k < contours.size(); k++)
		{
			contourAreaSize.push_back(contourArea(contours[k], false));
			cout << "每个黄色轮廓的面积：" << contourArea(contours[k], false) << endl;
		}
		for (size_t k = 0; k < contourAreaSize.size(); k++)
		{
			if (contourAreaSize[k] > 4000 && contourAreaSize[k] < 20000) //黄色面积需要修改
				contourAreaSize_need.push_back(contourAreaSize[k]);
		}
		if (contourAreaSize_need.size() == 0)
		{
			cout << "没有黄色面积，不是黄蓝排球" << endl;
			return false;
		}
		else
		{
			cout << "黄色面积满足黄蓝排球" << endl;
			auto it = max_element(contourAreaSize_need.begin(), contourAreaSize_need.end());
			size_t maxAreaIndex = it - contourAreaSize_need.begin();
			vector<Point> contourPoly;
			Point2f centerCircle;
			float radiusCircle;
			approxPolyDP(Mat(contours[maxAreaIndex]), contourPoly, 3, true);
			minEnclosingCircle((Mat)contourPoly, centerCircle, radiusCircle);
			circle(imgOriginal, centerCircle, (int)radiusCircle, color, 3, 8);
			circle(imgOriginal, centerCircle, 5, color, CV_FILLED, 8);
			centerCircle_Orange = centerCircle;
			radiusCircle_Orange = radiusCircle; //大于1000的都画圈圈
												//imshow("[黄色画圆图]",imgOriginal);
			if (contours_1.size() > 0)			//红色轮廓
			{
				vector<double> contourAreaSize_1;
				vector<double> contourAreaSize_need_1;
				for (size_t k = 0; k < contours_1.size(); k++)
				{
					contourAreaSize_1.push_back(contourArea(contours_1[k], false));
					cout << "每个红色轮廓的面积：" << contourArea(contours_1[k], false) << endl;
				}
				for (size_t k = 0; k < contourAreaSize_1.size(); k++)
				{
					if (contourAreaSize_1[k] > 1800)
						contourAreaSize_need_1.push_back(contourAreaSize_1[k]);
				}
				if (contourAreaSize_need_1.size() > 0)
				{
					cout << "有红色，不是黄蓝排球" << endl;
					return false;
				}
			}						   //红色if轮廓
			if (contours_2.size() > 0) //蓝色轮廓
			{
				vector<double> contourAreaSize_2;
				vector<double> contourAreaSize_need_2;
				for (size_t k = 0; k < contours_2.size(); k++)
				{
					contourAreaSize_2.push_back(contourArea(contours_2[k], false));
					cout << "每个蓝色轮廓的面积：" << contourArea(contours_2[k], false) << endl;
				}
				for (size_t k = 0; k < contourAreaSize_2.size(); k++)
				{
					if (contourAreaSize_2[k] > 1000 && contourAreaSize_2[k] < 20000) //蓝色面积需要修改
						contourAreaSize_need_2.push_back(contourAreaSize_2[k]);
				}
				if (contourAreaSize_need_2.size() == 0)
				{
					cout << "没有蓝色面积，不是黄蓝球" << endl;
					return false;
				}
				else
				{
					cout << "有蓝色面积，是黄篮球" << endl;
					auto it = max_element(contourAreaSize_need_2.begin(), contourAreaSize_need_2.end());
					size_t maxAreaIndex_1 = it - contourAreaSize_need_2.begin();
					vector<Point> contourPoly_1;
					Point2f centerCircle_1;
					float radiusCircle_1;
					approxPolyDP(Mat(contours_2[maxAreaIndex_1]), contourPoly_1, 3, true);
					minEnclosingCircle((Mat)contourPoly_1, centerCircle_1, radiusCircle_1);
					circle(imgOriginal_copy, centerCircle_1, (int)radiusCircle_1, color_1, 3, 8);
					circle(imgOriginal_copy, centerCircle_1, 5, color_1, CV_FILLED, 8);
					centerCircle_Blue = centerCircle_1;
					radiusCircle_Blue = radiusCircle_1; //大于1000的都画圈圈
														//imshow("[蓝色画圆图]",imgOriginal_copy_1);
				}
			}
			else
			{
				cout << "没有蓝色轮廓，不是黄蓝排球" << endl;
				return false;
			}
		} //蓝色的else
	}
	else
	{
		cout << "没有黄色轮廓，不是黄蓝排球" << endl;
		return false;
	}
	if (centerCircle_Orange.x != 0 && centerCircle_Blue.x == 0)
	{
		objectCenter = centerCircle_Orange;
		return true;
	}

	if (centerCircle_Orange.x != 0 && centerCircle_Blue.x != 0)
	{

		objectCenter.x += centerCircle_Orange.x + centerCircle_Blue.x;
		objectCenter.y += centerCircle_Orange.y + centerCircle_Blue.y;
		objectCenter.x /= 2;
		objectCenter.y /= 2;
		return true;
	}
}

/**************************find_volleyball**************************************/
bool Vision::find_volleyball(Mat &imgOriginal, Point2f &objectCener)
{
	Point2f objectCenter_1;
	Point2f objectCenter_2;
	if (find_orange_red_volleyball(imgOriginal, objectCenter_1))
	{
		cout << "success to find_volleyball(orange_red)" << endl;
		cout << "橘红色排球的zhongxin:" << objectCener << endl;
		objectCener = objectCenter_1;
		return true;
	}
	else
	{
		cout << "fail to find_orange_red_volleyball,next_one" << endl;
		if (find_orange_blue_volleyball(imgOriginal, objectCenter_2))
		{
			cout << "success to find_volleyball(orange_blue)" << endl;
			objectCener = objectCenter_2;
			return true;
		}
		else
		{
			cout << "fail to find _all_volleyball" << endl;
			return false;
		}
	}
}
