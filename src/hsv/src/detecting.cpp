#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include "hsv/vision.h"
#include "const_msg/object_param.h"

using namespace std;
using namespace ros;
using namespace cv;

int main(int argc, char **argv)
{
	init(argc, argv, "detecting");
	Vision Process;
	Rate r(30);
	pair<double, long> finalobjectAngleDistance_mix_1(0.0, 0); //标定柱最后要发送的角度距离
	pair<double, long> finalobjectAngleDistance_mix_2(0.0, 0); //球最后要发送的角度距离
	const_msg::object_param calibration_param;//标定柱最后要发送的消息数据
	const_msg::object_param ball_param;//球最后要发送的消息数据

	//pair<double,long>objectAngleDistance_Y;
	VideoCapture capture_0(0); //找标定柱
	if (!capture_0.isOpened())
	{
		ROS_ERROR("fail to open camera_0 ");
		return -1;
	}

	VideoCapture capture_1(1); //找球
	if (!capture_1.isOpened())
	{
		ROS_ERROR("fail to open camera_1 ");
		return -1;
	}
	while (ros::ok())
	{	
		while (ros::ok())
		{
			Point2f objectCenter;
			vector<long> data_1 = laser_data_range;			//激光的距离
			vector<pair<double, long>> objectAngleDistance_1;		 //一组角度距离，激光找的参数
			pair<double, long> finalobjectAngleDistance_1(0.0, 0);	 //找标定柱，标定柱最终的角度距离
			pair<double, long> finalobjectAngleDistance_2(0.0, 0);	 //找球，球最终的角度距离
			pair<double, long> finalobjectAngleDistance_3(50.0, 50); //仅通过激光找，激光最后找到的角度距离
			//pair<double,long>finalobjectAngleDistance_3(0.0,0);//仅通过激光找，激光最后找到的角度距离
			if (!(int)object_type_target)
			{
				ROS_INFO_STREAM("object target type is:"<<object_type_target);
				//ROS_ERROR("fail to receive target  2");
			}
			else
			{
				ROS_INFO("success to receive target");
				ROS_INFO_STREAM("the target type is:" << object_type_target);
				switch (object_type_target)
				{
					std::cout << "接受物体的类型：" << object_type_target << endl;
				case (1 /*calibration*/):
				{
					Mat imgOriginal;
					bool bSuccess = capture_1.read(imgOriginal); //capture_1
					if (!bSuccess)
					{
						std::cout << "fail to get frame from camera" << endl;
						break;
					}
					//imshow("image1", imgOriginal);
					waitKey(0);
					if (!Process.find_calibration_by_LC(imgOriginal, objectCenter)) //判断是否是标定柱
					{																//参数二：摄像头判定标定柱的中心位置
						std::cout << "fail to find_big_block_blue in main example" << endl;
						finalobjectAngleDistance_1.first = 0.0;
						finalobjectAngleDistance_1.second = 0;
						
						Process.send_final_result(finalobjectAngleDistance_1);//发送物体角度距离

						/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_1))
						{
							std::cout << "fail to send_final_object_result in main example" << endl;
						}
						else
						{
							std::cout << "success to send_final_object_result in main example" << endl;
						}*/
					}
					else
					{
						std::cout << "success to find_calibration(blue_block) in main example" << endl;
						if (data_1.empty()) //激光是否接收到数据
						{
							std::cout << "fail to receive_laser_data in main example" << endl;
						}
						else
						{
							std::cout << "success to receive_laser_data in main example" << endl;
							if (!Process.find_calibration_object_by_laser_data(data_1, objectAngleDistance_1)) //找到标定柱后通过激光数据找标定柱位置
							{																				   //参数1：所有激光距离，参数二：找到标定柱中心的角度距离（很多个，向量）
								std::cout << "fail to find_object_by_laser_data_in_CALIBRATION_in main example" << endl;
								finalobjectAngleDistance_1.first = 0.0;
								finalobjectAngleDistance_1.second = 0;
								Process.send_final_result(finalobjectAngleDistance_1);//发送物体角度距离

								/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_1))
								{
									std::cout << "fail to send_final_object_result in main example" << endl;
								}
								else
								{
									std::cout << "success to send_final_object_result in main example" << endl;
								}*/
							}
							else
							{
								std::cout << "success to find_object_by_laser_data_IN_CALIBRATION_ in main example" << endl;
								if (!Process.find_final_object(objectAngleDistance_1, objectCenter, finalobjectAngleDistance_1)) //找到最终位置
								{
									std::cout << "fail to find_final_calibration in main example" << endl;
									finalobjectAngleDistance_1.first = 0.0;
									finalobjectAngleDistance_1.second = 0;

									Process.send_final_result(finalobjectAngleDistance_1);//发送物体角度距离
									/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_1))
									{
										std::cout << "fail to send_final_object_result in main example" << endl;
									}
									else
									{
										std::cout << "success to send_final_object_result in main example" << endl;
									}*/
								}
								else
								{
									std::cout << "success to find_final_calibration in main example" << endl;
								}
							}
						}
					}
				}
				break;
				case (2 /*reddish_brown_basketball*/):
				{ //改
					std::cout << "success to complete communication" << endl;
					Mat imgOriginal;
					bool bSuccess = capture_0.read(imgOriginal);
					if (!bSuccess)
					{
						std::cout << "fail to get frame from camera" << endl;
						break;
					}
					if (!Process.find_Reddish_brown_basketball(imgOriginal, objectCenter))
					{
						std::cout << "fail to find_Reddish_brown_block in main example" << endl;
						finalobjectAngleDistance_2.first = 0.0;
						finalobjectAngleDistance_2.second = 0;

						Process.send_final_result(finalobjectAngleDistance_2);//发送数据
						/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_2))
						{
							std::cout << "fail to send_final_object_result in main example" << endl;
						}
						else
						{
							std::cout << "success to send_final_object_result in main example" << endl;
						}*/
					}
					else
					{
						std::cout << "success to find_Reddish_brown_block in main example" << endl;
						if (data_1.empty())
						{
							std::cout << "fail to receive_laser_data_1 in main example" << endl;
						}
						else
						{
							std::cout << "success to receive_laser_data_1 in main example" << endl;
							if (!Process.find_nearest_object_by_laser_data(data_1, objectAngleDistance_1))
							{
								std::cout << "fail to find_nearest_object_by_laser_data_1 in main example" << endl;
							}
							else
							{
								std::cout << "success to find_nearest_object_by_laser_data_1 in main example" << endl;
								if (!Process.find_final_object(objectAngleDistance_1, objectCenter, finalobjectAngleDistance_2))
								{
									std::cout << "fail to find_final_Reddish_brown_basketball in main example" << endl;
									finalobjectAngleDistance_2.first = 0.0;
									finalobjectAngleDistance_2.second = 0;

									Process.send_final_result(finalobjectAngleDistance_2);//发送数据
									/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_2))
									{
										std::cout << "fail to send_final_object_result in main example" << endl;
									}
									else
									{
										std::cout << "success to send_final_object_result in main example" << endl;
									}*/
								}
								else
								{
									std::cout << "success to find_final_Reddish_brown_basketball in main example" << endl;
								}
							}
						}
					}
				}
				break;
				case (3 /*basketball*/):
				{
					if (data_1.empty())
					{
						std::cout << "fail to receive_laser_data" << endl;
					}
					else
					{
						std::cout << "success to receive_laser_data" << endl;
						if (!Process.find_nearest_object_by_laser_data(data_1, objectAngleDistance_1))
						{
							std::cout << "fail to find_nearest_object_by_laser_data" << endl;
							finalobjectAngleDistance_2.first = 0.0;
							finalobjectAngleDistance_2.second = 0;

							Process.send_final_result(finalobjectAngleDistance_2);//发送数据
							/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_2))
							{
								std::cout << "fail to send_final_object_result in main example" << endl;
							}
							else
							{
								std::cout << "success to send_final_object_result in main example" << endl;
							}*/
						}
						else
						{
							std::cout << "success to find_nearest_object_by_laser_data" << endl;
							Mat imgOriginal;
							bool bSuccess = capture_0.read(imgOriginal);
							if (!bSuccess)
							{
								std::cout << "fail to get a frame from camera" << endl;
								break;
							}
							if (!Process.find_basketball(imgOriginal, objectCenter))
							{
								std::cout << "fail to find all basketball" << endl;
								finalobjectAngleDistance_2.first = 0.0;
								finalobjectAngleDistance_2.second = 0;

								Process.send_final_result(finalobjectAngleDistance_2);//发送数据
								/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_2))
								{
									std::cout << "fail to send_final_object_result in main example" << endl;
								}
								else
								{
									std::cout << "success to send_final_object_result in main example" << endl;
								}*/
							}
							else
							{
								std::cout << "success to find basketball" << endl;
								if (!Process.find_final_object(objectAngleDistance_1, objectCenter, finalobjectAngleDistance_2))
								{
									std::cout << "fail to find_final_basketball" << endl;
									finalobjectAngleDistance_2.first = 0.0;
									finalobjectAngleDistance_2.second = 0;

									Process.send_final_result(finalobjectAngleDistance_2);//发送数据
									/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_2))
									{
										std::cout << "fail to send_final_object_result in main example" << endl;
									}
									else
									{
										std::cout << "success to send_final_object_result in main example" << endl;
									}*/
								}
								else
								{
									std::cout << "success to find_final_basketball" << endl;
								}
							}
						}
					}
				}
				break;
				case (4 /*blue_gray_basketball*/):
				{
					if (data_1.empty())
					{
						std::cout << "fail to receive_laser_data" << endl;
					}
					else
					{
						std::cout << "success to receive_laser_data" << endl;
						if (!Process.find_nearest_object_by_laser_data(data_1, objectAngleDistance_1))
						{
							std::cout << "fail to find_nearest_object_by_laser_data" << endl;
						}
						else
						{
							std::cout << "success to find_nearest_object_by_laser_data" << endl;
							Mat imgOriginal;
							bool bSuccess = capture_0.read(imgOriginal);
							if (!bSuccess)
							{
								std::cout << "fail to get a frame from camera" << endl;
								break;
							}
							if (!Process.find_blue_gray_basketball(imgOriginal, objectCenter))
							{
								std::cout << "fail to find all volleyball in main example" << endl;
								finalobjectAngleDistance_2.first = 0.0;
								finalobjectAngleDistance_2.second = 0;
								
								Process.send_final_result(finalobjectAngleDistance_2);//发送角度距离
								/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_2))
								{
									std::cout << "fail to send_final_object_result in main example" << endl;
								}
								else
								{
									std::cout << "success to send_final_object_result in main example" << endl;
								}*/
							}
							else
							{
								std::cout << "success to find volleyball in main example" << endl;
								if (!Process.find_final_object(objectAngleDistance_1, objectCenter, finalobjectAngleDistance_2))
								{
									std::cout << "fail to find_final_basketball in main example" << endl;
									finalobjectAngleDistance_2.first = 0.0;
									finalobjectAngleDistance_2.second = 0;
									
									Process.send_final_result(finalobjectAngleDistance_2);//发送角度距离
									/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_2))
									{
										std::cout << "fail to send_final_object_result in main example" << endl;
									}
									else
									{
										std::cout << "success to send_final_object_result in main example" << endl;
									}*/
								}
								else
								{
									std::cout << "success to find_final_basketball in main example" << endl;
								}
							}
						}
					}
				}
				break;
				case (5 /*calibration_plus*/):
				{
					Mat imgOriginal;
					bool bSuccess = capture_0.read(imgOriginal);
					if (!bSuccess)
					{
						std::cout << "fail to get frame from camera" << endl;
						break;
					}
					if (!Process.find_calibration_by_LC(imgOriginal, objectCenter))
					{
						std::cout << "fail to find_big_block_blue in main example" << endl;
						finalobjectAngleDistance_2.first = 0.0;
						finalobjectAngleDistance_2.second = 0;

						Process.send_final_result(finalobjectAngleDistance_2);//发送数据
						/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_2))
						{
							std::cout << "fail to send_final_object_result in main example" << endl;
						}
						else
						{
							std::cout << "success to send_final_object_result in main example" << endl;
						}*/
					}
					else
					{
						std::cout << "success to find_calibration(blue_block) in main example" << endl;
						if (data_1.empty())
						{
							std::cout << "fail to receive_laser_data in main example" << endl;
						}
						else
						{
							std::cout << "success to receive_laser_data in main example" << endl;
							if (!Process.find_calibration_object_by_laser_data(data_1, objectAngleDistance_1))
							{
								std::cout << "fail to find_object_by_laser_data in main example" << endl;
							}
							else
							{
								std::cout << "success to find_object_by_laser_data in main example" << endl;
								if (!Process.find_final_object_plus(objectAngleDistance_1, objectCenter, finalobjectAngleDistance_2))
								{
									std::cout << "fail to find_final_calibration in main example" << endl;
									finalobjectAngleDistance_2.first = 0.0;
									finalobjectAngleDistance_2.second = 0;

									Process.send_final_result(finalobjectAngleDistance_2);//发送数据
									/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_2))
									{
										std::cout << "fail to send_final_object_result in main example" << endl;
									}
									else
									{
										std::cout << "success to send_final_object_result in main example" << endl;
									}*/
								}
								else
								{
									std::cout << "success to find_final_calibration in main example" << endl;
								}
							}
						}
					}
				}
				break;
				case (6 /*orange_red_volleyball*/):
				{
					if (data_1.empty())
					{
						std::cout << "fail to receive_laser_data" << endl;
					}
					else
					{
						std::cout << "success to receive_laser_data" << endl;
						if (!Process.find_nearest_object_by_laser_data(data_1, objectAngleDistance_1))
						{
							std::cout << "fail to find_nearest_object_by_laser_data" << endl;
						}
						else
						{
							std::cout << "success to find_nearest_object_by_laser_data" << endl;
							Mat imgOriginal;
							bool bSuccess = capture_0.read(imgOriginal);
							if (!bSuccess)
							{
								std::cout << "fail to get a frame from camera" << endl;
								break;
							}
							if (!Process.find_orange_red_volleyball(imgOriginal, objectCenter))
							{
								std::cout << "fail to find all volleyball in main example" << endl;
								finalobjectAngleDistance_2.first = 0.0;
								finalobjectAngleDistance_2.second = 0;

								Process.send_final_result(finalobjectAngleDistance_2);//发送数据
								/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_2))
								{
									std::cout << "fail to send_final_object_result in main example" << endl;
								}
								else
								{
									std::cout << "success to send_final_object_result in main example" << endl;
								}*/
							}
							else
							{
								std::cout << "success to find volleyball in main example" << endl;
								if (!Process.find_final_object(objectAngleDistance_1, objectCenter, finalobjectAngleDistance_2))
								{
									std::cout << "fail to find_final_basketball in main example" << endl;
									finalobjectAngleDistance_2.first = 0.0;
									finalobjectAngleDistance_2.second = 0;

									Process.send_final_result(finalobjectAngleDistance_2);//发送数据
									/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_2))
									{
										std::cout << "fail to send_final_object_result in main example" << endl;
									}
									else
									{
										std::cout << "success to send_final_object_result in main example" << endl;
									}*/
								}
								else
								{
									std::cout << "success to find_final_basketball in main example" << endl;
								}
							}
						}
					}
				}
				break;
				case (7 /*orange_blue_volleyball*/):
				{
					if (data_1.empty())
					{
						std::cout << "fail to receive_laser_data" << endl;
					}
					else
					{
						std::cout << "success to receive_laser_data" << endl;
						if (!Process.find_nearest_object_by_laser_data(data_1, objectAngleDistance_1))
						{
							std::cout << "fail to find_nearest_object_by_laser_data  in main process" << endl;
						}
						else
						{
							std::cout << "success to find_nearest_object_by_laser_data in main process" << endl;
							Mat imgOriginal;
							bool bSuccess = capture_0.read(imgOriginal);
							if (!bSuccess)
							{
								std::cout << "fail to get a frame from camera" << endl;
								break;
							}
							if (!Process.find_orange_blue_volleyball(imgOriginal, objectCenter))
							{
								std::cout << "fail to find all volleyball in main example" << endl;
								finalobjectAngleDistance_2.first = 0.0;
								finalobjectAngleDistance_2.second = 0;

								Process.send_final_result(finalobjectAngleDistance_2);//发送数据
								/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_2))
								{
									std::cout << "fail to send_final_object_result in main example" << endl;
								}
								else
								{
									std::cout << "success to send_final_object_result in main example" << endl;
								}*/
							}
							else
							{
								std::cout << "success to find volleyball in main example" << endl;
								if (!Process.find_final_object(objectAngleDistance_1, objectCenter, finalobjectAngleDistance_2))
								{
									std::cout << "fail to find_final_basketball in main example" << endl;
									finalobjectAngleDistance_2.first = 0.0;
									finalobjectAngleDistance_2.second = 0;

									Process.send_final_result(finalobjectAngleDistance_2);//发送数据
									/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_2))
									{
										std::cout << "fail to send_final_object_result in main example" << endl;
									}
									else
									{
										std::cout << "success to send_final_object_result in main example" << endl;
									}*/
								}
								else
								{
									std::cout << "success to find_final_basketball in main example" << endl;
								}
							}
						}
					}
				}
				break;
				case (8 /*volleyball*/):
				{
					if (data_1.empty())
					{
						std::cout << "fail to receive_laser_data" << endl;
					}
					else
					{
						std::cout << "success to receive_laser_data" << endl;
						if (!Process.find_nearest_object_by_laser_data(data_1, objectAngleDistance_1))
						{
							std::cout << "fail to find_nearest_object_by_laser_data" << endl;
						}
						else
						{
							std::cout << "success to find_nearest_object_by_laser_data" << endl;
							Mat imgOriginal;
							bool bSuccess = capture_0.read(imgOriginal);
							if (!bSuccess)
							{
								std::cout << "fail to get a frame from camera" << endl;
								break;
							}
							if (!Process.find_volleyball(imgOriginal, objectCenter))
							{
								std::cout << "fail to find all volleyball in main example" << endl;
								finalobjectAngleDistance_2.first = 0.0;
								finalobjectAngleDistance_2.second = 0;

								Process.send_final_result(finalobjectAngleDistance_2);//发送数据
								/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_2))
								{
									std::cout << "fail to send_final_object_result in main example" << endl;
								}
								else
								{
									std::cout << "success to send_final_object_result in main example" << endl;
								}*/
							}
							else
							{
								std::cout << "success to find volleyball in main example" << endl;
								if (!Process.find_final_object(objectAngleDistance_1, objectCenter, finalobjectAngleDistance_2))
								{
									std::cout << "last______________1" << objectAngleDistance_1.size() << endl;
									std::cout << "物体的中心：______________" << objectCenter << endl;
									std::cout << "fail to find_final_basketball in main example" << endl;
									finalobjectAngleDistance_2.first = 0.0;
									finalobjectAngleDistance_2.second = 0;

									Process.send_final_result(finalobjectAngleDistance_2);//发送数据
									/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_2))
									{
										std::cout << "fail to send_final_object_result in main example" << endl;
									}
									else
									{
										std::cout << "success to send_final_object_result i_n main example" << endl;
									}*/
								}
								else
								{
									std::cout << "success to find_final_basketball in main example" << endl;
								}
							}
						}
					}
				}
				break;
				case (9 /*laser_ball_1*/):
				{
					/*while(true)
							{*/
				
					vector<pair<double, long>> objectAngleDistance_XX;
					if (data_1.empty())
					{
						std::cout << "fail to receive_laser_data" << endl;
					}
					else
					{
						std::cout << "success to receive_laser_data" << endl;
						if (!Process.find_nearest_object_by_laser_data_laser_ball(data_1, objectAngleDistance_XX))
						{
							std::cout << "fail to find_final_basketball in main example" << endl;
							finalobjectAngleDistance_3.first = 0.0;
							finalobjectAngleDistance_3.second = 0;

							Process.send_final_result(finalobjectAngleDistance_3);//发送数据
							/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_3))
							{
								std::cout << "fail to send_final_object_result in main example" << endl;
							}
							else
							{
								std::cout << "success to send_final_object_result i_n main example" << endl;
							}*/
							std::cout << "fail to find_nearest_object_by_laser_data" << endl;
						}
						else
						{
							std::cout << "success to find_nearest_object_by_laser_data" << endl;
							std::cout << "case9:激光找球时找到物体的个数：" << objectAngleDistance_XX.size() << endl;
							for (size_t i = 0; i < objectAngleDistance_XX.size(); i++)
							{
								std::cout << "激光找到的物体的角度：" << objectAngleDistance_XX[i].first
									 << "激光收到的物体的距离：" << objectAngleDistance_XX[i].second << endl;
							}
							//finalobjectAngleDistance_3.first=0.0;
							//finalobjectAngleDistance_3.second=0;
							vector<double> angle;
							vector<double> angle_abs;
							for (size_t i = 0; i < objectAngleDistance_XX.size(); i++)
							{
								angle.push_back(90 - objectAngleDistance_XX[i].first);
							}
							for (size_t i = 0; i < angle.size(); i++)
							{
								angle_abs.push_back(abs(angle[i]));
							}
							auto it = min_element(angle_abs.begin(), angle_abs.end());
							int min_Index = it - angle_abs.begin();
							finalobjectAngleDistance_3.first = 90 - angle[min_Index];
							finalobjectAngleDistance_3.second = objectAngleDistance_XX[min_Index].second;

							if (finalobjectAngleDistance_3.second <= 50) //小于等于50就跳出
							{
								finalobjectAngleDistance_3.first = 0;
								finalobjectAngleDistance_3.second = 0;
								break;
							}

							Process.send_final_result(finalobjectAngleDistance_3);//发送数据
							/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_3))
							{
								std::cout << "fail to send_final_object_result in main9 example" << endl;
							}
							else
							{
								std::cout << "success to send_final_object_result in main example" << endl;
								std::cout << "发送最后物体的角度：" << finalobjectAngleDistance_3.first
									 << "发送最后物体的距离：" << finalobjectAngleDistance_3.second
									 << "步数：" << (finalobjectAngleDistance_3.first / 0.25) + 180 << endl;
								//	}//当大于50时，没找到，就在第二个while(true)里循环。
							}*/
						}
					}
				}
				break;

				default:
				{
					finalobjectAngleDistance_2.first = 0;
					finalobjectAngleDistance_2.second = 0;

					Process.send_final_result(finalobjectAngleDistance_2);//发送数据
					/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_2))
					{
						std::cout << "fail to send_final_object_result in main example" << endl;
					}
					else
					{
						std::cout << "success to send_final_object_result in main example" << endl;
					}*/
				}
				break;
				} //switch完

				if (finalobjectAngleDistance_1.first != 0 && finalobjectAngleDistance_1.second != 0)
				{
					finalobjectAngleDistance_mix_1 = finalobjectAngleDistance_1;
					//finalobjectAngleDistance_mix=finalobjectAngleDistance_1;

					break;
				} //标定柱的
				if (finalobjectAngleDistance_2.first != 0 && finalobjectAngleDistance_2.second != 0)
				{
					finalobjectAngleDistance_mix_2 = finalobjectAngleDistance_2;
					//finalobjectAngleDistance_mix=finalobjectAngleDistance_2;

					break;
				} //球的
				/*std::cout<<"last:"<<finalobjectAngleDistance_3.first
				<<"last second:"<<finalobjectAngleDistance_3.second<<endl;*/
				std::cout << "跳出switch" << endl;
				if (finalobjectAngleDistance_3.first == 0 && finalobjectAngleDistance_3.second == 0) //跳出第二个while(true)
				{
					std::cout << "last:" << finalobjectAngleDistance_3.first
						 << "last second_--22:" << finalobjectAngleDistance_3.second << endl;
					
					Process.send_final_result(finalobjectAngleDistance_3);//发送数据
					/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_3))
					{
						std::cout << "fail to send_final_object_result in main9 example" << endl;
					}
					else
					{
						std::cout << "success to send_final_object_result（finally） in main example" << endl;
					}*/
					break;
				}
			}//switch前的else
			spinOnce();
			r.sleep();
		}//第二个while true,没有找到球或标定柱就在第二个while（true）里循环
		std::cout << "111111111111111111111111111111111111111" << endl;
		while (finalobjectAngleDistance_mix_1.first != 0 && finalobjectAngleDistance_mix_1.second != 0)
		{
			std::cout << "找到标定柱mix的角度：" << finalobjectAngleDistance_mix_1.first
				 << "找到标定柱mix的距离：" << finalobjectAngleDistance_mix_1.second << endl;
			pair<double, long> finalobjectAngleDistance_Y_1(0.0, 0);
			vector<long> data_2 = laser_data_range;
			vector<pair<double, long>> objectAngleDistance_2;
			if (data_2.empty())
			{
				std::cout << "fail to receive_laser_data_2 in main example" << endl;
			}
			else
			{
				std::cout << "success to receive_laser_data_2 in main example" << endl;
				if (!Process.find_calibration_object_by_laser_data(data_2, objectAngleDistance_2))
				{
					std::cout << "fail to find_nearest_object_by_laser_data in main example_again" << endl;
				}
				else
				{
					std::cout << "success to find_nearest_object_by_laser_data in main example_again" << endl;
					std::cout << "找到标定柱mix的角度：" << finalobjectAngleDistance_mix_1.first << endl; //第一次最终位置
					for (size_t i = 0; i < objectAngleDistance_2.size(); i++)
					{
						std::cout << "找到球后激光继续找到的满足的角度:" << objectAngleDistance_2[i].first
							 << "找到球后激光继续找到的满足的距离：" << objectAngleDistance_2[i].second << endl;
						if (abs(finalobjectAngleDistance_mix_1.first - objectAngleDistance_2[i].first) < 35) //？还要再比较？
						{																					 //照理Y应该有很多组呀？////////////
							finalobjectAngleDistance_Y_1.first = objectAngleDistance_2[i].first;
							finalobjectAngleDistance_Y_1.second = objectAngleDistance_2[i].second;
						}
					}
					std::cout << "发送物体后续的角度：" << finalobjectAngleDistance_Y_1.first
						 << "发送物体后续的距离：" << finalobjectAngleDistance_Y_1.second << endl;
					if (finalobjectAngleDistance_Y_1.second < 2460 && finalobjectAngleDistance_Y_1.second >= 2340) //继续while,2000要改
					{
						finalobjectAngleDistance_Y_1.second = -1;

						Process.send_final_result(finalobjectAngleDistance_Y_1);//发送数据
						/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_Y_1))
						{
							std::cout << "fail to send_final_object_result in main example" << endl;
						}
						else
						{
							std::cout << "success to send_final_object_result in main example" << endl;
							std::cout << "发送最后物体的角度：" << finalobjectAngleDistance_Y_1.first
								 << "发送最后物体的距离：" << finalobjectAngleDistance_Y_1.second << endl;
						}*/
						std::cout << "发送最后物体的角度：" << finalobjectAngleDistance_Y_1.first
								 << "发送最后物体的距离：" << finalobjectAngleDistance_Y_1.second << endl;
						finalobjectAngleDistance_mix_1.first = 0;
						finalobjectAngleDistance_mix_1.second = 0;
						break;
					}
					else
					{

						Process.send_final_result(finalobjectAngleDistance_Y_1);//发送数据
						/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_Y_1))
						{
							std::cout << "fail to send_final_object_result in main example" << endl;
						}
						else
						{
							std::cout << "success to send_final_object_result in main example" << endl;
							std::cout << "发送最后物体的角度：" << finalobjectAngleDistance_Y_1.first
								 << "发送最后物体的距离：" << finalobjectAngleDistance_Y_1.second << endl;
						}*/
						std::cout << "发送最后物体的角度：" << finalobjectAngleDistance_Y_1.first
								 << "发送最后物体的距离：" << finalobjectAngleDistance_Y_1.second << endl;
					}
				}
			}
		}
		std::cout << "22222222222222222222222222222222" << endl;
		int count = 0;
		while (finalobjectAngleDistance_mix_2.first != 0 && finalobjectAngleDistance_mix_2.second != 0)
		{
			std::cout << "找到球的mix角度：" << finalobjectAngleDistance_mix_2.first
				 << "找到球的mix距离：" << finalobjectAngleDistance_mix_2.second << endl;
			pair<double, long> finalobjectAngleDistance_Y_2(0.0, 0);
			vector<long> data_2;
			vector<pair<double, long>> objectAngleDistance_2; //找到球后，继续用激光找的满足的角度、距离
			vector<pair<double, long>> objectAngleDistance_2_plus;

			if (data_2.empty())
			{
				std::cout << "fail to receive_laser_data_2 in main example" << endl;
			}
			else
			{
				std::cout << "success to receive_laser_data_2 in main example" << endl;
				if (!Process.find_nearest_object_by_laser_data(data_2, objectAngleDistance_2))
				{
					std::cout << "fail to find_nearest_object_by_laser_data in main example_again" << endl;
					count += 1;
					if (count > 5)
					{
						break;
					}
					else
					{
						continue;
					}
				}
				else
				{
					std::cout << "success to find_nearest_object_by_laser_data in main example_again" << endl;
					for (size_t i = 0; i < objectAngleDistance_2.size(); i++)
					{
						std::cout << "激光找到球后物体后续的角度:" << objectAngleDistance_2[i].first
							 << "激光找到球后物体后续的距离：" << objectAngleDistance_2[i].second << endl;
						if (abs(finalobjectAngleDistance_mix_2.first - objectAngleDistance_2[i].first) < 30)
						{
							objectAngleDistance_2_plus.push_back(objectAngleDistance_2[i]);
						}
					}
					/*vector<double>distance;
							distance.push_back(objectAngleDistance_2[i].second);
							auto it=min_element(distance.begin(),distance.end());
							int min_Index=it-distance.begin();*/
					if (objectAngleDistance_2_plus.size() > 0)
					{
						pair<double, long> min_objectAngleDistance_2 = objectAngleDistance_2_plus[0];
						for (size_t i = 0; i < objectAngleDistance_2_plus.size(); i++)
						{
							if (objectAngleDistance_2_plus[i].second <= min_objectAngleDistance_2.second)
							{
								min_objectAngleDistance_2 = objectAngleDistance_2_plus[i];
							}
						}
						std::cout << "激光找到球后物体后续的角度:" << min_objectAngleDistance_2.first
							 << "激光找到球后物体后续的距离：" << min_objectAngleDistance_2.second << endl;
						std::cout << "找到球时mix的角度：" << finalobjectAngleDistance_mix_2.first << endl;
						std::cout << "mix和后续激光找的角度差值：" << abs(finalobjectAngleDistance_mix_2.first - min_objectAngleDistance_2.first) << endl;
						finalobjectAngleDistance_Y_2.first = min_objectAngleDistance_2.first;
						//finalobjectAngleDistance_Y_2.second=objectAngleDistance_2[min_Index].second;
						finalobjectAngleDistance_Y_2.second = min_objectAngleDistance_2.second;
					}
					else
					{
						std::cout << "差值大于30度" << endl;
					}
					std::cout << "发送物体的角度：" << finalobjectAngleDistance_Y_2.first
						 << "发送物体的距离：" << finalobjectAngleDistance_Y_2.second << endl;
					if (finalobjectAngleDistance_Y_2.second <= 50)
					{
						std::cout << "进入小于等于50后" << endl;
						finalobjectAngleDistance_mix_2.first = 0;
						finalobjectAngleDistance_mix_2.second = 0;
						finalobjectAngleDistance_Y_2.first = 0;
						finalobjectAngleDistance_Y_2.second = 0;
						break; //跳出当前while(true)
					}

					Process.send_final_result(finalobjectAngleDistance_Y_2);//发送数据
					/*if (!dataTransfer.send_final_object_result(finalobjectAngleDistance_Y_2))
					{
						std::cout << "fail to send_final_object_result in main example" << endl;
					}
					else
					{
						std::cout << "success to send_final_object_result in main example" << endl;
						std::cout << "发送最后物体的角度：" << finalobjectAngleDistance_Y_2.first
							 << "发送最后物体的距离：" << finalobjectAngleDistance_Y_2.second << endl;
						if (!dataTransfer.create_communication_RAM_4())
						{
							std::cout << "fail to create_communication_RAM_4" << endl;
						}
						if (!dataTransfer.open_file_mapping_4())
						{
							std::cout << "fail to open_file_mapping_4" << endl;
						}
					}*/
					std::cout << "发送最后物体的角度：" << finalobjectAngleDistance_Y_2.first
							 << "发送最后物体的距离：" << finalobjectAngleDistance_Y_2.second << endl;
				}
			}
		}
		spinOnce();
		r.sleep();
	}//找到后在第一个while(true)里循环

	return 0;
}
