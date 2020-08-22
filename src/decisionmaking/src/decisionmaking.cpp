#include <ros/ros.h>
#include "decisionmaking/decisionmaking.h"

using namespace std;
using namespace ros;

DecisionMaking::DecisionMaking(/* args */)
{
	detect_sub = nh.subscribe("/detect_result", 1, &DecisionMaking::detect_callback, this);
	//订阅感知部分传过来的数据
	serial_sub = nh.subscribe("/dsp_to_pc", 1, &DecisionMaking::serial_callback, this);
	//订阅底层部分传过来的数据
	detect_pub = nh.advertise<std_msgs::Int32>("/target_type", 1);
	//发布视觉感知命令
	serial_pub = nh.advertise<const_msg::pc_to_dsp>("/pc_to_dsp", 1);
	//发布底层控制命令

	dsp_status.fAngle = 0;
	dsp_status.XDist = 0;
	dsp_status.YDist = 0;

	//控制命令初始化
	Control.V1 = 0;
	Control.V2 = 0;
	Control.V3 = 0;
	Control.V4 = 0;
	Control.flag_start_stop = 1; //松开电机，允许运行

	//PidAngle控制变量初始化（包括p,i,d三个参数）
	PidAngle_init(2, 0, 0);
	PidDistX_init(2, 0, 0);
	PidDistY_init(2, 0, 0);

	//单片机允许发送指令标志
}

DecisionMaking::~DecisionMaking()
{
}

void DecisionMaking::detect_callback(const const_msg::object_paramConstPtr &obj_param)
{
	objectAngleDistance_Y.first = obj_param->obj_angle;
	objectAngleDistance_Y.second = obj_param->obj_distance;
}

void DecisionMaking::serial_callback(const const_msg::dsp_to_pcConstPtr &dsp_state)
{
	dsp_status.XDist = dsp_state->XDist;
	dsp_status.YDist = dsp_state->YDist;
	dsp_status.Vx = dsp_state->Vx;
	dsp_status.Vy = dsp_state->Vy;
	dsp_status.fAngle = dsp_state->fAngle;
	dsp_status.DW = dsp_state->dw;
	dsp_status.RecData_state = dsp_state->RecData_State;
	Strategy();
	control_pub(Control);
}

void DecisionMaking::object_pub(objectParameter &obj)
{
	std_msgs::Int32 object_msg;
	object_msg.data = (int)obj.whatObject;
	detect_pub.publish(object_msg);
}

void DecisionMaking::control_pub(DSPControl &ctrl)
{
	const_msg::pc_to_dsp control_msg;
	control_msg.V1 = ctrl.V1;
	control_msg.V2 = ctrl.V2;
	control_msg.V3 = ctrl.V3;
	control_msg.V4 = ctrl.V4;
	control_msg.flag_start_stop = ctrl.flag_start_stop;
	control_msg.SendData_State = ctrl.SendData_state;
	Strategy();
	serial_pub.publish(control_msg);
}

void DecisionMaking::Strategy()
{
	/*******等补充******读取控件Normal_num值********/
	//place_num=1;		//（场地号）1是向右出发，y取正值；2是向左出发，y取负值。
	//Normal_num=0;	//当通过mfc的界面控件选择控制程序（pass1-3或bat1-3）时，这句要屏蔽。

	//********************下面补充回合选择**********************
	switch (Normal_num) //Normal_num还未初始化，最终由控件控制
	{
	case 0:
		Normal_pass0();
		break;
	case 1:
		Normal_pass1();
		break;
	case 2:
		Normal_pass2();
		break;
	case 3:
		Normal_pass3();
		break;
	case 4:
		Normal_bat1();
		break;
	case 5:
		Normal_bat2();
		break;
	case 6:
		Normal_bat3();
		break;
	case 7:
		Normal_position();
		break;
	case 8:
		Normal_AtoB_test();
		break;
	case 9:
		Normal_Pttest();
		break;
	case 10:
		Normal_avoidance();
		break;
	case 11:
		Normal_return_test(); //new add
		break;
	case 12:
		Normal_pass1_new();
		break;
	case 13:
		Normal_pass2_new();
		break;
	case 14:
		Normal_pass3_new();
		break;
	case 15:
		Normal_bat1_new();
		break;
	case 16:
		Normal_bat2_new();
		break;
	case 17:
		Normal_bat3_new();
		break;
	case 18:
		Normal_bat0();
		break;
	default:
		break;
	}
}

bool DecisionMaking::find_object_by_laser(vector<long> &data, vector<pair<double, long>> &objectAngleDist)
{
	if (!data.empty())
	{
		for (size_t i = 0; i < data.size(); i++)
		{
			if (data[i] > 5000)
			{
				data[i] = 5000;
			}
		}
		int start = 0;
		int end = 0;
		for (size_t i = 60; i < 660 + 1; i++)
		{
			long dataSub = data[i + 1] - data[i];
			//if(dataSub<-300 && !start && data[i+1]<9000){
			if (dataSub < -300 && data[i + 2] < 4000)
			{
				start = i + 1; //get object start index
			}
			if (dataSub > 300 && start && data[i - 1] < 4000)
			{
				end = i; //get object end index
			}
			if (start && end)
			{
				if (end - start >= 4)
				{ //eliminate small object
					long centerDist = data[(int)((start + end) / 2)];
					pair<double, long> tempPair;
					tempPair.first = 90 - 0.25 * ((int)(start + end) / 2);
					tempPair.second = centerDist;
					objectAngleDist.push_back(tempPair);
				}
				start = 0; //initialize start and end value so as to find next object
				end = 0;
			}
		}
	}
	if (!objectAngleDist.empty())
	{
		return true;
	}
	else
	{
		return false;
	}
}

int DecisionMaking::find_object(vector<long> &data, vector<pair<double, long>> &objectInfo)
{
	vector<pair<double, long>> objectAngleDist;
	if (find_object_by_laser(data, objectAngleDist))
	{
		size_t counter = 0;
		for (size_t i = 0; i < objectAngleDist.size(); i++)
		{
			if (objectAngleDist[i].first >= 0)
			{
				counter++;
			}
		}
		if (counter > 0 && counter < objectAngleDist.size())
		{
			double maxAngle = 90;
			double minAngle = -90;
			pair<double, long> tempAngleDist;
			for (size_t i = 0; i < objectAngleDist.size(); i++)
			{
				if (objectAngleDist[i].first > 0 && objectAngleDist[i].first < maxAngle)
				{
					tempAngleDist.first = objectAngleDist[i].first;
					tempAngleDist.second = objectAngleDist[i].second;
					maxAngle = objectAngleDist[i].first;
				}
			}
			objectInfo.push_back(tempAngleDist);
			for (size_t i = 0; i < objectAngleDist.size(); i++)
			{
				if (objectAngleDist[i].first < 0 && objectAngleDist[i].first > minAngle)
				{
					tempAngleDist.first = objectAngleDist[i].first;
					tempAngleDist.second = objectAngleDist[i].second;
					minAngle = objectAngleDist[i].first;
				}
			}
			objectInfo.push_back(tempAngleDist);
			if (2 == objectInfo.size())
			{
				return 2;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			if (counter == objectAngleDist.size())
			{
				double maxAngle = 90;
				pair<double, long> tempAngleDist;
				for (size_t i = 0; i < objectAngleDist.size(); i++)
				{
					if (objectAngleDist[i].first < maxAngle)
					{
						tempAngleDist.first = objectAngleDist[i].first;
						tempAngleDist.second = objectAngleDist[i].second;
						maxAngle = objectAngleDist[i].first;
					}
				}
				objectInfo.push_back(tempAngleDist);
				return 1;
			}
			else
			{
				double minAngle = -90;
				pair<double, long> tempAngleDist;
				for (size_t i = 0; i < objectAngleDist.size(); i++)
				{
					if (objectAngleDist[i].first > minAngle)
					{
						tempAngleDist.first = objectAngleDist[i].first;
						tempAngleDist.second = objectAngleDist[i].second;
						minAngle = objectAngleDist[i].first;
					}
				}
				objectInfo.push_back(tempAngleDist);
				return -1;
			}
		}
	}
	else
	{
		return 0;
	}
}

void DecisionMaking::Normal_pass3_new()
{
	//一些决策相关的标志
	static int intention_num = 0;		  //回合过程的意图顺序号
	static bool findballflag = 0;		  //找到目标标志位,每找完一次目标后都要记得清零
	static bool aimballflag = 0;		  //对准目标标志位，每对准一次目标后都要记得清零
	static bool Isthreescoreunnormal = 0; //捡三分球是否出现异常
	static int normal_time = 0;			  //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0;	  //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;						  //DSP 45ms发送一次数据

	Posture ptOO, ptAA, ptA0, ptBB, ptCC, ptM, ptD;
	static Posture ptN = {0, 0, 0};

	ptA0.x = 1000;
	ptA0.y = 0;

	if (1 == place_num)
	{
		ptOO.x = 1200; //终点前，用于回位定点
		ptOO.y = 0;

		ptAA.x = 4050; //中点，扫描起点
		ptAA.y = 4750;

		ptM.x = 875;
		ptM.y = 4800; //投球目标点

		ptBB.x = 1575; //第二个分区开始扫描点，45度开始扫
		ptBB.y = 5600;

		ptCC.x = 800;
		ptCC.y = 7500; //边界点（临界主要用到X）

		ptD.x = 3000; //内圈扫描定点，位于三分线和内圈之间的一个辅助定位点，用于定点旋转找球
		ptD.y = 6700;
	}
	if (2 == place_num)
	{
		ptOO.x = 1600; //终点前，用于回位定点
		ptOO.y = 100;

		ptAA.x = 4050; //中点，扫描起点
		ptAA.y = -4750;

		ptM.x = 875;
		ptM.y = -4800; //投球目标点

		ptBB.x = 1575; //第二个分区开始扫描点，45度开始扫
		ptBB.y = -5600;

		ptCC.x = 800;
		ptCC.y = -7500; //边界点（临界主要用到X）

		ptD.x = 2700; //内圈扫描定点
		ptD.y = -6900;
	}

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 00000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		if ((normal_time - temp_normal_time) > 200) //延时大约9s
		{
			temp_normal_time = normal_time;
			intention_num = 1;
		}
		break;

	case 1: //从起点0到，转向、前往第一个三分捡球区点AA
		ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA0))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 100;
		}
		break;

	case 100:
		ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111111111111\n");
		if (ToAngleByMPU6050(ptAA))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 22222222222222222222222222222222222222222222222222222222222222222222222222222\n");
		if (ToPostureByMilemeter_Pid3(ptAA))
		{
			Stop();
			ROS_INFO_STREAM("走到AA点处！！！\n");
			temp_normal_time = normal_time;
			intention_num = 3;
		}
		break;

	case 3: //使车子成66度倾斜，为横向移动找三分球做准备
		ROS_INFO_STREAM("case 3333333333333333333333333333333333333333333333333333333333333333333333333\n");
		/*ROS_INFO_STREAM("正在旋转！！！\n");
		if(ToAngleByMPU(66))
		{
			ROS_INFO_STREAM("转到66度了！！！\n");
			if(stop())
			{
				ROS_INFO_STREAM("停稳！！！\n");
				temp_normal_time=normal_time;
				intention_num=4;
			}							
		}*/
		if (1 == place_num)
		{
			ROS_INFO_STREAM("角度调整过程ing................\n");
			if (fabs(dsp_status.fAngle - 66.00) > 2)
			{
				ROS_INFO_STREAM("和66度相差大于2度................\n");
				if (dsp_status.fAngle > 68)
				{
					ROS_INFO_STREAM("角度为正，需要左旋................\n");
					ToFindObjectByturnleft();
				}
				if (dsp_status.fAngle < 64)
				{
					ROS_INFO_STREAM("角度为负，需要右旋................\n");
					ToFindObjectByturnright();
				}
			}
			else
			{
				if (stop())
				{
					ROS_INFO_STREAM("角度为66度，不要动了！！！................\n");
					temp_normal_time = normal_time;
					intention_num = 4;
				}
			}
		}
		if (2 == place_num)
		{
			ROS_INFO_STREAM("角度调整过程ing................\n");
			if (fabs(dsp_status.fAngle - (-66.00)) > 2)
			{
				ROS_INFO_STREAM("和66度相差大于2度................\n");
				if (dsp_status.fAngle > -64)
				{
					ROS_INFO_STREAM("角度为正，需要左旋................\n");
					ToFindObjectByturnleft();
				}
				if (dsp_status.fAngle < -68)
				{
					ROS_INFO_STREAM("角度为负，需要右旋................\n");
					ToFindObjectByturnright();
				}
			}
			else
			{
				if (stop())
				{
					ROS_INFO_STREAM("角度为-66度，不要动了！！！................\n");
					temp_normal_time = normal_time;
					intention_num = 4;
				}
			}
		}

		break;

	case 4:
		ROS_INFO_STREAM("case 44444444444444444444444444444444444444444444444444444444444444444444444444444\n");
		if ((normal_time - temp_normal_time) > 5)
		{
			//TODO:接受发送数据
			/*if(!Vision.create_communication_RAM_2())
			{
				ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
				if(!Vision.open_file_mapping_2())
				{
					ROS_INFO_STREAM("fail to open_file_mapping_2\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to open_file_mapping_2\n");
					if(!Vision.create_communication_RAM_4())
					{
						ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
						if(!Vision.open_file_mapping_4())
						{
							ROS_INFO_STREAM("fail to open_file_mapping_4\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to open_file_mapping_4\n");
							TheObject.whatObject =(objectType)3;//找什么东西在这里改
							if(!Vision.send_find_whatobject(TheObject.whatObject))
							{
								ROS_INFO_STREAM("Fail to send command\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to send command\n");
								TheObject.whatObject = (objectType)0;//unkownObject变量重新初始化，为下一次找球做准备
								if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
								{
									ROS_INFO_STREAM("Fail to receive final object result\n");
								}
								else
								{
									ROS_INFO_STREAM("Success to receive final object result\n");
									ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
									ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
								}
							}
						}
					}
				}
			}*/

			if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
			{
				findballflag = 0;
				aimballflag = 0;
				if (1 == place_num)
				{
					if (abs((dsp_status.XDist) - (ptCC.x)) < 50)
					{
						Stop();
						ROS_INFO_STREAM("已经到边界了，但啥也没找着，算了，去找内圈球吧！！！\n");

						ptN.x = dsp_status.XDist; //把当前坐标赋给点N
						ptN.y = dsp_status.YDist;
						ROS_INFO("点N的坐标为：%ld , %ld\n", ptN.x, ptN.y);

						temp_normal_time = normal_time;
						intention_num = 103;
					}
					else
					{
						ROS_INFO_STREAM("以66度方向行进...\n");
						robotstraightrightlow(); //继续右走
					}
				}
				if (2 == place_num)
				{
					if (abs((dsp_status.XDist) - (ptCC.x)) < 50)
					{
						Stop();
						ROS_INFO_STREAM("已经到边界了，但啥也没找着，算了，去找内圈球吧！！！\n");

						ptN.x = dsp_status.XDist; //把当前坐标赋给点N
						ptN.y = dsp_status.YDist;
						ROS_INFO("点N的坐标为：%ld , %ld\n", ptN.x, ptN.y);

						temp_normal_time = normal_time;
						intention_num = 103;
					}
					else
					{
						ROS_INFO_STREAM("以-66度方向行进...\n");
						robotstraightleftlow(); //继续左走
					}
				}
			}
			else
			{
				findballflag = 1;
				ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
			}
			if ((findballflag == 1) && (aimballflag == 0))
			{
				ROS_INFO_STREAM(" cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
				if (ToAimBallByVision())
				{
					ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //对准球的过程中突然丢了
						findballflag = 0;
						aimballflag = 0;
						ROS_INFO("丢了 丢了 丢了 ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 4;
						}
					}
					else
					{
						aimballflag = 1;
						ROS_INFO("!!!!!!!!三分线球:objectAngle aimedaimedaimed is %lf;三分线球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					}
				}
			}
			if ((findballflag == 1) && (aimballflag == 1))
			{
				ROS_INFO_STREAM(" ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n");
				if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
				{
					ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //前往球的跟前过程中，又没看到球，重新去找
						aimballflag = 0;
						findballflag = 0;
						ROS_INFO_STREAM(" 又丢了 又丢了 又丢了 ！！！\n");
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 4;
						}
					}
					else
					{
						aimballflag = 0;
						findballflag = 0;
						intention_num = 5;
						ROS_INFO_STREAM("捡球 捡球 捡球 ！！！\n");
					}
				}
			}
		}
		break;

	case 5:
		ROS_INFO_STREAM("case 55555555555555555555555555555555555555555555555555555555555555555555555555555\n");
		Stop();
		//m_bSend = 1;				//TODO:允许给单片机发送命令
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令
		ROS_INFO_STREAM("case uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuppppppppppppppppppppppppppppppppppppp\n");
		temp_normal_time = normal_time;
		intention_num = 6; //抬机械臂命令已发送
		break;

	case 6: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 66666666666666666666666666666666666666666666666666666666666666666666666666666\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 40)
		{
			ROS_INFO_STREAM("机械臂抬起，等待延时...\n");
			temp_normal_time = normal_time;
			intention_num = 7;
		}
		break;

	case 7:
		ROS_INFO_STREAM("case 77777777777777777777777777777777777777777777777777777777777777777777777777777\n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中发第一次判断命令
		{
			//m_bSend = 1;				//TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			ROS_INFO_STREAM("第一次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 8;
		}
		break;

	case 8:
		ROS_INFO_STREAM("case 88888888888888888888888888888888888888888888888888888888888888888888888888888\n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			temp_normal_time = normal_time;
			intention_num = 80;
			ROS_INFO_STREAM("发现已经捡到球了，架子上有球！！！\n");
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			temp_normal_time = normal_time;
			intention_num = 81;
			ROS_INFO_STREAM("判断没有球啊！等待抬臂稳定后在判断一遍！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 80:
		ROS_INFO_STREAM("case 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80\n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 101;
		}
		break;

	case 81:
		ROS_INFO_STREAM("case 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81\n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			//m_bSend = 1;				//TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("第二次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 82;
		}
		break;

	case 82:
		ROS_INFO_STREAM("case 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82\n");
		Control.SendData_state = 0; //单片机位置0
		ROS_INFO_STREAM("第二次，判断架子上有没有球！！！！\n");
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("发现有球啊！！！第一次误判！！！\n");
			temp_normal_time = normal_time;
			intention_num = 101;
		}
		else //判断两次都没球，可以再去找了
		{
			findballflag = 0;
			aimballflag = 0;
			intention_num = 4;
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 101: //瞄准，准备发射篮球
		ROS_INFO_STREAM("case 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101\n");
		ROS_INFO_STREAM("正在对准传球目标点！！！！\n");
		Control.SendData_state = 6; //关闭里程计
		if (ToAngleByMPU6050(ptM))
		{
			ROS_INFO_STREAM("已经对准了传球目标点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，准备弹射！！！！\n");
				Control.SendData_state = 7; //旋转停稳之后，再打开里程计
				temp_normal_time = normal_time;
				intention_num = 102;
			}
		}
		break;

	case 102:
		ROS_INFO_STREAM("case 102 102 102 102 102 102 102 102 102 102 102 102 102 102 102 102 \n");
		Control.SendData_state = 0;
		if ((normal_time - temp_normal_time) > 2)
		{
			ptN.x = dsp_status.XDist; //把当前坐标赋给点N
			ptN.y = dsp_status.YDist;
			ROS_INFO("点N的坐标为：%ld , %ld\n", ptN.x, ptN.y);

			if (dsp_status.XDist > 3300)
			{
				Control.SendData_state = 2; //发送发射指令，下一步立即置位0 //gaile gaile
				ROS_INFO_STREAM("发射 发射 发射 ！！！................\n");
				temp_normal_time = normal_time;
				intention_num = 103;
			}
			else
			{
				Control.SendData_state = 4; //发送发射指令，下一步立即置位0 //gaile gaile
				ROS_INFO_STREAM("发射 发射 发射 ！！！................\n");
				temp_normal_time = normal_time;
				intention_num = 103;
			}
		}
		break;

	case 103: //再转回来，对准内圈定位点
		ROS_INFO_STREAM("case 103 103 103 103 103 103 103 103 103 103 103 103 103 103 103 103 \n");
		ROS_INFO_STREAM("正在对准内圈定位点！！！！\n");
		Control.SendData_state = 0;
		Control.SendData_state = 6; //关闭里程计
		if (ToAngleByMPU6050(ptD))
		{
			ROS_INFO_STREAM("已经对准了内圈定位点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，下一步要往前走！！！！\n");
				Control.SendData_state = 7; //打开里程计
				temp_normal_time = normal_time;
				intention_num = 104;
			}
		}
		break;

	case 104:
		ROS_INFO_STREAM("case 104 104 104 104 104 104 104 104 104 104 104 104 104 104 104 104 104 104 \n");
		temp_normal_time = normal_time;
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid4(ptD)) //到达D点
		{
			Stop();
			ROS_INFO_STREAM("到 DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD啦啦啦啦啦\n");
			intention_num = 105;
		}
		break;

		/***********************************  旋转识球  ***********************************/
	case 105:
		ROS_INFO_STREAM("case 105 105 105 105 105 105 105 105 105 105 105 105 105 105 105 105 105 105 \n");
		temp_normal_time = normal_time; //为延时作准备
		//TODO:发送接受数据
		/*if (!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if (!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if (!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if (!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");

						//视觉修改版找球测试start
						TheObject.whatObject = (objectType)3; //找什么东西在这里改
						if (TheObject.whatObject == (objectType)1)
							ROS_INFO_STREAM("send command to find calibration\n");
						if (TheObject.whatObject == (objectType)2)
							ROS_INFO_STREAM("send command to find reddish_brown_basketball\n");
						if (TheObject.whatObject == (objectType)3)
							ROS_INFO_STREAM("send command to find basketball\n");
						if (TheObject.whatObject == (objectType)4)
							ROS_INFO_STREAM("send command to find blue_gray_basketball\n");
						if (TheObject.whatObject == (objectType)5)
							ROS_INFO_STREAM("send command to find calibration_plus\n");
						if (TheObject.whatObject == (objectType)6)
							ROS_INFO_STREAM("send command to find orange_red_volleyball\n");
						if (TheObject.whatObject == (objectType)7)
							ROS_INFO_STREAM("send command to find red_blue_volleyball\n");
						if (TheObject.whatObject == (objectType)8)
							ROS_INFO_STREAM("send command to find volleyball\n");

						if (!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM(" Success to send command Success to send command Success to send command Success to send command\n");
							TheObject.whatObject = (objectType)0 ; //unkownObject变量重新初始化，为下一次找球做准备
							if (Vision.receive_final_object_result(objectAngleDistance_Y) == false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n", objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n", objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}*/
		if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				//Control.SendData_state=6;	//关闭里程计
				ToFindObjectByturnright(); //右旋找球测试（需要测试旋转找球时请将该函数释放）
				ROS_INFO_STREAM(" 我在右转 我在右转 我在右转 我在右转 我在右转 我在右转 我在右转 我在右转 我在右转 我在右转 \n");
				//robotstraightrightlow();	//右移找球测试
				//ROS_INFO_STREAM(" 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 \n");
			}
			if (2 == place_num)
			{
				//Control.SendData_state=6;	//关闭里程计
				ToFindObjectByturnleft(); //右旋找球测试（需要测试旋转找球时请将该函数释放）
				ROS_INFO_STREAM(" 我在右转 我在右转 我在右转 我在右转 我在右转 我在右转 我在右转 我在右转 我在右转 我在右转 \n");
				//robotstraightrightlow();	//右移找球测试
				//ROS_INFO_STREAM(" 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 \n");
			}
		}
		else
		{
			//Control.SendData_state=7;	//打开里程计
			findballflag = 1;
			ROS_INFO_STREAM(" findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			//Control.SendData_state=6;	//关闭里程计
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					ROS_INFO("丢了 丢了 丢了 : %lf \n", objectAngleDistance_Y.first);
					if ((normal_time - temp_normal_time) > 2)
					{
						temp_normal_time = normal_time; //为延时作准备
						intention_num = 105;
					}
				}
				else
				{
					//Control.SendData_state=7;	//打开里程计
					aimballflag = 1;
					ROS_INFO("!!!!!!!!中圈球:objectAngle aimedaimedaimed is %lf;中圈球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1))
		{
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("又丢了 又丢了 又丢了 \n");
					if ((normal_time - temp_normal_time) > 2)
					{
						temp_normal_time = normal_time; //为延时作准备
						intention_num = 105;
					}
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					intention_num = 106;
					ROS_INFO_STREAM("捡球 捡球 捡球 \n");
				}
			}
		}
		break;

	case 106:
		ROS_INFO_STREAM("case 106 106 106 106 106 106 106 106 106 106 106 106 106 106 106 106 106 106 106 \n");
		//立即捡球，不要延时
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		ROS_INFO_STREAM("发送抬臂指令！！！");
		temp_normal_time = normal_time;
		intention_num = 107; //抬机械臂命令已发送
		break;

	case 107: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 107 107 107 107 107 107 107 107 107 107 107 107 107 107 107 107 107 107 107 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 40)
		{
			temp_normal_time = normal_time;
			intention_num = 108;
		}
		break;

	case 108:
		ROS_INFO_STREAM("case 108 108 108 108 108 108 108 108 108 108 108 108 108 108 108 108 108 108 108 \n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中就发第一次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			ROS_INFO_STREAM("第一次，发送捡球判断！！！");
			temp_normal_time = normal_time;
			intention_num = 109;
		}
		break;

	case 109:
		ROS_INFO_STREAM("case 109 109 109 109 109 109 109 109 109 109 109 109 109 109 109 109 109 109 109 \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			ROS_INFO_STREAM("检测到有球！！！");
			temp_normal_time = normal_time;
			intention_num = 110;
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			ROS_INFO_STREAM("没检测到有球，在判断一次！！！");
			temp_normal_time = normal_time;
			intention_num = 111;
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 110:
		ROS_INFO_STREAM("case 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 \n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 114;
		}
		break;

	case 111:
		ROS_INFO_STREAM("case 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 \n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("第二次，发送捡球判断！！！");
			temp_normal_time = normal_time;
			intention_num = 113;
		}
		break;

	case 113:
		ROS_INFO_STREAM("case 113 113 113 113 113 113 113 113 113 113 113 113 113 113 113 113 113 113 113 \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("第一次为误判，明明有球！！！");
			temp_normal_time = normal_time;
			intention_num = 114;
		}
		else //判断两次都没球，可以再去找了
		{
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！");
			findballflag = 0;
			aimballflag = 0;
			intention_num = 105;
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 114:
		ROS_INFO_STREAM(" 114 114 114 114 114 114 114 114 114 114 114 114 114 114 114 114 114 114 114 114 \n");
		Control.SendData_state = 6; //旋转时关闭里程计
		ROS_INFO_STREAM("正在对准投球定位点N ！！！！\n");
		ROS_INFO("点N的坐标为：%ld , %ld\n", ptN.x, ptN.y);
		if (ToAngleByMPU6050(ptN))
		{
			ROS_INFO_STREAM("已经对准了投球定位点N ！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，下一步要往前走！！！！\n");
				Control.SendData_state = 7; //打开里程计
				temp_normal_time = normal_time;
				intention_num = 115;
			}
		}
		break;

	case 115:
		ROS_INFO_STREAM("case 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 \n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid4(ptN)) //到达N点
		{
			Stop();
			ROS_INFO_STREAM("到 NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN啦啦啦啦啦\n");
			temp_normal_time = normal_time;
			intention_num = 1150;
		}
		break;

	case 1150:
		ROS_INFO_STREAM("case 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 \n");
		if ((normal_time - temp_normal_time) < 20) //以开环速度移动1.5s
		{
			ROS_INFO_STREAM("开环走2秒...\n");
			robotforward();
		}
		else
		{
			Stop();
			ROS_INFO_STREAM("走到2秒处！！！\n");
			temp_normal_time = normal_time;
			intention_num = 116;
		}
		break;

	case 116: //瞄准，准备发射篮球
		ROS_INFO_STREAM("case 116 116 116 116 116 116 116 116 116 116 116 116 116 116 116 116 116 116 116 \n");
		Control.SendData_state = 6; //旋转时关闭里程计
		ROS_INFO_STREAM("正在对准传球目标点！！！！\n");
		if (ToAngleByMPU6050(ptM))
		{
			ROS_INFO_STREAM("已经对准了传球目标点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，准备弹射！！！！\n");
				Control.SendData_state = 7; //打开里程计
				temp_normal_time = normal_time;
				intention_num = 117;
			}
		}
		break;

	case 117:
		ROS_INFO_STREAM("case 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 \n");
		Control.SendData_state = 0;
		if ((normal_time - temp_normal_time) > 2)
		{
			if (dsp_status.XDist > 3300)
			{
				Control.SendData_state = 2; //发送发射指令，下一步立即置位0 //gaile gaile
				ROS_INFO_STREAM("发射 发射 发射 ！！！................\n");
				temp_normal_time = normal_time;
				intention_num = 118;
			}
			else
			{
				Control.SendData_state = 4; //发送发射指令，下一步立即置位0 //gaile gaile
				ROS_INFO_STREAM("发射 发射 发射 ！！！................\n");
				temp_normal_time = normal_time;
				intention_num = 118;
			}
		}
		break;

	case 118: //**********车子归位开始**********
		ROS_INFO_STREAM("case 118 118 118 118 118 118 118 118 118 118 118 118 118 118 118 118 118 118 118 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 2)
		{
			ROS_INFO_STREAM("在去00点途中 ！！！！\n");
			if (ToPostureByMilemeter_Pid2(ptOO))
			{
				ROS_INFO_STREAM("已经到达00点！！！！\n");
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 220;
				}
			}
		}
		break;

	case 220:
		ROS_INFO_STREAM("case 220 220 220 220 220 220 220 220 220 220 220 220 220 220 220 220 220 220 220 \n");
		if (fabs(dsp_status.fAngle) > 2.5)
		{
			Control.SendData_state = 6; //旋转时关闭里程计
			ROS_INFO("当前角度为：%lf \n", dsp_status.fAngle);
			ToAngleByMPU(0);
		}
		else
		{
			ROS_INFO("调整之后角度为：%lf \n", dsp_status.fAngle);
			Stop();
			Control.SendData_state = 7; //打开里程计
			intention_num = 330;
		}
		break;

	case 330:
		ROS_INFO_STREAM("case 330 330 330 330 330 330 330 330 330 330\n");
		Control.SendData_state = 0;
		if (abs(dsp_status.YDist) > 30)
		{
			ROS_INFO("当前坐标为：%ld, %ld \n", dsp_status.XDist, dsp_status.YDist);
			if (dsp_status.YDist > 0)
			{
				ROS_INFO_STREAM("左直走 ！！！！\n");
				robotstraightleft();
			}
			else
			{
				ROS_INFO_STREAM("右直走 ！！！！\n");
				robotstraightright();
			}
		}
		else
		{
			Stop();
			intention_num = 440;
		}
		break;

	case 440:
		ROS_INFO_STREAM("case 440 440 440 440 440 440 440 440 440 440 440 440 440 440 440 440\n");
		Control.SendData_state = 6;
		if (ToAngleByMPU(-6))
		{
			ROS_INFO_STREAM("停 ！！！ \n");
			Control.SendData_state = 7;
			Stop();
			intention_num = 550;
		}
		/*if (fabs(dsp_status.fAngle)>1.5)
		{
			ROS_INFO_STREAM("当前角度为：%lf \n",dsp_status.fAngle );
			if (dsp_status.fAngle>0)
			{
				ROS_INFO_STREAM("左转！！！ \n");
				ToFindObjectByturnleftlow();
			}
			else
			{
				ROS_INFO_STREAM("右转！！！ \n");
				ToFindObjectByturnrightlow();
			}
		}
		else
		{
			ROS_INFO_STREAM("停 ！！！ \n");
			Stop();
			intention_num=550;
		}*/
		break;

	case 550:
		ROS_INFO_STREAM("case 550 550 550 550 550 550 550 550 550 550 550 550 550 550 550 550\n");
		Control.SendData_state = 0;
		if (1 == place_num)
		{
			if (dsp_status.XDist > (-50))
			{
				ROS_INFO_STREAM("后退 ！！！ \n");
				robotbacklow();
			}
			else
			{
				Stop();
				intention_num = 660;
			}
		}
		if (2 == place_num)
		{
			if (dsp_status.XDist > 500)
			{
				ROS_INFO_STREAM("后退 ！！！ \n");
				robotbacklow();
			}
			else
			{
				Stop();
				intention_num = 660;
			}
		}
		break;

	case 660:
		ROS_INFO_STREAM("case 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23\n");
		Stop(); //停车
		break;

	default:
		normal_time = 0;
		break;
		//**********车子归位结束**********
	}
}

void DecisionMaking::Normal_position() //直行测试
{
	short Vx, Vy, W;
	float jd;
	Vx = 450;
	Vy = 0;
	W = 0;
	jd = 0;
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
}

void DecisionMaking::Normal_avoidance() //避障测试
{
	static int intention_num = 0;	 //回合过程的意图顺序号
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptM, ptC;
	ptM.x = 5000;
	ptM.y = 0;
	ptC.x = 3200; //用G走折线时坐标2175	//捡三分球30度位置（4115,3940）		//捡三分球的中间位置点的坐标（2365,5120）（45度三分线前与三分线原点距离7300）
	ptC.y = 3200; //用G走折线时坐标4930

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 00000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;

	case 120:
		if ((normal_time - temp_normal_time) > 150) //延时17.1s
		{
			temp_normal_time = normal_time;
			intention_num = 8;
		}
		break;

	case 8: //转向目的点，去找三分球
		ROS_INFO_STREAM("case 88888888888888888888888888888888888888888888888888888888888888888888888888888\n");
		Control.SendData_state = 0; //单片机位置0
		if (ToAngleByMPU6050(ptC))
		{
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 81;
			}
		}
		break;

	case 81:
		ROS_INFO_STREAM("case 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81\n");
		if (ToPostureByAvoidance(ptC)) //避障
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 810;
		}
		if ((normal_time - temp_normal_time) > 180)
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 82;
		}
		break;

	case 810:
		ROS_INFO_STREAM("case 810 810 810 810 810 810 810 810 810 810 810 810 810 810 810 810\n");
		if ((normal_time - temp_normal_time) < 10) //以开环速度向前2s
		{
			robotforward();
		}
		else
		{
			temp_normal_time = normal_time;
			intention_num = 82;
		}
		break;

	case 82: //避障后再次对准目标点
		ROS_INFO_STREAM("case 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (ToAngleByMPU6050(ptC))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 9;
				}
			}
		}
		break;

	case 9:
		ROS_INFO_STREAM("case 999999999999999999999999999999999999999999999999999999999999999999999999\n");
		if (ToPostureByMilemeter(ptC))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 10;
		}
		break;

	case 10:
		Stop();
		break;

	default:
		break;
	}

	/*if(ToPostureByAvoidance(ptM))
	{
		Stop();
		ROS_INFO_STREAM(" ToPostureByAvoidance(ptM) ToPostureByAvoidance(ptM)ToPostureByAvoidance(ptM)\n");
	}*/

	Control.flag_start_stop = 1; //允许电机转动
}

void DecisionMaking::Normal_return_test() //归位测试
{
	static int intention_num = 0;	 //回合过程的意图顺序号
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptOO, ptA, ptB;

	ptOO.x = 1000.0;
	ptOO.y = 0.0; //回合结束前，用于回位定点

	ptA.x = 3000.0;
	ptA.y = 0.0; //A点

	ptB.x = 3000.0;
	ptB.y = 3000.0; //B点：由A→B需要：右转90度

	//设计两种归位方式：1、点对点直接归位，不掉头； 2、掉头归位，中途避障
	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;

	case 120:
		if ((normal_time - temp_normal_time) > 45) //延时2s
		{
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA)) //到达A点
		{
			Stop();
			ROS_INFO_STREAM("到 AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA啦啦啦啦啦\n");
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 2222222222222222222222222222222222222222222222222222222222222222222222222222\n");
		if ((normal_time - temp_normal_time) > 10)
		{
			temp_normal_time = normal_time;
			if (ToAngleByMPU(90)) //旋转90度
			{
				Stop();
				ROS_INFO_STREAM("转 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 啦啦啦啦啦\n");
				intention_num = 3;
			}
		}
		break;

	case 3:
		ROS_INFO_STREAM("case 3333333333333333333333333333333333333333333333333333333333333333333333333333\n");
		if ((normal_time - temp_normal_time) > 10)
		{
			temp_normal_time = normal_time;
			if (ToPostureByMilemeter_Pid2(ptB)) //到达B点
			{
				Stop();
				ROS_INFO_STREAM("到 BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB啦啦啦啦啦\n");
				intention_num = 11;
			}
		}
		break;

	case 11: //旋转对准pt00点
		ROS_INFO_STREAM("case 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 3)
		{
			if (ToPostureByMilemeter_Pid2(ptOO)) //边走边调整角度先回位OO点，再微调退回框
			//if (ToAngleForHomeByMPU6050())//走折线回位
			//if(ToAngleByMPU6050(ptOO))//先对准OO，再去OO，然后微调回位
			{
				ROS_INFO_STREAM("到 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 啦啦啦啦啦\n");
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 12;
				}
			}
		}
		break;

		//case 1200:

		//    if (fabs(dsp_status.fAngle)>2.0)
		//	{
		//		ToAngleByMPU(0);
		//	}
		//	else
		//	{
		//		Stop();
		//		temp_normal_time=normal_time;
		//		intention_num=121;
		//	}
		//	break;

	case 12:
		ROS_INFO_STREAM("case 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12\n");
		ROS_INFO("入库时角度大于2 ！！！需要旋转调整！！！jiaodu jiaodu before ...................:%lf\n", dsp_status.fAngle);
		dsp_status.fAngle =DecisionMaking::mth_ChangeAngle(dsp_status.fAngle);
		ROS_INFO("入库时角度大于2 ！！！需要旋转调整！！！jiaodu jiaodu after ...................:%lf\n", dsp_status.fAngle);
		if ((dsp_status.fAngle > 2.0) || (dsp_status.fAngle < -2.0)) // m_DSPComm.m_dsp_status.fAngle
		{
			/*dsp_status.fAngle = DecisionMaking::mth_ChangeAngle(dsp_status.fAngle);*/
			/*ROS_INFO_STREAM("入库时角度大于2 ！！！需要旋转调整！！！jiaodu jiaodu before ...................:%lf\n",dsp_status.fAngle);
			dsp_status.fAngle = DecisionMaking::mth_ChangeAngle(dsp_status.fAngle);
			ROS_INFO_STREAM("入库时角度大于2 ！！！需要旋转调整！！！jiaodu jiaodu after ...................:%lf\n",dsp_status.fAngle);*/
			if (dsp_status.fAngle > 0.0)
			{
				ROS_INFO_STREAM("turn left ... turn left ... turn left ... turn left ... turn left ... \n");
				//ToAngleByMPU(0);
				ToFindObjectByturnleft();
			}
			else
			{
				ROS_INFO_STREAM("turn right ... turn right ... turn right ... turn right ... turn right ... \n");
				ToFindObjectByturnright();
			}
		}
		else
		{
			ROS_INFO("角度对准了，为0了！！！jiaodu jiaodu .....................:%lf\n", dsp_status.fAngle); //就这里，总是为0
			Stop();
			intention_num = 121;
		}
		break;

	case 121:
		ROS_INFO_STREAM("case 121 121 121 121 121 121 121 121 121 121 121 121 121 121 121 121\n");
		if (abs(dsp_status.YDist) >= 30)
		{
			ROS_INFO("入库时Y轴没对准！！！需要左右调整！！！zuobiao zuobiao ...................: %ld, %ld\n", dsp_status.XDist, dsp_status.YDist);
			if (dsp_status.YDist > 0)
			{
				ROS_INFO_STREAM("GO left ... GO left ... GO left ... GO left ... GO left ... GO left ... \n");
				robotstraightleft();
			}
			else
			{
				ROS_INFO_STREAM("GO right ... GO right ... GO right ... GO right ... GO right ... GO right ... \n");
				robotstraightright();
			}
		}
		else
		{
			ROS_INFO_STREAM("入库时左右对准了！！！不要动了！！！stop stop .....................................................");
			Stop();
			intention_num = 122;
		}
		break;

	/*case 1201:
		if (fabs(dsp_status.fAngle)>0.5)
		{
			ToAngleByMPU(0);
		}
		else
		{
			Stop();
			temp_normal_time=normal_time;
			intention_num=123;
		}
		break;*/
	case 122:
		ROS_INFO_STREAM("case 122 122 122 122 122 122 122 122 122 122 122 122 122 122 122 122\n");
		ROS_INFO("入库时角度大于0.5！！！需要旋转调整！！！jiaodu jiaodu before ...................:%lf\n", dsp_status.fAngle);
		dsp_status.fAngle =DecisionMaking::mth_ChangeAngle(dsp_status.fAngle);
		ROS_INFO("入库时角度大于0.5 ！！！需要旋转调整！！！jiaodu jiaodu after ...................:%lf\n", dsp_status.fAngle);
		if ((dsp_status.fAngle > 0.5) || (dsp_status.fAngle < -0.5))
		{

			/*dsp_status.fAngle = DecisionMaking::mth_ChangeAngle(dsp_status.fAngle);*/
			/*ROS_INFO_STREAM("入库时角度大于0.5！！！需要旋转调整！！！jiaodu jiaodu before ...................:%lf\n",dsp_status.fAngle);
			dsp_status.fAngle = DecisionMaking::mth_ChangeAngle(dsp_status.fAngle);
			ROS_INFO_STREAM("入库时角度大于0.5 ！！！需要旋转调整！！！jiaodu jiaodu after ...................:%lf\n",dsp_status.fAngle);*/
			if (dsp_status.fAngle > 0.0)
			{
				ROS_INFO_STREAM("turn left ... turn left ... turn left ... turn left ... turn left ... \n");
				ToFindObjectByturnleft();
			}
			else
			{
				ROS_INFO_STREAM("turn right ... turn right ... turn right ... turn right ... turn right ... \n");
				ToFindObjectByturnright();
			}
		}
		else
		{
			ROS_INFO_STREAM("角度再一次对准了，为0了！！！jiaodu jiaodu ................................................\n");
			Stop();
			intention_num = 123;
		}
		break;

	case 123:
		ROS_INFO_STREAM("case 123 123 123 123 123 123 123 123 123 123 123\n");
		if (dsp_status.XDist > (-10))
		{
			ROS_INFO_STREAM("back back back back back back back back back back back back back back back back\n");
			robotbacklow();
		}
		else
		{
			ROS_INFO_STREAM("stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop \n");
			Stop();
			intention_num = 15;
		}
		break;

	case 15:
		ROS_INFO_STREAM("case 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15\n");
		Stop(); //停车
		break;

	default:
		Stop();
		normal_time = 0;
		break;
	}
}

void DecisionMaking::Normal_Pttest() //陀螺仪测试：旋转多少度
{
	static int intention_num = 0;	 //回合过程的意图顺序号
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptOO, ptM;

	ptOO.x = 1000.0;
	ptOO.y = 0.0; //回合结束前，用于回位定点

	ptM.x = 0000.0;
	ptM.y = 4000.0; //M点

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;
	case 120:
		if ((normal_time - temp_normal_time) > 45) //延时2s
		{
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 1;
		}
		break;
		/*case 1:                        
	ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111\n");				
		ROS_INFO_STREAM("正在对准传球标定柱！！！！\n");
		if(ToAngleByMPU6050(ptM))
		{
			ROS_INFO_STREAM("已经对准了传球标定柱！！！！\n");
			if(stop())
			{
				temp_normal_time=normal_time;
				intention_num=2;
			}
		}
		break;
		case 2:
	ROS_INFO_STREAM("case 22222222222222222222222222222222222222222222222222222222222222222222\n");
			stop();
		default:
			break;
	}*/
	case 1:
		if (ToAngleByMPU(150))
		{
			if (stop())
			{
				ROS_INFO_STREAM("Normal_Pttest() stop...............................................................\n");
				intention_num = 2;
			}
		}
		break;

	case 2:
		Stop();
		break;
	}
}

void DecisionMaking::Normal_AtoB_test() //定点测试：O -> A -> B
{
	static int intention_num = 0;	 //回合过程的意图顺序号
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptOO, ptA, ptB;

	ptOO.x = 1000.0;
	ptOO.y = 0.0; //回合结束前，用于回位定点

	ptA.x = 2000.0;
	ptA.y = 0.0; //A点

	ptB.x = 2000.0;
	ptB.y = 1000.0; //B点

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;
	case 120:
		if ((normal_time - temp_normal_time) > 45) //延时2s
		{
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 1;
		}
		break;
	case 1:
		ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA)) //到达A点
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 3;
		}
		break;
		//	case 2:
		//ROS_INFO_STREAM("case 2222222222222222222222222222222222222222222222222222222222222222222222222222\n");
		//		if((normal_time-temp_normal_time)>10)
		//		{
		//			temp_normal_time=normal_time;
		//			if(ToAngleByMPU(90))	//旋转90度
		//			{
		//				Stop();
		//				intention_num=3;
		//			}
		//		}
		//		break;
	case 3:
		ROS_INFO_STREAM("case 3333333333333333333333333333333333333333333333333333333333333333333333333333\n");
		if ((normal_time - temp_normal_time) > 10)
		{
			temp_normal_time = normal_time;
			if (ToPostureByMilemeter_Pid2(ptB)) //到达B点
			{
				Stop();
				intention_num = 4;
			}
		}
		break;
	default:
		Stop();
		normal_time = 0;
		break;
	}
}

void DecisionMaking::Normal_bat3_new()
{
	//一些决策相关的标志
	static int intention_num = 0; //回合过程的意图顺序号
	static bool findballflag = 0; //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
	static bool aimballflag = 0;
	static bool Isthreescoreunnormal = 0; //捡三分球是否出现异常

	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;

	Posture ptA0, ptA, ptC, ptD, ptE, ptCC, ptD1;
	static Posture ptN = {0, 0, 0};

	ptA0.x = 1000;
	ptA0.y = 0;

	if (1 == place_num)
	{
		ptA.x = 4050; //中点，扫描起点
		ptA.y = 4600;

		ptC.x = 4600; //投篮标定柱位置
		ptC.y = 10925;

		ptD.x = 4500; //投篮正中定位点（激光距离篮筐2400）（钢板400计，激光超出30计）
		ptD.y = 7900;

		ptE.x = 1890; //投篮45偏角位置（预留出车身宽度和激光距离，分别400和30计）（2630处）
		ptE.y = 9065;

		ptCC.x = 800; //投篮标定柱位置
		ptCC.y = 7500;

		ptD1.x = 4600; //投篮正中定位点（激光距离篮筐2400）（钢板400计，激光超出30计）
		ptD1.y = 7900;

		//ptE.x=2053;			//投篮45偏角位置（2400处）
		//ptE.y=9228;
	}
	if (2 == place_num)
	{
		ptA.x = 3650; //中点，扫描起点
		ptA.y = -4800;

		ptC.x = 4250; //投篮标定柱位置
		ptC.y = -10925;

		ptD.x = 4100; //投篮正中定位点（激光距离篮筐2400）（钢板400计，激光超出30计）
		ptD.y = -8100;

		ptE.x = 1890; //投篮45偏角位置（预留出车身宽度和激光距离，分别400和30计）（2630处）
		ptE.y = -9065;

		ptCC.x = 800; //投篮标定柱位置
		ptCC.y = -7500;

		ptD1.x = 4100; //投篮正中定位点（激光距离篮筐2400）（钢板400计，激光超出30计）
		ptD1.y = -8100;
	}

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 000000000000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;
	case 120:
		if ((normal_time - temp_normal_time) > 200) //延时9s
		{
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 111111111111111111111111111111111111111111111111111111111111111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA0))
		{
			Stop();
			ROS_INFO_STREAM("已经到了A0点了 ！！！\n");
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 222222222222222222222222222222222222222222222222222222222222222\n");
		if (ToAngleByMPU6050(ptA))
		{
			if (stop())
			{
				ROS_INFO_STREAM("已经对准A点了 ！！！\n");
				temp_normal_time = normal_time;
				intention_num = 201;
			}
		}
		break;

	case 201:
		ROS_INFO_STREAM("case 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 \n");
		if (ToPostureByMilemeter_Pid3(ptA))
		{
			if (stop())
			{
				ROS_INFO_STREAM("已经到了A点了 ！！！\n");
				temp_normal_time = normal_time;
				intention_num = 3;
			}
		}
		break;
		/********************* 运动过程中避车 ************************/

		/********************* 运动避车结束 ************************/

	case 3: //使车子成66度倾斜，为横向移动找三分球做准备
		ROS_INFO_STREAM("case 3333333333333333333333333333333333333333333333333333333333333333333333333\n");
		if (1 == place_num)
		{
			ROS_INFO_STREAM("角度调整过程ing................\n");
			if (fabs(dsp_status.fAngle - 60.00) > 2)
			{
				ROS_INFO_STREAM("和66度相差大于2度................\n");
				if (dsp_status.fAngle > 62)
				{
					ROS_INFO_STREAM("角度为正，需要左旋................\n");
					ToFindObjectByturnleft();
				}
				if (dsp_status.fAngle < 58)
				{
					ROS_INFO_STREAM("角度为负，需要右旋................\n");
					ToFindObjectByturnright();
				}
			}
			else
			{
				if (stop())
				{
					ROS_INFO_STREAM("角度为60度，不要动了！！！................\n");
					temp_normal_time = normal_time;
					intention_num = 4;
				}
			}
		}
		if (2 == place_num)
		{
			ROS_INFO_STREAM("角度调整过程ing................\n");
			if (fabs(dsp_status.fAngle - (-63.00)) > 2)
			{
				ROS_INFO_STREAM("和-63度相差大于2度................\n");
				if (dsp_status.fAngle > -61)
				{
					ROS_INFO_STREAM("角度为正，需要左旋................\n");
					ToFindObjectByturnleft();
				}
				if (dsp_status.fAngle < -65)
				{
					ROS_INFO_STREAM("角度为负，需要右旋................\n");
					ToFindObjectByturnright();
				}
			}
			else
			{
				if (stop())
				{
					ROS_INFO_STREAM("角度为-66度，不要动了！！！................\n");
					temp_normal_time = normal_time;
					intention_num = 4;
				}
			}
		}
		break;

		/***************************** 第一次识别球 *******************************/
	case 4:
		ROS_INFO_STREAM("case 44444444444444444444444444444444444444444444444444444444444444444444444444444\n");
		if ((normal_time - temp_normal_time) > 5)
		{
			//TODO: 发送接收数据
			/*
			if(!Vision.create_communication_RAM_2())
			{
				ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
				if(!Vision.open_file_mapping_2())
				{
					ROS_INFO_STREAM("fail to open_file_mapping_2\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to open_file_mapping_2\n");
					if(!Vision.create_communication_RAM_4())
					{
						ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
						if(!Vision.open_file_mapping_4())
						{
							ROS_INFO_STREAM("fail to open_file_mapping_4\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to open_file_mapping_4\n");
							TheObject.whatObject =(objectType)8;//找什么东西在这里改
							if(!Vision.send_find_whatobject(TheObject.whatObject))
							{
								ROS_INFO_STREAM("Fail to send command\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to send command\n");
								TheObject.whatObject = (objectType)0;//unkownObject变量重新初始化，为下一次找球做准备
								if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
								{
									ROS_INFO_STREAM("Fail to receive final object result\n");
								}
								else
								{
									ROS_INFO_STREAM("Success to receive final object result\n");
									ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
									ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
								}
							}
						}
					}
				}
			}*/
			if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
			{
				findballflag = 0;
				aimballflag = 0;
				if (1 == place_num)
				{
					if (abs((dsp_status.XDist) - (ptCC.x)) < 50)
					{
						Stop();
						Isthreescoreunnormal = 1; //找三分球，没有找到，需要避障去向三分线内继续找
						ROS_INFO_STREAM("已经到边界了，但啥也没找着，算了，去找内圈球吧！！！\n");
						//ptN.x = dsp_status.XDist;				//把当前坐标赋给点N
						//ptN.y = dsp_status.YDist;
						//ROS_INFO_STREAM("点N的坐标为：%ld , %ld\n",ptN.x, ptN.y);
						temp_normal_time = normal_time;
						intention_num = 9;
					}
					else
					{
						ROS_INFO_STREAM("以66度方向行进...\n");
						robotstraightrightlow(); //继续右走
					}
				}
				if (2 == place_num)
				{
					if (abs((dsp_status.XDist) - (ptCC.x)) < 50)
					{
						Stop();
						Isthreescoreunnormal = 1; //找三分球，没有找到，需要避障去向三分线内继续找
						ROS_INFO_STREAM("已经到边界了，但啥也没找着，算了，去找内圈球吧！！！\n");
						//ptN.x = dsp_status.XDist;				//把当前坐标赋给点N
						//ptN.y = dsp_status.YDist;
						//ROS_INFO_STREAM("点N的坐标为：%ld , %ld\n",ptN.x, ptN.y);
						temp_normal_time = normal_time;
						intention_num = 9;
					}
					else
					{
						ROS_INFO_STREAM("以-66度方向行进...\n");
						robotstraightleftlow(); //继续左走
					}
				}
			}
			else
			{
				findballflag = 1;
				Isthreescoreunnormal = 0;
				ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
			}
			if ((findballflag == 1) && (aimballflag == 0))
			{
				ROS_INFO_STREAM(" cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
				if (ToAimBallByVision())
				{
					ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //对准球的过程中突然丢了
						findballflag = 0;
						aimballflag = 0;
						ROS_INFO("丢了 丢了 丢了 ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 4;
						}
					}
					else
					{
						aimballflag = 1;
						ROS_INFO("!!!!!!!!三分线球:objectAngle aimedaimedaimed is %lf;三分线球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					}
				}
			}
			if ((findballflag == 1) && (aimballflag == 1))
			{
				ROS_INFO_STREAM(" ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n");
				if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
				{
					ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //前往球的跟前过程中，又没看到球，重新去找
						aimballflag = 0;
						findballflag = 0;
						ROS_INFO_STREAM(" 又丢了 又丢了 又丢了 ！！！\n");
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 4;
						}
					}
					else
					{
						aimballflag = 0;
						findballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 5;
						ROS_INFO_STREAM("捡球 捡球 捡球 ！！！\n");
					}
				}
			}
		}
		break;

	case 5:
		ROS_INFO_STREAM("case 55555555555555555555555555555555555555555555555555555555555555555555555555555\n");
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令
		ROS_INFO_STREAM("case uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuppppppppppppppppppppppppppppppppppppp\n");
		temp_normal_time = normal_time;
		intention_num = 6; //抬机械臂命令已发送
		break;

	case 6: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 66666666666666666666666666666666666666666666666666666666666666666666666666666\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 40)
		{
			ROS_INFO_STREAM("机械臂抬起，等待延时...\n");
			temp_normal_time = normal_time;
			intention_num = 7;
		}
		break;

	case 7:
		ROS_INFO_STREAM("case 77777777777777777777777777777777777777777777777777777777777777777777777777777\n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中发第一次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			ROS_INFO_STREAM("第一次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 8;
		}
		break;

	case 8:
		ROS_INFO_STREAM("case 88888888888888888888888888888888888888888888888888888888888888888888888888888\n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			temp_normal_time = normal_time;
			intention_num = 80;
			ROS_INFO_STREAM("发现已经捡到球了，架子上有球！！！\n");
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			temp_normal_time = normal_time;
			intention_num = 81;
			ROS_INFO_STREAM("判断没有球啊！等待抬臂稳定后在判断一遍！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 80:
		ROS_INFO_STREAM("case 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80\n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 9;
		}
		break;

	case 81:
		ROS_INFO_STREAM("case 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81\n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("第二次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 82;
		}
		break;

	case 82:
		ROS_INFO_STREAM("case 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82\n");
		Control.SendData_state = 0; //单片机位置0
		ROS_INFO_STREAM("第二次，判断架子上没有球！！！！\n");
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("发现有球啊！！！第一次误判！！！\n");
			temp_normal_time = normal_time;
			intention_num = 9;
		}
		else //判断两次都没球，可以再去找了
		{
			findballflag = 0;
			aimballflag = 0;
			temp_normal_time = normal_time;
			intention_num = 4;
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 9: //转向目的点，去找标定柱
		ROS_INFO_STREAM("case 99999999999999999999999999999999999999999999999999999999999999999999999\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (ToAngleByMPU6050(ptD))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 90;
				}
			}
		}
		break;

	case 90:
		ROS_INFO_STREAM("case 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 \n");
		ROS_INFO_STREAM("正在对准传球目标点,准备避障行��！！！！\n");
		if (ToPostureByAvoidance(ptD))
		{
			ROS_INFO_STREAM("已经对准了投篮正中目标点，开始避障行进 ！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("ToPostureByAvoidance(ptM) ToPostureByAvoidance(ptM) ToPostureByAvoidance(ptM)\n");
				temp_normal_time = normal_time;
				intention_num = 91;
			}
		}
		if ((normal_time - temp_normal_time) > 180)
		{
			Stop();
			ROS_INFO_STREAM("到了100秒 ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 12;
		}
		break;

	case 91:
		ROS_INFO_STREAM("case 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 \n");
		if ((normal_time - temp_normal_time) < 22) //以开环速度向前1s
		{
			robotforward();
		}
		else
		{
			temp_normal_time = normal_time;
			intention_num = 92;
		}
		break;

	case 92: //避障后再次对准目标点
		ROS_INFO_STREAM("case 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 \n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (ToAngleByMPU6050(ptD))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 11;
				}
			}
		}
		break;

		//	case 10:
		//ROS_INFO_STREAM("case 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 \n");
		//		if((normal_time-temp_normal_time)>2)
		//		{
		//			if(ToAngleByMPU6050(ptD))		//对准投篮定位点
		//			{
		//				ROS_INFO_STREAM("已经对准了D ！！！\n");
		//				if(stop())
		//				{
		//					ROS_INFO_STREAM("停下 ！！！\n");
		//					temp_normal_time=normal_time;
		//					intention_num=11;
		//				}
		//			}
		//		}
		//		break;

	case 11:
		ROS_INFO_STREAM("case 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 \n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (ToPostureByMilemeter_Pid3(ptD))
			{
				Stop();
				ROS_INFO_STREAM("已经到了D点 ！！！n");
				temp_normal_time = normal_time;
				intention_num = 12;
			}
		}
		break;

	case 12:
		ROS_INFO_STREAM("case 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 \n");
		if (ToAngleByMPU6050(ptC)) //对准投篮标定柱
		{
			ROS_INFO_STREAM("已经对准了C ！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("停下 ！！！\n");
				temp_normal_time = normal_time;
				intention_num = 13;
			}
		}
		break;

		/********************************* 第一次识别标定柱 *********************************/
	case 13:
		ROS_INFO_STREAM("case 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 \n ");
		//TODO:发送接收数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
						TheObject.whatObject = (objectType)1;	//找什么东西在这里改
						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to send command\n");
							TheObject.whatObject = (objectType)0;//变量初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");	
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}*/
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				ToFindObjectByturnright();
			}
			if (2 == place_num)
			{
				ToFindObjectByturnleft();
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag \n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((00.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 13;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!标注杆:objectAngle aimedaimedaimed is %lf;标注杆:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					stop();
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("!!!!!!!!标定柱:The distance has been to the min error;The distance has been to the min error!!!!!!!!\n");
					temp_normal_time = normal_time;
					intention_num = 14;
				}
			}
		}
		break;

	case 14: //发射
		ROS_INFO_STREAM("case 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 \n");
		if (normal_time - temp_normal_time > 15)
		{
			Stop();
			Control.SendData_state = 3; //发送发射指令，下一步要立即置位0
			ROS_INFO_STREAM("发射 ！！！\n");
			temp_normal_time = normal_time;
			intention_num = 15;
		}
		break;

	case 15:
		ROS_INFO_STREAM("case 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 \n");
		Control.SendData_state = 0;
		if (1 == place_num)
		{
			if (ToAngleByMPU(-90))
			{
				Stop();
				temp_normal_time = normal_time;
				intention_num = 16;
				/*robotstraightleftlow();
				ROS_INFO_STREAM("横向左移找球..\n");*/
			}
		}
		if (2 == place_num)
		{
			if (ToAngleByMPU(90))
			{
				Stop();
				temp_normal_time = normal_time;
				intention_num = 16;
				/*robotstraightrightlow();
				ROS_INFO_STREAM("横向右移找球..\n");*/
			}
		}
		break;

		/************************************ 第二次找球 ***********************************/
	case 16:
		ROS_INFO_STREAM("case 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 \n");
		temp_normal_time = normal_time; //为延时作准备
		//TODO:发送接收数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");

						//视觉修改版找球测试start
						TheObject.whatObject =(objectType)8;//找什么东西在这里改
						if(TheObject.whatObject==(objectType)1)
							ROS_INFO_STREAM("send command to find calibration\n");
						if(TheObject.whatObject==(objectType)2)
							ROS_INFO_STREAM("send command to find reddish_brown_basketball\n");
						if(TheObject.whatObject==(objectType)3)
							ROS_INFO_STREAM("send command to find basketball\n");
						if(TheObject.whatObject==(objectType)4)
							ROS_INFO_STREAM("send command to find blue_gray_basketball\n");
						if(TheObject.whatObject==(objectType)5)
							ROS_INFO_STREAM("send command to find calibration_plus\n");
						if(TheObject.whatObject==(objectType)6)
							ROS_INFO_STREAM("send command to find orange_red_volleyball\n");
						if(TheObject.whatObject==(objectType)7)
							ROS_INFO_STREAM("send command to find red_blue_volleyball\n");
						if(TheObject.whatObject==(objectType)8)
							ROS_INFO_STREAM("send command to find volleyball\n");

						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM(" Success to send command Success to send command Success to send command Success to send command\n");
							TheObject.whatObject = (objectType)0;//unkownObject变量重新初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}*/
		if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || (objectAngleDistance_Y.second > 1000))
		{
			aimballflag = 0;
			findballflag = 0;
			if (1 == place_num)
			{
				robotstraightleftlow();
				ROS_INFO_STREAM("横向左移找球..\n");
			}
			if (2 == place_num)
			{
				robotstraightrightlow();
				ROS_INFO_STREAM("横向右移找球..\n");
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM(" findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 16;
					ROS_INFO("丢了 丢了 丢了 : %lf \n", objectAngleDistance_Y.first);
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!中圈球:objectAngle aimedaimedaimed is %lf;中圈球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1))
		{
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					intention_num = 16;
					ROS_INFO_STREAM("又丢了 又丢了 又丢了 \n");
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					intention_num = 17;
					ROS_INFO_STREAM("捡球 捡球 捡球 \n");
				}
			}
		}
		break;

	case 17:
		ROS_INFO_STREAM("case 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 \n");
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令
		ROS_INFO_STREAM("case uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuppppppppppppppppppppppppppppppppppppp\n");
		temp_normal_time = normal_time;
		intention_num = 18; //抬机械臂命令已发送
		break;

	case 18: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 40)
		{
			ROS_INFO_STREAM("机械臂抬起，等待延时...\n");
			temp_normal_time = normal_time;
			intention_num = 19;
		}
		break;

	case 19:
		ROS_INFO_STREAM("case 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 \n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中发第一次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			ROS_INFO_STREAM("第一次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 20;
		}
		break;

	case 20:
		ROS_INFO_STREAM("case 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			temp_normal_time = normal_time;
			intention_num = 21;
			ROS_INFO_STREAM("发现已经捡到球了，架子上有球！！！\n");
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			temp_normal_time = normal_time;
			intention_num = 22;
			ROS_INFO_STREAM("判断没有球啊！等待抬臂稳定后在判断一遍！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 21:
		ROS_INFO_STREAM("case 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 \n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 24;
		}
		break;

	case 22:
		ROS_INFO_STREAM("case 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 \n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("第二次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 23;
		}
		break;

	case 23:
		ROS_INFO_STREAM("case 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 \n");
		Control.SendData_state = 0; //单片机位置0
		ROS_INFO_STREAM("第二次，判断架子上有没有球！！！！\n");
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("发现有球啊！！！第一次误判！！！\n");
			temp_normal_time = normal_time;
			intention_num = 24;
		}
		else //判断两次都没球，可以再去找了
		{
			findballflag = 0;
			aimballflag = 0;
			intention_num = 16;
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 24:
		ROS_INFO_STREAM("case 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 \n");
		ROS_INFO_STREAM("正在对准投篮定位点！！！！\n");
		if (ToAngleByMPU6050(ptD1))
		{
			ROS_INFO_STREAM("已经对准了投篮定位点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 25;
			}
		}
		break;

	case 25:
		ROS_INFO_STREAM("case 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 \n");
		if (ToPostureByMilemeter_Pid3(ptD1))
		{
			Stop();
			ROS_INFO_STREAM("已经到了D点 ！！！n");
			temp_normal_time = normal_time;
			intention_num = 26;
		}
		break;

	case 26:
		ROS_INFO_STREAM("case 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 \n");
		if (ToAngleByMPU6050(ptC)) //对准投篮标定柱
		{
			ROS_INFO_STREAM("已经对准了C ！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("停下 ！！！\n");
				temp_normal_time = normal_time;
				intention_num = 27;
			}
		}
		break;

		/***************************** 第二次找标定柱 *******************************/
	case 27:
		ROS_INFO_STREAM("case 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 \n ");
		//TODO:接受发送数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
						TheObject.whatObject = (objectType)1;	//找什么东西在这里改
						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to send command\n");
							TheObject.whatObject = (objectType)0;//变量初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");	
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}
		*/
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				ToFindObjectByturnright();
			}
			if (2 == place_num)
			{
				ToFindObjectByturnleft();
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((00.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 27;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!标注杆:objectAngle aimedaimedaimed is %lf;标注杆:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90) < 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					stop();
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("!!!!!!!!标定柱:The distance has been to the min error;The distance has been to the min error!!!!!!!!\n");
					temp_normal_time = normal_time;
					intention_num = 28;
				}
			}
		}
		break;

	case 28:
		ROS_INFO_STREAM("case 28 28 28 28 28 28 28 28 28 28 28 28 28 28 28 28 28 28 28 28 28 28 \n");
		if ((normal_time - temp_normal_time) > 15)
		{
			Control.SendData_state = 3; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("发射 发射 发射 ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 29;
		}
		break;

	case 29:
		ROS_INFO_STREAM("case 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 \n");
		Control.SendData_state = 0;
		Stop();

	default:
		break;
	}
}

void DecisionMaking::Normal_bat0()
{
	static int intention_num = 0;	  //回合过程的意图顺序号
	static bool find_object_flag = 0; //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
	static bool aim_object_flag = 0;
	static bool findballflag = 0; //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
	static bool aimballflag = 0;
	static bool armupflag = 0;
	static bool stopflag = 0;
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;

	Posture ptA0, ptC, ptD, ptE;
	ptA0.x = 1000; //出发定点1
	ptA0.y = 0;

	if (1 == place_num)
	{
		ptC.x = 4600; //投篮标定柱位置
		ptC.y = 10925;

		ptD.x = 4600; //投篮正中定位点（激光距离篮筐2400）（钢板400计，激光超出30计）
		ptD.y = 7500;

		ptE.x = 1890; //投篮45偏角位置（预留出车身宽度和激光距离，分别400和30计）（2630处）
		ptE.y = 9065;
	}
	if (2 == place_num)
	{
		ptC.x = 4500; //投篮标定柱位置
		ptC.y = -10925;

		ptD.x = 4600; //投篮正中定位点（激光距离篮筐2400）（钢板400计，激光超出30计）
		ptD.y = -7500;

		ptE.x = 1890; //投篮45偏角位置（预留出车身宽度和激光距离，分别400和30计）（2630处）
		ptE.y = -9065;
	}

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 00000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		//m_bSend=1;       //TODO:允许给单片机发送命令
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;

	case 120:
		if ((normal_time - temp_normal_time) > 100) //延时17.1s
		{
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 13;
		}
		break;

	case 13:
		ROS_INFO_STREAM("case 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 \n ");
		//TODO:发送接收数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
						TheObject.whatObject = (objectType)1;	//找什么东西在这里改
						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to send command\n");
							TheObject.whatObject = (objectType)0;//变量初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");	
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}
		*/

		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				ToFindObjectByturnright();
			}
			if (2 == place_num)
			{
				ToFindObjectByturnleft();
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag \n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((00.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 13;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!标注杆:objectAngle aimedaimedaimed is %lf;标注杆:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || abs(objectAngleDistance_Y.second - 2400) < 50) && (aimballflag == 1))
				{
					stop();
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO("!!!!!!!!标定柱:The distance has been to the min error;The distance has been to the min error!!!!!!!!\n");
					temp_normal_time = normal_time;
					intention_num = 14;
				}
			}
		}
		break;

	case 14: //发射
		ROS_INFO_STREAM("case 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 \n");
		if (normal_time - temp_normal_time > 15)
		{
			Stop();
			Control.SendData_state = 3; //发送发射指令，下一步要立即置位0
			ROS_INFO_STREAM("发射 ！！！\n");
			temp_normal_time = normal_time;
			intention_num = 4;
		}
		break;

		//	case 1:
		//ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111111111111\n");
		//		if(ToPostureByMilemeter_Pid1(ptA0))
		//		{
		//			Stop();
		//			temp_normal_time=normal_time;
		//			intention_num=111;
		//		}
		//		break;
		//
		//	case 111:
		//ROS_INFO_STREAM("case 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111\n");
		//		if(ToAngleByMPU6050(ptD))
		//		{
		//			if(stop())
		//			{
		//				temp_normal_time=normal_time;
		//				intention_num=112;
		//			}
		//		}
		//		break;
		//
		//	case 112:
		//ROS_INFO_STREAM("case 112 112 112 112 112 112 112 112 112 112 112 112 112 112 112\n");
		//		if(ToPostureByMilemeter_Pid3(ptD))
		//		{
		//			if(stop())
		//			{
		//				temp_normal_time=normal_time;
		//				intention_num=2;
		//			}
		//		}
		//		break;
		//
		//	case 2:
		//ROS_INFO_STREAM("case 22222222222222222222222222222222222222222222222222222222222222222222222222222\n ");
		//		if(!Vision.create_communication_RAM_2())
		//		{
		//			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		//		}
		//		else
		//		{
		//			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
		//			if(!Vision.open_file_mapping_2())
		//			{
		//				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
		//			}
		//			else
		//			{
		//				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
		//				if(!Vision.create_communication_RAM_4())
		//				{
		//					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
		//				}
		//				else
		//				{
		//					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
		//					if(!Vision.open_file_mapping_4())
		//					{
		//						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
		//					}
		//					else
		//					{
		//						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
		//						TheObject.whatObject = (objectType)1;	//找什么东西在这里改
		//						if(!Vision.send_find_whatobject(TheObject.whatObject))
		//						{
		//							ROS_INFO_STREAM("Fail to send command\n");
		//						}
		//						else
		//						{
		//							ROS_INFO_STREAM("Success to send command\n");
		//							TheObject.whatObject = (objectType)0;//变量初始化，为下一次找球做准备
		//							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
		//							{
		//								ROS_INFO_STREAM("Fail to receive final object result\n");
		//							}
		//							else
		//							{
		//								ROS_INFO_STREAM("Success to receive final object result\n");
		//								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
		//								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
		//							}
		//						}
		//					}
		//				}
		//			}
		//		}
		//		if((objectAngleDistance_Y.first==0.0)&&(objectAngleDistance_Y.second==0))
		//		{
		//			find_object_flag=0;
		//			if (1==place_num)
		//			{
		//				ToFindObjectByturnright();
		//			}
		//			if (2==place_num)
		//			{
		//				ToFindObjectByturnleft();
		//			}
		//		}
		//		else
		//		{
		//			find_object_flag=1;
		//			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		//		}
		//		if((find_object_flag==1)&&(aim_object_flag==0))
		//		{
		//			if (ToAimBallByVision())
		//			{
		//				if ((00.0==objectAngleDistance_Y.first)&&(0.0==objectAngleDistance_Y.second))
		//				{//对准的过程中突然丢了
		//					find_object_flag=0;
		//					intention_num=2;
		//				}
		//				else
		//				{
		//					aim_object_flag=1;
		//					ROS_INFO_STREAM("!!!!!!!!标注杆:objectAngle aimedaimedaimed is %lf;标注杆:distance aimedaimedaimed is %ld!!!!!!!!\n",objectAngleDistance_Y.first,objectAngleDistance_Y.second);
		//				}
		//			}
		//		}
		//		if((aim_object_flag==1))//前进到距离2.5m
		//		{
		//			if(ToPoleByVision(objectAngleDistance_Y.second,objectAngleDistance_Y.first))
		//			{
		//				if((objectAngleDistance_Y.first==90.0)&&(objectAngleDistance_Y.second==0)&&(aim_object_flag==1))
		//				{
		//					stop();
		//					aim_object_flag=0;
		//					find_object_flag=0;
		//					ROS_INFO_STREAM("!!!!!!!!标定柱:The distance has been to the min error;The distance has been to the min error!!!!!!!!\n");
		//					intention_num=3;
		//				}
		//			}
		//		}
		//		break;
		//
		//	case 3:                         //发射
		//ROS_INFO_STREAM("case 33333333333333333333333333333333333333333333333333333333333333333333333333333\n");
		//		if((normal_time-temp_normal_time)>11)
		//		{
		//			Stop();
		//			m_bSend=1;TODO:
		//			Control.SendData_state=3;               //发送发射指令，下一步要立即置位0
		//			temp_normal_time=normal_time;
		//			intention_num=4;
		//		}
		//		break;

	case 4:
		ROS_INFO_STREAM("case 4444444444444444444444444444444444444444444444444444444444444444444444444444444\n");
		Control.SendData_state = 0;
		Stop();
		break;

	default:
		break;
	}
}

void DecisionMaking::Normal_pass0() //激光摄像头测试
{
	static int intention_num = 0; //回合过程的意图顺序号
	static bool findballflag = 0; //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
	static bool aimballflag = 0;
	static bool armupflag = 0;
	static bool stopflag = 0;
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;

	case 120:
		if ((normal_time - temp_normal_time) > 45) //延时2S
		{
			/*if(ToBallByVision_new())
			{*/
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 2;
			//}
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 2222222222222222222222222222222222222222222222222222222222222222222222222222222\n");
		//temp_normal_time=normal_time;     //为延时作准备
		//TODO:发送接收数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");

		//视觉修改版找球测试start
						TheObject.whatObject =(objectType)8;//找什么东西在这里改
						if(TheObject.whatObject==(objectType)1)
							ROS_INFO_STREAM("send command to find calibration\n");
						if(TheObject.whatObject==(objectType)2)
							ROS_INFO_STREAM("send command to find reddish_brown_basketball\n");
						if(TheObject.whatObject==(objectType)3)
							ROS_INFO_STREAM("send command to find basketball\n");
						if(TheObject.whatObject==(objectType)4)
							ROS_INFO_STREAM("send command to find blue_gray_basketball\n");
						if(TheObject.whatObject==(objectType)5)
							ROS_INFO_STREAM("send command to find calibration_plus\n");
						if(TheObject.whatObject==(objectType)6)
							ROS_INFO_STREAM("send command to find orange_red_volleyball\n");
						if(TheObject.whatObject==(objectType)7)
							ROS_INFO_STREAM("send command to find red_blue_volleyball\n");
						if(TheObject.whatObject==(objectType)8)
							ROS_INFO_STREAM("send command to find volleyball\n");

						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM(" Success to send command Success to send command Success to send command Success to send command\n");
							TheObject.whatObject = (objectType)0;unkownObject//变量重新初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}
		*/

		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			//ToFindObjectByturnright(); //左旋找球测试（需要测试旋转找球时请将该函数释放）
			//ROS_INFO_STREAM(" 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转\n");
			robotstraightleft(); //右移找球测试
			ROS_INFO_STREAM(" 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 \n");
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM(" findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					intention_num = 2;
					ROS_INFO("丢了 丢了 丢了 : %lf \n", objectAngleDistance_Y.first);
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!中圈球:objectAngle aimedaimedaimed is %lf;中圈球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1))
		{
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					intention_num = 2;
					ROS_INFO_STREAM("又丢了 又丢了 又丢了 \n");
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					intention_num = 5;
					ROS_INFO_STREAM("捡球 捡球 捡球 \n");
				}
			}
		}
		break;

	case 5:
		ROS_INFO_STREAM("case 55555555555555555555555555555555555555555555555555555555555555555555555555555\n");
		//立即捡球，不要延时
		Stop();
		//m_bSend=1;    //TODO:允许给单片机发送命令
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		temp_normal_time = normal_time;
		intention_num = 6; //抬机械臂命令已发送
		break;
	case 6: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 66666666666666666666666666666666666666666666666666666666666666666666666666666\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 80)
		{
			temp_normal_time = normal_time;
			intention_num = 7;
		}
		break;

	case 7:
		Stop();
		break;
	}
}

void DecisionMaking::Normal_pass1() //**********传球-回合1**********
{
	//一些决策相关的标志
	static int intention_num = 0; //回合过程的意图顺序号
	static bool findballflag = 0; //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
	static bool aimballflag = 0;

	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	static int vision_scan_time = 0; //视觉扫描次数统计，达到设定值时，车子要回归原位
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptOO, ptA, ptB, ptM, ptM2, ptN, ptxy;

	ptOO.x = 1000;
	ptOO.y = 0; //回合结束前，用于回位定点

	ptA.x = 3300;
	ptA.y = 0; //车子带着篮球进入传球边界线内的点，车子在此处往目标点M发射球

	if (1 == place_num)
	{
		ptxy.x = 0;
		ptxy.y = 100;

		ptM.x = 800;
		ptM.y = 4250; //标定柱

		ptM2.x = 500;
		ptM2.y = 5500;

		ptN.x = 2400;
		ptN.y = 1200; //投球定位点
	}
	if (2 == place_num)
	{
		ptxy.x = 0;
		ptxy.y = 0;

		ptM.x = 875;
		ptM.y = -4300; //A口进入时投球区中心位置

		ptM2.x = 500;
		ptM2.y = -5500;

		ptN.x = 2400;
		ptN.y = -1200;
	}

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;

	case 120:
		if ((normal_time - temp_normal_time) > 200) //延时大约9s
		{
			temp_normal_time = normal_time;
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA)) //走到点A
		{
			ROS_INFO("我到 A 点了 我到 A 点了 我到 A 点了 ！！！X is : %ld，Y is : %ld, angle is : %lf\n", dsp_status.XDist, dsp_status.YDist, dsp_status.fAngle);
			Stop();
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 22222222222222222222222222222222222222222222222222222222222222222222222222222\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			ROS_INFO_STREAM("正在对准传球标定柱！！！！\n");
			if (ToAngleByMPU6050(ptM)) //瞄准目标点M
			{
				ROS_INFO_STREAM("已经对准了传球标定柱！！！！\n");
				if (stop())
				{
					ROS_INFO_STREAM("对准之后已经停下，可以发射了！！！！\n");
					temp_normal_time = normal_time;
					intention_num = 3;
				}
			}
		}
		break;

	case 3: //发射第一个篮球
		ROS_INFO_STREAM("case 33333333333333333333333333333333333333333333333333333333333333333333333333333\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("发射 发射 发射 ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 30;
		}
		break;

	case 30:
		ROS_INFO_STREAM("case 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 3)
		{
			ROS_INFO_STREAM("正在转回来！！！\n");
			if (ToAngleByMPU(4))
			{
				ROS_INFO_STREAM("转回来找第二个球,Stop！！！\n");
				Stop();
				temp_normal_time = normal_time;
				intention_num = 4;
			}
		}
		break;

	case 4: //**********视觉找篮球开始**********
		ROS_INFO_STREAM("case 44444444444444444444444444444444444444444444444444444444444444444444444444444\n");
		if ((normal_time - temp_normal_time) > 5)
		{
			the_object.whatObject = (objectType)3;
			object_pub(the_object);
			//TODO:发送接收数据
			/*
			if(!Vision.create_communication_RAM_2())
			{
				ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
				if(!Vision.open_file_mapping_2())
				{
					ROS_INFO_STREAM("fail to open_file_mapping_2\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to open_file_mapping_2\n");
					if(!Vision.create_communication_RAM_4())
					{
						ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
						if(!Vision.open_file_mapping_4())
						{
							ROS_INFO_STREAM("fail to open_file_mapping_4\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to open_file_mapping_4\n");
							TheObject.whatObject =(objectType)3;//找什么东西在这里改
							if(!Vision.send_find_whatobject(TheObject.whatObject))
							{
								ROS_INFO_STREAM("Fail to send command\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to send command\n");
								TheObject.whatObject = (objectType)0;//unkownObject变量重新初始化，为下一次找球做准备
								if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
								{
									ROS_INFO_STREAM("Fail to receive final object result\n");
								}
								else
								{
									ROS_INFO_STREAM("Success to receive final object result\n");
									ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
									ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
								}
							}
						}
					}
				}
			}*/
		}

		if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				if ((normal_time - temp_normal_time) < 50)
				{
					robotstraightleft(); //开环横向低速左移
					ROS_INFO_STREAM(" 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移  \n");
				}
				else if ((normal_time - temp_normal_time) < 72)
				{
					Stop();
				}
				else
				{
					robotstraightright(); //开环横向低速右移
					ROS_INFO_STREAM(" 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 \n");
				}
				if ((dsp_status.YDist) > 3500) //找了好久，一个都没找到，就回去了
				{
					ROS_INFO_STREAM("中圈走完了，也没找到球！！！那就回家吧！！！\n");
					Stop();
					temp_normal_time = normal_time;
					intention_num = 11;
				}
			}
			if (2 == place_num)
			{
				if ((normal_time - temp_normal_time) < 50)
				{
					robotstraightright(); //开环横向低速右移
					ROS_INFO_STREAM(" 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 \n");
				}
				else if ((normal_time - temp_normal_time) < 72)
				{
					Stop();
				}
				else
				{
					robotstraightleft(); //开环横向低速左移
					ROS_INFO_STREAM(" 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移  \n");
				}
				if ((dsp_status.YDist) < -3500) //找了好久，一个都没找到，就回去了
				{
					ROS_INFO_STREAM("中圈走完了，也没找到球！！！那就回家吧！！！\n");
					Stop();
					temp_normal_time = normal_time;
					intention_num = 11;
				}
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			ROS_INFO_STREAM(" cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 4;
					ROS_INFO("丢了 丢了 丢了 ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!中圈球:objectAngle aimedaimedaimed is %lf;中圈球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1) && (aimballflag == 1))
		{
			ROS_INFO_STREAM(" ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n");
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 4;
					ROS_INFO_STREAM(" 又丢了 又丢了 又丢了 ！！！\n");
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 401;
					ROS_INFO_STREAM("捡球 捡球 捡球 ！！！\n");
				}
			}
		}
		break; //**********视觉找篮球结束**********
	case 401:
		ROS_INFO_STREAM("case 401 401 401 \n");
		if ((normal_time - temp_normal_time) < 17)
			robotforward();
		else
		{
			temp_normal_time = normal_time;
			intention_num = 5;
		}

		break;
	case 5:
		ROS_INFO_STREAM("case 55555555555555555555555555555555555555555555555555555555555555555555555555555\n");
		//立即捡球，不要延时
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令，下一步立即置位0
		temp_normal_time = normal_time;
		intention_num = 6; //抬机械臂命令已发送
		break;

	case 6: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 66666666666666666666666666666666666666666666666666666666666666666666666666666\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 40)
		{
			temp_normal_time = normal_time;
			intention_num = 7;
		}
		break;

	case 7:
		ROS_INFO_STREAM("case 77777777777777777777777777777777777777777777777777777777777777777777777777777\n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中发第一次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			temp_normal_time = normal_time;
			intention_num = 8;
		}
		break;

	case 8:
		ROS_INFO_STREAM("case 88888888888888888888888888888888888888888888888888888888888888888888888888888\n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			temp_normal_time = normal_time;
			intention_num = 80;
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			temp_normal_time = normal_time;
			intention_num = 81;
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 80:
		ROS_INFO_STREAM("case 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80\n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 9;
		}
		break;

	case 81:
		ROS_INFO_STREAM("case 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81\n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			//m_bSend=1;    //TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球
			temp_normal_time = normal_time;
			intention_num = 82;
		}
		break;

	case 82:
		ROS_INFO_STREAM("case 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82\n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //
		{
			temp_normal_time = normal_time;
			intention_num = 9;
		}
		else //判断两次都没球，可以再去找了
		{
			findballflag = 0;
			aimballflag = 0;
			temp_normal_time = normal_time;
			intention_num = 4;
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 9: //确认有球后，有球，有球有球！！！！！
		ROS_INFO_STREAM("case 99999999999999999999999999999999999999999999999999999999999999999999999999999\n");
		ROS_INFO_STREAM("正在对准传球定位点！！！！\n");
		Control.SendData_state = 6;
		if (ToAngleByMPU6050(ptA)) //准备发射
		{
			ROS_INFO_STREAM("已经对准了传球定位点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停到了投球定位点，再旋转调整！！！！\n");
				Control.SendData_state = 7;
				temp_normal_time = normal_time;
				intention_num = 91;
			}
		}
		break;

	case 91:
		ROS_INFO_STREAM("case 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 \n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid2(ptA)) //到达N点
		{
			Stop();
			ROS_INFO_STREAM("到 NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN啦啦啦啦啦\n");
			temp_normal_time = normal_time;
			intention_num = 92;
		}
		break;

	/*case 910:
		ROS_INFO_STREAM("case 910 910 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 \n");
		if (ToPostureByMilemeter_Pid6(ptN)) //到达N点
		{
			Stop();
			ROS_INFO_STREAM("到 NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN啦啦啦啦啦\n");
			temp_normal_time = normal_time;
			intention_num = 92;
		}
		break;
		*/
	case 92:
		ROS_INFO_STREAM("case 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 \n");
		ROS_INFO_STREAM("正在对准传球标定柱！！！！\n");
		if (ToAngleByMPU6050(ptM2))
		{
			ROS_INFO_STREAM("已经对准了传球标定柱！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，可以发射了！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 10;
			}
		}
		break;

	case 10:
		ROS_INFO_STREAM("case 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("发射 发射 发射 ！！！！\n");
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 11;
		}
		break;

	case 11: //**********车子归位开始**********
		ROS_INFO_STREAM("case 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 3)
		{
			ROS_INFO_STREAM("走向 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ing\n");
			if (ToAngleByPosture2(ptOO)) //边走边调整角度先回位OO点，再微调退回框
			{
				ROS_INFO_STREAM("到 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 啦啦啦啦啦\n");
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 12;
				}
			}
		}
		break;

	case 12:
		ROS_INFO_STREAM("case 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12\n");
		if (ToAngleByMPU(-170))
		{
			ROS_INFO("角度对准了，为0了！！！jiaodu jiaodu .....................:%lf\n", dsp_status.fAngle);
			Stop();
			temp_normal_time = normal_time;
			intention_num = 121;
		}
		/*if ((dsp_status.fAngle>2.0)||(dsp_status.fAngle<-2.0))
		{
			if (dsp_status.fAngle>0)
			{
				ROS_INFO_STREAM("turn left ... turn left ... turn left ... turn left ... turn left ... \n");
				ToFindObjectByturnleft();
			}
			else
			{
				ROS_INFO_STREAM("turn right ... turn right ... turn right ... turn right ... turn right ... \n");
				ToFindObjectByturnright();
			}
		}
		else
		{
			ROS_INFO_STREAM("角度对准了，为0了！！！jiaodu jiaodu .....................:%lf\n",dsp_status.fAngle);
			Stop();
			intention_num=121;
		}*/
		break;

	case 121:
		ROS_INFO_STREAM("case 121 121 121 121 121 121 121 121 121 121 121 121 121 121 121 121\n");
		if (abs(dsp_status.YDist) >= 30)
		{
			ROS_INFO("入库时Y轴没对准！！！需要左右调整！！！zuobiao zuobiao ...................: %ld, %ld\n", dsp_status.XDist, dsp_status.YDist);
			if (dsp_status.YDist > 0)
			{
				ROS_INFO_STREAM("GO left ... GO left ... GO left ... GO left ... GO left ... GO left ... \n");
				robotstraightrightlow();
			}
			else
			{
				ROS_INFO_STREAM("GO right ... GO right ... GO right ... GO right ... GO right ... GO right ... \n");
				robotstraightleftlow();
			}
		}
		else
		{
			ROS_INFO_STREAM("入库时左右对准了！！！不要动了！！！stop stop .....................................................");
			Stop();
			temp_normal_time = normal_time;
			intention_num = 124;
		}
		break;

		/*case 123:
ROS_INFO_STREAM("case 123 123 123 123 123 123 123 123 123 123 123\n");
		if (dsp_status.XDist>(-10))
		{
			ROS_INFO_STREAM("back back back back back back back back back back back back back back back back\n");
			robotbacklow();
		}
		else
		{
			ROS_INFO_STREAM("stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop \n");
			Stop();
		}
		break;*/

		/*case 123:
ROS_INFO_STREAM("case 123 123 123 123 123 123 123 123 123 123 123 123 123 \n");
		if((normal_time-temp_normal_time)>5)
		{
			Control.SendData_state=8;
			temp_normal_time=normal_time;
			intention_num=124;
		}
		break;*/

	case 124:
		ROS_INFO_STREAM("case 124 124 124 124 124 124 124 124 124 124 124 124 124 \n");
		//Control.SendData_state=0;
		if ((normal_time - temp_normal_time) > 2)
		{
			if (ToPostureByMilemeter_Pid1(ptxy))
			{
				ROS_INFO_STREAM("stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop \n");
				Stop();
			}
		}
		break;

	case 125:
		ROS_INFO_STREAM("case 125 125 125 125 125 125 125 125 125 125 125 125 125 \n");
		Stop();
		break;

	default:
		ROS_INFO_STREAM("case default default default default default default default \n");
		normal_time = 0;
		break; //**********车子归位结束**********
	}
}

void DecisionMaking::Normal_pass1_new()
{
	//一些决策相关的标志
	static int intention_num = 0; //回合过程的意图顺序号
	static bool findballflag = 0; //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
	static bool aimballflag = 0;

	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	static int vision_scan_time = 0; //视觉扫描次数统计，达到设定值时，车子要回归原位
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptOO, ptA, ptB, ptM, ptN, ptxy;

	ptOO.x = 1000;
	ptOO.y = 0; //回合结束前，用于回位定点

	ptxy.x = 0;
	ptxy.y = 0;

	ptA.x = 3200;
	ptA.y = 0; //车子带着篮球进入传球边界线内的点，车子在此处往目标点M发射球

	ptB.x = 3000;
	ptB.y = 1000; //车子在此处激光扫描球

	if (1 == place_num)
	{
		ptM.x = 875;
		ptM.y = 4300; //标定柱

		ptN.x = 3000;
		ptN.y = 2100; //投球定位点
	}
	if (2 == place_num)
	{
		ptM.x = 875;
		ptM.y = -4300; //A口进入时投球区中心位置

		ptN.x = 3000;
		ptN.y = -2100;
	}

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;

	case 120:
		if ((normal_time - temp_normal_time) > 200) //延时大约9s
		{
			temp_normal_time = normal_time;
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA)) //走到点A
		{
			ROS_INFO("我到 A 点了 我到 A 点了 我到 A 点了 ！！！X is : %ld，Y is : %ld, angle is : %lf\n", dsp_status.XDist, dsp_status.YDist, dsp_status.fAngle);
			Stop();
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 22222222222222222222222222222222222222222222222222222222222222222222222222222\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			ROS_INFO_STREAM("正在对准传球标定柱！！！！\n");
			if (ToAngleByMPU6050(ptM)) //瞄准目标点M
			{
				ROS_INFO_STREAM("已经对准了传球标定柱！！！！\n");
				if (stop())
				{
					ROS_INFO_STREAM("对准之后已经停下，可以发射了！！！！\n");
					temp_normal_time = normal_time;
					intention_num = 3;
				}
			}
		}
		break;

	case 3: //发射第一个篮球
		ROS_INFO_STREAM("case 33333333333333333333333333333333333333333333333333333333333333333333333333333\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("发射 发射 发射 ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 30;
		}
		break;

	case 30:
		ROS_INFO_STREAM("case 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 3)
		{
			ROS_INFO_STREAM("正在转回来！！！\n");
			if (ToAngleByMPU(0))
			{
				ROS_INFO_STREAM("转回来找第二个球,Stop！！！\n");
				Stop();
				temp_normal_time = normal_time;
				intention_num = 4;
			}
		}
		break;

	case 4: //**********视觉找篮球开始**********
		ROS_INFO_STREAM("case 44444444444444444444444444444444444444444444444444444444444444444444444444444\n");
		if ((normal_time - temp_normal_time) > 5)
		{
			//TODO:发送接收数据
			/*
			if(!Vision.create_communication_RAM_2())
			{
				ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
				if(!Vision.open_file_mapping_2())
				{
					ROS_INFO_STREAM("fail to open_file_mapping_2\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to open_file_mapping_2\n");
					if(!Vision.create_communication_RAM_4())
					{
						ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
						if(!Vision.open_file_mapping_4())
						{
							ROS_INFO_STREAM("fail to open_file_mapping_4\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to open_file_mapping_4\n");
							TheObject.whatObject =(objectType)3;//找什么东西在这里改
							if(!Vision.send_find_whatobject(TheObject.whatObject))
							{
								ROS_INFO_STREAM("Fail to send command\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to send command\n");
								TheObject.whatObject = (objectType)0;//unkownObject变量重新初始化，为下一次找球做准备
								if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
								{
									ROS_INFO_STREAM("Fail to receive final object result\n");
								}
								else
								{
									ROS_INFO_STREAM("Success to receive final object result\n");
									ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
									ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
								}
							}
						}
					}
				}
			}
			*/
		}
		if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				robotstraightrightlow(); //开环横向低速右移
				ROS_INFO_STREAM(" 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 \n");
				if ((dsp_status.YDist) > 3500) //找了好久，一个都没找到，就回去了
				{
					ROS_INFO_STREAM("中圈走完了，也没找到球！！！那就回家吧！！！\n");
					Stop();
					intention_num = 11;
				}
			}
			if (2 == place_num)
			{
				robotstraightleftlow(); //开环横向低速右移
				ROS_INFO_STREAM(" 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移  \n");
				if ((dsp_status.YDist) < -3500) //找了好久，一个都没找到，就回去了
				{
					ROS_INFO_STREAM("中圈走完了，也没找到球！！！那就回家吧！！！\n");
					Stop();
					intention_num = 11;
				}
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			ROS_INFO_STREAM(" cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					intention_num = 4;
					ROS_INFO("丢了 丢了 丢了 ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!中圈球:objectAngle aimedaimedaimed is %lf;中圈球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1) && (aimballflag == 1))
		{
			ROS_INFO_STREAM(" ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n");
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					intention_num = 4;
					ROS_INFO_STREAM(" 又丢了 又丢了 又丢了 ！！！\n");
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					intention_num = 5;
					ROS_INFO_STREAM("捡球 捡球 捡球 ！！！\n");
				}
			}
		}
		break; //**********视觉找篮球结束**********

	case 5:
		ROS_INFO_STREAM("case 55555555555555555555555555555555555555555555555555555555555555555555555555555\n");
		//立即捡球，不要延时
		Stop();
		//m_bSend=1;    //TODO:允许给单片机发送命令
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令，下一步立即置位0
		temp_normal_time = normal_time;
		intention_num = 6; //抬机械臂命令已发送
		break;

	case 6: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 66666666666666666666666666666666666666666666666666666666666666666666666666666\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 40)
		{
			temp_normal_time = normal_time;
			intention_num = 7;
		}
		break;

	case 7:
		ROS_INFO_STREAM("case 77777777777777777777777777777777777777777777777777777777777777777777777777777\n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中发第一次判断命令
		{
			//m_bSend = 1;				//TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			temp_normal_time = normal_time;
			intention_num = 8;
		}
		break;

	case 8:
		ROS_INFO_STREAM("case 88888888888888888888888888888888888888888888888888888888888888888888888888888\n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			temp_normal_time = normal_time;
			intention_num = 80;
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			temp_normal_time = normal_time;
			intention_num = 81;
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 80:
		ROS_INFO_STREAM("case 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80\n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 9;
		}
		break;

	case 81:
		ROS_INFO_STREAM("case 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81\n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			//m_bSend = 1;				//TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球
			temp_normal_time = normal_time;
			intention_num = 82;
		}
		break;

	case 82:
		ROS_INFO_STREAM("case 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82\n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //
		{
			temp_normal_time = normal_time;
			intention_num = 9;
		}
		else //判断两次都没球，可以再去找了
		{
			findballflag = 0;
			aimballflag = 0;
			intention_num = 4;
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 9: //确认有球后，有球，有球有球！！！！！
		ROS_INFO_STREAM("case 99999999999999999999999999999999999999999999999999999999999999999999999999999\n");
		ROS_INFO_STREAM("正在对准传球定位点！！！！\n");
		Control.SendData_state = 6;
		if (ToAngleByMPU6050(ptN)) //准备发射
		{
			ROS_INFO_STREAM("已经对准了传球定位点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停到了投球定位点，再旋转调整！！！！\n");
				Control.SendData_state = 7;
				temp_normal_time = normal_time;
				intention_num = 91;
			}
		}
		break;

	case 91:
		ROS_INFO_STREAM("case 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 \n");
		Control.SendData_state = 0;
		temp_normal_time = normal_time;
		if (ToPostureByMilemeter_Pid3(ptN)) //到达N点
		{
			Stop();
			ROS_INFO_STREAM("到 NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN啦啦啦啦啦\n");
			intention_num = 92;
		}
		break;

	case 92:
		ROS_INFO_STREAM("case 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 \n");
		ROS_INFO_STREAM("正在对准传球标定柱！！！！\n");
		if (ToAngleByMPU6050(ptM))
		{
			ROS_INFO_STREAM("已经对准了传球标定柱！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之��已经停下，可以发射了！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 10;
			}
		}
		break;

	case 10:
		ROS_INFO_STREAM("case 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			//m_bSend=1; TODO:
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("发射 发射 发射 ！！！！\n");
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 11;
		}
		break;

	case 11: //**********车子归位开始**********
		ROS_INFO_STREAM("case 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 3)
		{
			ROS_INFO_STREAM("走向 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ing\n");
			if (ToPostureByMilemeter_Pid2(ptOO)) //边走边调整角度先回位OO点，再微调退回框
			{
				ROS_INFO_STREAM("到 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 啦啦啦啦啦\n");
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 12;
				}
			}
		}
		break;

	case 12:
		ROS_INFO_STREAM("case 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12\n");
		if (ToAngleByMPU(180))
		{
			ROS_INFO("角度对准了，为0了！！！jiaodu jiaodu .....................:%lf\n", dsp_status.fAngle);
			Stop();
			intention_num = 121;
		}
		/*if ((dsp_status.fAngle>2.0)||(dsp_status.fAngle<-2.0))
		{
			if (dsp_status.fAngle>0)
			{
				ROS_INFO_STREAM("turn left ... turn left ... turn left ... turn left ... turn left ... \n");
				ToFindObjectByturnleft();
			}
			else
			{
				ROS_INFO_STREAM("turn right ... turn right ... turn right ... turn right ... turn right ... \n");
				ToFindObjectByturnright();
			}
		}
		else
		{
			ROS_INFO_STREAM("角度对准了，为0了！！！jiaodu jiaodu .....................:%lf\n",dsp_status.fAngle);
			Stop();
			intention_num=121;
		}*/
		break;

	case 121:
		ROS_INFO_STREAM("case 121 121 121 121 121 121 121 121 121 121 121 121 121 121 121 121\n");
		if (abs(dsp_status.YDist) >= 30)
		{
			ROS_INFO("入库时Y轴没对准！！！需要左右调整！！！zuobiao zuobiao ...................: %ld, %ld\n", dsp_status.XDist, dsp_status.YDist);
			if (dsp_status.YDist > 0)
			{
				ROS_INFO_STREAM("GO left ... GO left ... GO left ... GO left ... GO left ... GO left ... \n");
				robotstraightright();
			}
			else
			{
				ROS_INFO_STREAM("GO right ... GO right ... GO right ... GO right ... GO right ... GO right ... \n");
				robotstraightleft();
			}
		}
		else
		{
			ROS_INFO_STREAM("入库时左右对准了！！！不要动了！！！stop stop .....................................................");
			Stop();
			intention_num = 124;
		}
		break;

		/*case 122:
ROS_INFO_STREAM("case 122 122 122 122 122 122 122 122 122 122 122 122 122 122 122 122\n");
		if ((dsp_status.fAngle>0.5)||(dsp_status.fAngle<-0.5))
		{
			if (dsp_status.fAngle>0)
			{
				ROS_INFO_STREAM("turn left ... turn left ... turn left ... turn left ... turn left ... \n");
				ToFindObjectByturnleft();
			}
			else
			{
				ROS_INFO_STREAM("turn right ... turn right ... turn right ... turn right ... turn right ... \n");
				ToFindObjectByturnright();
			}
		}
		else
		{
			ROS_INFO_STREAM("角度再一次对准了，为0了！！！jiaodu jiaodu ................................................\n");
			Stop();
			intention_num=123;
		}
		break;

	case 123:
ROS_INFO_STREAM("case 123 123 123 123 123 123 123 123 123 123 123\n");
		if (dsp_status.XDist>(-10))
		{
			ROS_INFO_STREAM("back back back back back back back back back back back back back back back back\n");
			robotbacklow();
		}
		else
		{
			ROS_INFO_STREAM("stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop \n");
			Stop();
		}
		break;*/

	case 124:
		ROS_INFO_STREAM("case 124 124 124 124 124 124 124 124 124 124 124 124 124 \n");
		if (ToPostureByMilemeter_Pid2(ptxy))
		{
			ROS_INFO_STREAM("stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop \n");
			Stop();
		}

	default:
		normal_time = 0;
		break; //**********车子归位结束**********
	}
}

void DecisionMaking::Normal_pass2() //**********传球-回合2**********
{
	//一些决策相关的标志
	static int intention_num = 0; //回合过程的意图顺序号

	static int armupflag = 0; //抬臂标志
	static int stopflag = 0;  //停车标志

	static bool findballflag = 0;			 //找到目标标志位,每找完一次目标后都要记得清零
	static bool aimballflag = 0;			 //对准目标标志位，每对准一次目标后都要记得清零
	static bool Iscircleavoidancecorner = 0; //中圈避障是否从边缘靠近了
	static bool Isthreescoreabs45ready = 0;	 //三分线角度调整完成判断

	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;					 //DSP 45ms发送一次数据

	//这些点如果需要控件改变值，需要定义在成员变量，并在初始化函数里通过控件得到初始化值
	Posture ptOO, ptA0, ptC, ptM1, ptM, ptN, ptZ, ptxy, ptT; //ptZ时中圈进口有障碍球时临时确定的辅助点
															 //终点前，用于回位定点

	if (1 == place_num)
	{
		ptOO.x = 1200;
		ptOO.y = 0;

		ptA0.x = 3350; //（用向右偏的方法是2500）	//这个地方开始判断中圈进口是否有障碍球
		ptA0.y = -800;

		ptxy.x = -400;
		ptxy.y = 400;

		ptC.x = 3650; //三分线中点识别球定点处
		ptC.y = 4800;

		ptM1.x = 500;
		ptM1.y = 5000; //第一次传球标定柱位置

		ptM.x = 875;
		ptM.y = 4300; //传球区中心位置（第二次）

		/*ptN.x = 3000;
		ptN.y = 2100;*/
		ptN.x = 2400;
		ptN.y = 1200;

		ptZ.x = 2800;
		ptZ.y = 4000; //归位辅助点

		ptT.x = 2600;
		ptT.y = 4800;
	}
	if (2 == place_num)
	{
		ptOO.x = 1200;
		ptOO.y = -200;

		ptA0.x = 3300; //（用向右偏的方法是2500）	//这个地方开始判断中圈进口是否有障碍球
		ptA0.y = 800;

		ptxy.x = 150;
		ptxy.y = -150;

		ptC.x = 3650; //三分线球点
		ptC.y = -4600;

		ptM1.x = 500;
		ptM1.y = -4770; //第一次传球标定柱位置

		ptM.x = 875;
		ptM.y = -4300; //传球区中心位置（第二次）

		ptN.x = 2400;
		ptN.y = -1200; //投球定位点

		ptZ.x = 2800;
		ptZ.y = -4000; //归位辅助点

		ptT.x = 2540;
		ptT.y = -4800;
	}
	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 00000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;
	case 120:
		if ((normal_time - temp_normal_time) > 200) //延时17.1s
		{
			temp_normal_time = normal_time;
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid2(ptA0)) //从起点O到点A0
		{
			ROS_INFO("我到 A 点了 我到 A 点了 我到 A 点了 ！！！X is : %ld，Y is : %ld, angle is : %lf\n", dsp_status.XDist, dsp_status.YDist, dsp_status.fAngle);
			Stop();
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 222222222222222222222222222222222222222222222222222222222222222222222222222222\n");
		Control.SendData_state = 6;
		if (1 == place_num)
		{
			if (fabs(dsp_status.fAngle) > 2) //调整角度归零
			{
				ROS_INFO("调整角度到零 调整角度到零 调整角度到零 ！！！X is:%ld，Y is:%ld, angle is : %lf\n", dsp_status.XDist, dsp_status.YDist, dsp_status.fAngle);
				ToAngleByMPU(-1);
			}
			else
			{
				ROS_INFO("角度已经归零 ！！！X is:%ld，Y is:%ld, angle is : %lf\n", dsp_status.XDist, dsp_status.YDist, dsp_status.fAngle);
				Control.SendData_state = 7;
				temp_normal_time = normal_time;
				intention_num = 3;
			}
		}
		if (2 == place_num)
		{
			if (fabs(dsp_status.fAngle) > 2) //调整角度归零
			{
				ROS_INFO("调整角度到零 调整角度到零 调整角度到零 ！！！X is:%ld，Y is:%ld, angle is : %lf\n", dsp_status.XDist, dsp_status.YDist, dsp_status.fAngle);
				ToAngleByMPU(-6);
			}
			else
			{
				ROS_INFO("角度已经归零 ！！！X is:%ld，Y is:%ld, angle is : %lf\n", dsp_status.XDist, dsp_status.YDist, dsp_status.fAngle);
				Control.SendData_state = 7;
				temp_normal_time = normal_time;
				intention_num = 3;
			}
		}
		break;

		/********************************** 中圈识别球 ***********************************/
	case 3: //开启识别
		ROS_INFO_STREAM("case 333333333333333333333333333333333333333333333333333333333333333333\n");
		Control.SendData_state = 0;
		//TODO:发送接收数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
						TheObject.whatObject =(objectType)3;//找什么东西在这里改
						if(TheObject.whatObject==(objectType)1)
							ROS_INFO_STREAM("send command to find calibration\n");
						if(TheObject.whatObject==(objectType)2)
							ROS_INFO_STREAM("send command to find reddish_brown_basketball\n");
						if(TheObject.whatObject==(objectType)3)
							ROS_INFO_STREAM("send command to find basketball\n");
						if(TheObject.whatObject==(objectType)4)
							ROS_INFO_STREAM("send command to find blue_gray_basketball\n");
						if(TheObject.whatObject==(objectType)5)
							ROS_INFO_STREAM("send command to find calibration_plus\n");
						if(TheObject.whatObject==(objectType)6)
							ROS_INFO_STREAM("send command to find orange_red_volleyball\n");
						if(TheObject.whatObject==(objectType)7)
							ROS_INFO_STREAM("send command to find red_blue_volleyball\n");
						if(TheObject.whatObject==(objectType)8)
							ROS_INFO_STREAM("send command to find volleyball\n");

						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to send command\n");
							TheObject.whatObject = (objectType)0;//unkownObject变量重新初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}
		*/

		if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				robotstraightright(); //开环横向低速右移
				ROS_INFO_STREAM(" 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 \n");
				if ((dsp_status.YDist) > 3500) //找了好久，一个都没找到，就回去了
				{
					ROS_INFO_STREAM("中圈走完了，也没找到球！！！那就回家吧！！！\n");
					Stop();
					temp_normal_time = normal_time;
					intention_num = 85;
				}
			}
			if (2 == place_num)
			{
				robotstraightleft(); //开环横向低速右移
				ROS_INFO_STREAM(" 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 \n");
				if ((dsp_status.YDist) < -3500) //找了好久，一个都没找到，就回去了
				{
					ROS_INFO_STREAM("中圈走完了，也没找到球！！！那就回家吧！！！\n");
					Stop();
					temp_normal_time = normal_time;
					intention_num = 85;
				}
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM(" find_objectflag find_objectflag find_objectflag find_objectflag find_objectflag find_objectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			ROS_INFO_STREAM(" cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
			if (ToAimBallByVision()) //是停下的时候通过摄像头返回的角度偏差对准球
			{
				ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{
					findballflag = 0;
					aimballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 3; //找球的时候，丢球，再去找
					ROS_INFO("丢了 丢了 丢了 ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("！！！ 中圈球找到了 中圈球找到了 中圈球找到了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1) && (armupflag == 0))
		{
			ROS_INFO_STREAM(" ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n");
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					ROS_INFO_STREAM(" can not find now...can not find now...can not find now...can not find now...\n");
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 3;
					ROS_INFO_STREAM(" 又丢了 又丢了 又丢了 ！！！\n");
				}
				else
				{
					armupflag = 1; //抬臂标识位置1
					ROS_INFO_STREAM("!!!!!!!!!!I have walked to the ball...I have walked to the ball...I have walked to the ball...I have walked to the ball!!!!!!!!!!\n");
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 4;
					ROS_INFO_STREAM("捡球 捡球 捡球 ！！！\n");
				}
			}
		}
		break;

	case 4:
		ROS_INFO_STREAM("case 444444444444444444444444444444444444444444444444444444444444444444444444444444444\n");
		temp_normal_time = normal_time; //为延时作准备
		if ((armupflag == 1) && (stopflag == 0))
		{
			Control.SendData_state = 1; //执行抬臂动作
			ROS_INFO_STREAM("case uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuppppppppppppppppppppppppppppppppppppp\n");
			temp_normal_time = normal_time;
			//stopflag=1;		//停止标识位置1
			intention_num = 5;
		}
		break;

	case 5: //判断是否捡到球
		ROS_INFO_STREAM("case 55555555555555555555555555555555555555555555555555555555555555555555555555555\n");
		Control.SendData_state = 0;
		if ((normal_time - temp_normal_time) > 40) //判断是否捡上来，2秒足够了，不管他放不放下
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一位立即置位0
			ROS_INFO_STREAM("第一次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 6;
		}
		break;

	case 6:
		ROS_INFO_STREAM("case 6666666666666666666666666666666666666666666666666666666666666666666666666666\n");
		Control.SendData_state = 0;
		if (('y' == dsp_status.RecData_state) || ('n' == dsp_status.RecData_state))
		{
			ROS_INFO_STREAM("第一次，判断架子上有没有球！！！！\n");
			if ('y' == dsp_status.RecData_state)
			{
				temp_normal_time = normal_time;
				intention_num = 80;
				ROS_INFO_STREAM("发现已经捡到球了，架子上有球！！！\n");
			}
			else //如果没捡到球，放下机械臂，重新捡球
			{
				findballflag = 0;
				aimballflag = 0;
				temp_normal_time = normal_time;
				intention_num = 81;
				ROS_INFO_STREAM("判断没有球啊！等待抬臂稳定后在判断一遍！！！\n");
			}
		}
		else
		{
			Control.SendData_state = 5;
			temp_normal_time = normal_time;
			intention_num = 6;
			ROS_INFO_STREAM("既没有收到Y也没有收到N，没接受到？还是没法出去？不管了，再问一遍\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 80:
		ROS_INFO_STREAM("case 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80\n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 820;
		}
		break;

	case 81:
		ROS_INFO_STREAM("case 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81\n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("第二次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 82;
		}
		break;

	case 82:
		ROS_INFO_STREAM("case 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 \n");
		Control.SendData_state = 0; //单片机位置0
		ROS_INFO_STREAM("第二次，判断架子上有没有球！！！！\n");
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("发现有球啊！！！第一次误判！！！\n");
			temp_normal_time = normal_time;
			intention_num = 820;
		}
		else //判断两次都没球，可以再去找了
		{
			findballflag = 0;
			aimballflag = 0;
			temp_normal_time = normal_time;
			intention_num = 3;
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

		/*case 820:                        
ROS_INFO_STREAM("case 820 820 820 820 820 820 820 820 820 820 820 820 820 820 820 820 820 820 \n");				
		ROS_INFO_STREAM("正在对准传球定位点！！！！\n");
		Control.SendData_state=6;
		if(ToAngleByMPU6050(ptN))
		{
			ROS_INFO_STREAM("已经对准了传球定位点！！！！\n");
			if(stop())
			{
				ROS_INFO_STREAM("对准之后已经停到了投球定位点，再旋转调整！！！！\n");
				Control.SendData_state=7;
				temp_normal_time=normal_time;
				intention_num=821;
			}
		}
		break;*/

	case 820:
		ROS_INFO_STREAM("case 820 821 821 821 821 821 821 821 821 821 821 821 820 820 820 820 820 820 \n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid2(ptN)) //到达N点
		{
			Stop();
			ROS_INFO_STREAM("刚到 NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN啦啦啦啦啦\n");
			intention_num = 821;
		}
		break;

	case 821:
		ROS_INFO_STREAM("case 821 821 821 821 821 821 821 821 821 821 821 821 820 820 820 820 820 820 \n");
		if (ToPostureByMilemeter_Pid6(ptN)) //到达N点
		{
			Stop();
			ROS_INFO_STREAM("到 NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN啦啦啦啦啦\n");
			intention_num = 83;
		}
		break;

	case 83:
		ROS_INFO_STREAM("case 83 83 83 83 83 83 83 83 83 83 83 83 83 83 83 83 83 83 83 83 83 83 \n");
		ROS_INFO_STREAM("正在对准传球标定柱！！！！\n");
		Control.SendData_state = 6;
		if (ToAngleByMPU6050(ptM1))
		{
			ROS_INFO_STREAM("已经对准了传球标定柱！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，可以发射了！！！！\n");
				Control.SendData_state = 7;
				temp_normal_time = normal_time;
				intention_num = 84;
			}
		}
		break;

	case 84: //发射
		ROS_INFO_STREAM("case 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 \n");
		Control.SendData_state = 0;
		if ((normal_time - temp_normal_time) > 2)
		{
			Control.SendData_state = 2; //发送发射指令,下一步立即置位0
			ROS_INFO_STREAM("发射 发射 发射 ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 85;
		}
		break;

	case 85: //转向目的点，去找三分球
		ROS_INFO_STREAM("case 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 \n");
		ROS_INFO_STREAM("调整角度，对准三分线目标点——调整角度，对准三分线目标点——调整角度，对准三分线目标点 ！！！！\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 3)
		{
			Control.SendData_state = 6;
			if (ToAngleByMPU6050(ptC))
			{
				ROS_INFO_STREAM("对准了 对准了！！！！\n");
				if (stop())
				{
					ROS_INFO_STREAM("我停下了！！！！\n");
					Control.SendData_state = 7;
					temp_normal_time = normal_time;
					intention_num = 87;
				}
			}
		}
		break;

		//	case 86:
		//ROS_INFO_STREAM("case 86 86 86 86 86 86 86 86 86 86 86 86 86 86 86 86 86 86 86 86 \n");
		//		ROS_INFO_STREAM("避障程序，走向三分点——避障程序，走向三分点——避障程序，走向三分点！！！！\n");
		//		Control.SendData_state=0;
		//		if(ToPostureByAvoidance(ptC))//避障
		//		{
		//			ROS_INFO_STREAM("避障完成了 避障完成了 避障完成了！！！！\n");
		//			Stop();
		//			temp_normal_time=normal_time;
		//			intention_num=860;
		//		}
		//		ROS_INFO_STREAM("避障和延时之间！！！！\n");
		//		if((normal_time-temp_normal_time)>100)
		//		{
		//			ROS_INFO_STREAM("避障延时180之内！！！！\n");
		//			Stop();
		//			temp_normal_time=normal_time;
		//			intention_num=861;
		//		}
		//		break;
		//
		//	case 860:
		//ROS_INFO_STREAM("case 810 810 810 810 810 810 810 810 810 810 810 810 810 810 810 810\n");
		//		if((normal_time-temp_normal_time)>33)//以开环速度向前2s
		//		{
		//			robotforward();
		//			ROS_INFO_STREAM("开环往前走2秒！！！！\n");
		//		}
		//		else
		//		{
		//			temp_normal_time=normal_time;
		//			intention_num=861;
		//		}
		//		break;
		//
		//	case 861://避障后再次对准目标点
		//ROS_INFO_STREAM("case 861 861 861 861 861 861 861 861 861 861 861 861 861 861 861 861 861 861 \n");
		//
		//		if((normal_time-temp_normal_time)>2)
		//		{
		//			Control.SendData_state=6;
		//			if(ToAngleByMPU6050(ptC))
		//			{
		//				ROS_INFO_STREAM("避障后再次对准目标点！！！！\n");
		//				if(stop())
		//				{
		//					Control.SendData_state=7;
		//					temp_normal_time=normal_time;
		//					intention_num=87;
		//				}
		//			}
		//		}
		//		break;

	case 87:
		ROS_INFO_STREAM("case 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 \n");
		Control.SendData_state = 0;
		ROS_INFO_STREAM("避障之后，再次对准，ToPostureByMilemeter函数，走向C点！！！！\n");
		if (ToPostureByMilemeter_Pid3(ptC))
		{
			ROS_INFO_STREAM("到达三分点！！！！\n");
			Stop();
			temp_normal_time = normal_time;
			intention_num = 870;
		}
		break;

	case 870:
		ROS_INFO_STREAM("case 870 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 \n");
		if (ToPostureByMilemeter_Pid6(ptC))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 88;
		}
		break;

	case 88:
		ROS_INFO_STREAM("case 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (1 == place_num)
			{
				ROS_INFO_STREAM("角度调整过程ing................\n");
				if (fabs(dsp_status.fAngle - 63.00) > 2)
				{
					ROS_INFO_STREAM("和66度相差大于2度................\n");
					if (dsp_status.fAngle > 65)
					{
						ROS_INFO_STREAM("角度为正，需要左旋................\n");
						ToFindObjectByturnleft();
					}
					if (dsp_status.fAngle < 61)
					{
						ROS_INFO_STREAM("角度为负，需要右旋................\n");
						ToFindObjectByturnright();
					}
				}
				else
				{
					ROS_INFO_STREAM("角度为63度，不要动了！！！................\n");
					temp_normal_time = normal_time;
					intention_num = 90;
				}
			}
			if (2 == place_num)
			{
				ROS_INFO_STREAM("角度调整过程ing................\n");
				if (fabs(dsp_status.fAngle - (-66.00)) > 2)
				{
					ROS_INFO_STREAM("和-66度相差大于2度................\n");
					if (dsp_status.fAngle > -64)
					{
						ROS_INFO_STREAM("角度为正，需要左旋................\n");
						ToFindObjectByturnleft();
					}
					if (dsp_status.fAngle < -68)
					{
						ROS_INFO_STREAM("角度为负，需要右旋................\n");
						ToFindObjectByturnright();
					}
				}
				else
				{
					ROS_INFO_STREAM("角度为-63度，不要动了！！！................\n");
					temp_normal_time = normal_time;
					intention_num = 90;
				}
			}
		}
		break;

		/********************************** 三分线识别球 ***********************************/
	case 90: //拾球（当视觉识别出目标即视为到达点，开始使用视觉返回的距离与角度）
		ROS_INFO_STREAM("case 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 \n");
		if ((normal_time - temp_normal_time) > 5)
		{
			//TODO:接受发送数据
			/*
			if(!Vision.create_communication_RAM_2())
			{
				ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
				if(!Vision.open_file_mapping_2())
				{
					ROS_INFO_STREAM("fail to open_file_mapping_2\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to open_file_mapping_2\n");
					if(!Vision.create_communication_RAM_4())
					{
						ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
						if(!Vision.open_file_mapping_4())
						{
							ROS_INFO_STREAM("fail to open_file_mapping_4\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to open_file_mapping_4\n");
							TheObject.whatObject =(objectType)3;//找什么东西在这里改
							if(TheObject.whatObject==(objectType)1)
								ROS_INFO_STREAM("send command to find calibration\n");
							if(TheObject.whatObject==(objectType)2)
								ROS_INFO_STREAM("send command to find reddish_brown_basketball\n");
							if(TheObject.whatObject==(objectType)3)
								ROS_INFO_STREAM("send command to find basketball\n");
							if(TheObject.whatObject==(objectType)4)
								ROS_INFO_STREAM("send command to find blue_gray_basketball\n");
							if(TheObject.whatObject==(objectType)5)
								ROS_INFO_STREAM("send command to find calibration_plus\n");
							if(TheObject.whatObject==(objectType)6)
								ROS_INFO_STREAM("send command to find orange_red_volleyball\n");
							if(TheObject.whatObject==(objectType)7)
								ROS_INFO_STREAM("send command to find red_blue_volleyball\n");
							if(TheObject.whatObject==(objectType)8)
								ROS_INFO_STREAM("send command to find volleyball\n");

							if(!Vision.send_find_whatobject(TheObject.whatObject))
							{
								ROS_INFO_STREAM("Fail to send command\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to send command\n");
								TheObject.whatObject = (objectType)0;//unkownObject变量重新初始化，为下一次找球做准备
								if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
								{
									ROS_INFO_STREAM("Fail to receive final object result\n");
								}
								else
								{
									ROS_INFO_STREAM("Success to receive final object result\n");
									ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
									ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
								}
							}
						}
					}
				}
				
			}
			*/
			if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
			{
				findballflag = 0;
				aimballflag = 0;
				if (1 == place_num)
				{
					if (abs(dsp_status.XDist - 1000) > 50)
					{
						ROS_INFO_STREAM("66度角，向右横向移动................\n");
						robotstraightright();
						temp_normal_time = normal_time;
					}
					else
					{
						ROS_INFO_STREAM("到边界了，没找到！！！................\n");
						Stop();
						if (ToAngleByMPU(90))
						{
							ROS_INFO_STREAM("边界右转结束 ！！！................\n");
							if (normal_time - temp_normal_time < 120)
							{
								ROS_INFO_STREAM("边界前进 ！！！................\n");
								robotforwardtoolow();
							}
							else
							{
								ROS_INFO_STREAM("边界前进，停止 ！！！................\n");
								Stop();
								temp_normal_time = normal_time;
								intention_num = 1011;
							}
						}
					}
				}
				if (2 == place_num)
				{
					if (abs(dsp_status.XDist - 1000) > 50)
					{
						ROS_INFO_STREAM("-66度角，向左横向移动................\n");
						robotstraightleft();
						temp_normal_time = normal_time;
					}
					else
					{
						ROS_INFO_STREAM("到边界了，没找到！！！................\n");
						Stop();
						if (normal_time - temp_normal_time < 66)
						{
							ROS_INFO_STREAM("边界左转 ！！！................\n");
							ToFindObjectByturnleft();
						}
						else
						{
							temp_normal_time = normal_time;
							intention_num = 1011;
						}
					}
				}
			}
			else
			{
				findballflag = 1;
				ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
			}
			if ((findballflag == 1) && (aimballflag == 0))
			{

				ROS_INFO_STREAM(" cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
				if (ToAimBallByVision())
				{
					ROS_INFO_STREAM(" eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee\n");
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //对准球的过程中突然丢了
						findballflag = 0;
						aimballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 90;
						ROS_INFO_STREAM("对准球的过程中突然丢了！！！................\n");
					}
					else
					{
						aimballflag = 1;
						ROS_INFO("!!!!!!!!三分球:objectAngle aimedaimedaimed is %lf;三分球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					}
				}
			}
			if ((aimballflag == 1))
			{
				ROS_INFO_STREAM(" ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n");
				if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
				{
					ROS_INFO_STREAM("前往球的位置 前往球的位置 前往球的位置 ！！！................\n");
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //前往球的跟前过程中，又没看到球，重新去找
						ROS_INFO_STREAM("又丢了 又丢了 又丢了 ！！！................\n");
						aimballflag = 0;
						findballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 90;
					}
					else
					{
						ROS_INFO_STREAM("可以捡球了 可以捡球了 可以捡球了 ！！！................\n");
						aimballflag = 0;
						findballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 91;
					}
				}
			}
		}
		break;
	case 91:
		ROS_INFO_STREAM("case 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 \n");
		if (normal_time - temp_normal_time > 2)
		{
			Stop();						//停车并立即捡球
			Control.SendData_state = 1; //球已持住，发送抬机械臂命令，下一步立即置位0
			ROS_INFO_STREAM("发送抬臂命令！！！................\n");
			temp_normal_time = normal_time;
			intention_num = 93; //抬机械臂命令已发送
		}
		break;
	case 93: //等待发送抬臂指令收到停车，等待机械臂抬起,然后落下
		ROS_INFO_STREAM("case 93 93 93 93 93 93 93 93 93 93 93 93 93 93 93 93 93 93 93 93 \n");
		Control.SendData_state = 0; //单片机位置0
		ROS_INFO_STREAM("等待抬杆放下！！！................\n");
		if ((normal_time - temp_normal_time) > 40) //1.5s时间抬臂、放臂
		{
			temp_normal_time = normal_time;
			intention_num = 94;
		}
		break;
	case 94:
		ROS_INFO_STREAM("case 94 94 94 94 94 94 94 94 94 94 94 94 94 94 94 94 94 94 94 94 \n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中就发第一次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			ROS_INFO_STREAM("有没有球！！！发送判断指令！！！................\n");
			temp_normal_time = normal_time;
			intention_num = 95;
		}
		break;
	case 95:
		ROS_INFO_STREAM("case 95 95 95 95 95 95 95 95 95 95 95 95 95 95 95 95 95 95 95 95 95 \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			ROS_INFO_STREAM("抬臂过程中就检测到有球！！！................\n");
			temp_normal_time = normal_time;
			intention_num = 950;
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			ROS_INFO_STREAM("抬臂过程中没看到，准备持球机构稳定了再判断一次！！！................\n");
			temp_normal_time = normal_time;
			intention_num = 951;
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;
	case 950:
		ROS_INFO_STREAM("case 950 950 950 950 950 950 950 950 950 950 950 950 950 950 950 950 \n");
		ROS_INFO_STREAM("等待持球和球机构稳定！！！................\n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 960;
		}
		break;
	case 951:
		ROS_INFO_STREAM("case 951 951 951 951 951 951 951 951 951 951 951 951 951 951 951 951 \n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("有没有球！！！第二次发送判断指令！！！................\n");
			temp_normal_time = normal_time;
			intention_num = 96;
		}
		break;
	case 96:
		ROS_INFO_STREAM("case 96 96 96 96 96 96 96 96 96 96 96 96 96 96 96 96 96 96 96 96 96 \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("第二次，抬臂过程中就检测到有球！！！................\n");
			temp_normal_time = normal_time;
			intention_num = 960;
		}
		else //判断两次都没球，可以再去找了
		{
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！................\n");
			findballflag = 0;
			aimballflag = 0;
			temp_normal_time = normal_time;
			intention_num = 90;
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 960:
		ROS_INFO_STREAM("case 960 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82\n");
		if ((normal_time - temp_normal_time) < 10)
		{
			robotbacklow();
		}
		else
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 961;
		}
		break;

	case 961:
		ROS_INFO_STREAM("case 961 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82\n");
		if (ToPostureByMilemeter_Pid6(ptT))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 962;
		}
		break;

	case 962:
		ROS_INFO_STREAM("case 962 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82\n");
		if (ToPostureByMilemeter_Pid2(ptT))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 963;
		}
		break;

	case 963: //瞄准，准备发射篮球
		ROS_INFO_STREAM("case 963 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101\n");
		ROS_INFO_STREAM("正在对准传球目标点！！！！\n");
		Control.SendData_state = 6; //关闭里程计
		if (ToAngleByMPU6050(ptM))
		{
			ROS_INFO_STREAM("已经对准了传球目标点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，准备弹射！！！！\n");
				Control.SendData_state = 7; //旋转停稳之后，再打开里程计
				temp_normal_time = normal_time;
				intention_num = 964;
			}
		}
		break;

	case 964:
		ROS_INFO_STREAM("case 964 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101\n");
		Control.SendData_state = 0;
		if ((normal_time - temp_normal_time) > 2)
		{
			if (abs(dsp_status.XDist - ptT.x) > 50)
			{
				if ((dsp_status.XDist - ptT.x) > 50)
					robotforwardtoolow();
				else
					robotbacktoolow();
			}
			else
			{
				Stop();
				temp_normal_time = normal_time;
				intention_num = 99;
			}
		}
		break;

		/*case 97:
ROS_INFO_STREAM("case 97 97 97 97 97 97 97 97 97 97 97 97 97 97 97 97 97 97 97 97 97 97 \n");
		ROS_INFO_STREAM("对准传球标定柱ing ！！！................\n");
		Control.SendData_state=6;
		if(ToAngleByMPU6050(ptM))
		{
			if(stop())
			{
				ROS_INFO_STREAM("对准，停下！！！................\n");
				Control.SendData_state=7;
				temp_normal_time=normal_time;
				intention_num=98;
			}							
		}			
		break;

	case 98:
ROS_INFO_STREAM("case 98 98 98 98 98 98 98 98 98 98 98 98 98 98 98 98 98 98 98 98 98 98 98 98  \n");	
		Control.SendData_state=0;
		if((normal_time-temp_normal_time)<22)
		{
			ROS_INFO_STREAM("前进，走出三分线！！！ \n");	
			robotforwardtoolow();		
		}else
		{
			temp_normal_time=normal_time;
			intention_num=99;
		}
		break;*/

	case 99:
		ROS_INFO_STREAM("case 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 \n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (dsp_status.XDist > 3600)
			{
				Control.SendData_state = 2; //发送发射指令，下一步立即置位0 //gaile gaile
				ROS_INFO_STREAM("发射 发射 发射 ！！！................\n");
				temp_normal_time = normal_time;
				intention_num = 111;
			}
			else
			{
				Control.SendData_state = 4; //发送发射指令，下一步立即置位0 //gaile gaile
				ROS_INFO_STREAM("发射 发射 发射 ！！！................\n");
				temp_normal_time = normal_time;
				intention_num = 1011;
			}
		}
		break;

		/************************************************  归位 归位 归位 ******************************************************/

	case 1011:
		ROS_INFO_STREAM("case 1011 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 \n");
		Control.SendData_state = 0;
		if ((normal_time - temp_normal_time) > 3)
		{
			if (ToPostureByMilemeter_Pid2(ptZ))
			{
				Stop();
				temp_normal_time = normal_time;
				intention_num = 110;
			}
		}
		break;

	case 110: //旋转对准pt00点
		ROS_INFO_STREAM("case 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 \n");

		if ((normal_time - temp_normal_time) > 3)
		{
			ROS_INFO_STREAM("走向 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ing\n");
			Control.SendData_state = 6;
			if (ToAngleByMPU6050(ptOO)) //先转向00
			{
				ROS_INFO_STREAM("到 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 啦啦啦啦啦\n");
				if (stop())
				{
					Control.SendData_state = 7;
					temp_normal_time = normal_time;
					intention_num = 111;
				}
			}
		}
		break;

	case 111: //旋转对准pt00点
		ROS_INFO_STREAM("case 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 3)
		{
			ROS_INFO_STREAM("走向 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ing\n");
			if (ToPostureByMilemeter_Pid2(ptOO)) //走向00
			{
				ROS_INFO_STREAM("到 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 啦啦啦啦啦\n");
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 220;
				}
			}
		}
		break;

	case 220:
		ROS_INFO_STREAM("case 220 220 220 220 220 220 220 220 220 220 220 220 220 220 220 220 220 \n");
		Control.SendData_state = 6;
		if ((normal_time - temp_normal_time) > 10)
		{
			if (1 == place_num)
			{
				if (ToAngleByMPU(-4))
				{
					ROS_INFO("角度对准了，为0了！！！jiaodu jiaodu .....................:%lf\n", dsp_status.fAngle); //就这里，总是为0
					Stop();
					Control.SendData_state = 7;
					temp_normal_time = normal_time;
					intention_num = 330;
				}
			}
			if (2 == place_num)
			{
				if (ToAngleByMPU(0))
				{
					ROS_INFO("角度对准了，为0了！！！jiaodu jiaodu .....................:%lf\n", dsp_status.fAngle); //就这里，总是为0
					Stop();
					Control.SendData_state = 7;
					temp_normal_time = normal_time;
					intention_num = 330;
				}
			}
		}
		break;

	case 330:
		ROS_INFO_STREAM("case 330 330 330 330 330 330 330 330 330 330 330 330 330 330 330 330 330 \n");
		Control.SendData_state = 0;
		if ((normal_time - temp_normal_time) > 5)
		{
			if (1 == place_num)
			{
				if (abs(dsp_status.YDist - 200) >= 30)
				{
					ROS_INFO("入库时Y轴没对准！！！需要左右调整！！！zuobiao zuobiao ...................: %ld, %ld\n", dsp_status.XDist, dsp_status.YDist);
					if ((dsp_status.YDist - 200) > 0)
					{
						ROS_INFO_STREAM("GO left ... GO left ... GO left ... GO left ... GO left ... GO left ... \n");
						robotstraightleftlow();
					}
					else
					{
						ROS_INFO_STREAM("GO right ... GO right ... GO right ... GO right ... GO right ... GO right ... \n");
						robotstraightrightlow();
					}
				}
				else
				{
					ROS_INFO_STREAM("入库时左右对准了！！！不要动了！！！stop stop .....................................................");
					Stop();
					temp_normal_time = normal_time;
					intention_num = 333;
				}
			}
			if (2 == place_num)
			{
				if (abs(dsp_status.YDist - 200) >= 30)
				{
					ROS_INFO("入库时Y轴没对准！！！需要左右调整！！！zuobiao zuobiao ...................: %ld, %ld\n", dsp_status.XDist, dsp_status.YDist);
					if ((dsp_status.YDist - 200) > 0)
					{
						ROS_INFO_STREAM("GO left ... GO left ... GO left ... GO left ... GO left ... GO left ... \n");
						robotstraightleftlow();
					}
					else
					{
						ROS_INFO_STREAM("GO right ... GO right ... GO right ... GO right ... GO right ... GO right ... \n");
						robotstraightrightlow();
					}
				}
				else
				{
					ROS_INFO_STREAM("入库时左右对准了！！！不要动了！！！stop stop .....................................................");
					Stop();
					temp_normal_time = normal_time;
					intention_num = 333;
				}
			}
		}
		break;

	case 333:
		ROS_INFO_STREAM("case 333 333 333 333 333 333 333 333 333 333 333 333 333 333 333 333 333 333  \n");
		if ((normal_time - temp_normal_time) > 10)
		{
			if (1 == place_num)
			{
				if (ToAngleByMPU(-4))
				{
					ROS_INFO("角度对准了，为0了！！！jiaodu jiaodu .....................:%lf\n", dsp_status.fAngle); //就这里，总是为0
					Stop();
					temp_normal_time = normal_time;
					intention_num = 550;
				}
			}
			if (2 == place_num)
			{
				if (ToAngleByMPU(-3))
				{
					ROS_INFO("角度对准了，为0了！！！jiaodu jiaodu .....................:%lf\n", dsp_status.fAngle); //就这里，总是为0
					Stop();
					temp_normal_time = normal_time;
					intention_num = 550;
				}
			}
		}

		break;

		/*case 440:
ROS_INFO_STREAM("case 440 440 440 440 440 440 440 440 440 440 440 440 440 440 440 440 \n");
		if((normal_time-temp_normal_time)>5)
		{
			Control.SendData_state=8;
			temp_normal_time=normal_time;
			intention_num=550;
		}
		break;*/

	case 550:
		ROS_INFO_STREAM("case 550 550 550 550 550 550 550 550 550 550 550 550 550 550 550 550 \n");
		//Control.SendData_state=0;
		if ((normal_time - temp_normal_time) > 5)
		{
			if (1 == place_num)
			{
				if (abs(dsp_status.XDist - 100) < 10)
				{
					ROS_INFO_STREAM("stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop \n");
					Stop();
					intention_num = 444;
				}
				else
				{
					ROS_INFO_STREAM("我在后退！！\n");
					robotbacklow();
				}
			}
			if (2 == place_num)
			{
				/*if(ToPostureByMilemeter_Pid2(ptxy))
				{
					ROS_INFO_STREAM("stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop \n");
					Stop();
					intention_num=444;
				}*/
				if (abs(dsp_status.XDist - 300) < 10)
				{
					ROS_INFO_STREAM("stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop \n");
					Stop();
					intention_num = 444;
				}
				else
				{
					ROS_INFO_STREAM("我在后退！！\n");
					robotbacklow();
				}
			}
		}
		break;

	case 444:
		ROS_INFO_STREAM("case 444 444 444 444 444 444 444 444 444 444 444 444 444 444 444 444 444 \n");
		ROS_INFO_STREAM("停下了！！！ \n");
		Stop();
		break;

	default:
		break;
	}
}

void DecisionMaking::Normal_pass2_new()
{
	//一些决策相关的标志
	static int intention_num = 0; //回合过程的意图顺序号

	static int armupflag = 0; //抬臂标志
	static int stopflag = 0;  //停车标志

	static bool findballflag = 0;			 //找到目标标志位,每找完一次目标后都要记得清零
	static bool aimballflag = 0;			 //对准目标标志位，每对准一次目标后都要记得清零
	static bool Iscircleavoidancecorner = 0; //中圈避障是否从边缘靠近了
	static bool Isthreescoreabs45ready = 0;	 //三分线角度调整完成判断

	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;					 //DSP 45ms发送一次数据

	//这些点如果需要控件改变值，需要定义在成员变量，并在初始化函数里通过控件得到初始化值
	Posture ptOO, ptA0, ptC, ptM, ptN, ptZ, ptxy; //ptZ时中圈进口有障碍球时临时确定的辅助点
	ptOO.x = 1200;
	ptOO.y = 0; //终点前，用于回位定点

	if (1 == place_num)
	{
		ptA0.x = 3350; //（用向右偏的方法是2500）	//这个地方开始判断中圈进口是否有障碍球
		ptA0.y = -800;

		ptxy.x = -400;
		ptxy.y = 400;

		ptC.x = 3650; //三分线中点识别球定点处
		ptC.y = 4800;

		ptM.x = 875;
		ptM.y = 4300; //传球区中心位置

		ptN.x = 3000;
		ptN.y = 2100;
	}
	if (2 == place_num)
	{
		ptA0.x = 3200; //（用向右偏的方法是2500）	//这个地方开始判断中圈进口是否有障碍球
		ptA0.y = 500;

		ptxy.x = 200;
		ptxy.y = 200;

		ptC.x = 3650; //三分线球点
		ptC.y = -4800;

		ptM.x = 875;
		ptM.y = -4300; //传球区中心位置

		ptN.x = 3000;
		ptN.y = -2100; //投球定位点
	}
	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 00000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;
	case 120:
		if ((normal_time - temp_normal_time) > 200) //延时17.1s
		{
			temp_normal_time = normal_time;
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid2(ptA0)) //从起点O到点A0
		{
			ROS_INFO("我到 A 点了 我到 A 点了 我到 A 点了 ！！！X is : %ld，Y is : %ld, angle is : %lf\n", dsp_status.XDist, dsp_status.YDist, dsp_status.fAngle);
			Stop();
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 222222222222222222222222222222222222222222222222222222222222222222222222222222\n");
		Control.SendData_state = 6;
		if (fabs(dsp_status.fAngle) > 2) //调整角度归零
		{
			ROS_INFO("调整角度到零 调整角度到零 调整角度到零 ！！！X is:%ld，Y is:%ld, angle is : %lf\n", dsp_status.XDist, dsp_status.YDist, dsp_status.fAngle);
			ToAngleByMPU(-1);
		}
		else
		{
			ROS_INFO("角度已经归零 ！！！X is:%ld，Y is:%ld, angle is : %lf\n", dsp_status.XDist, dsp_status.YDist, dsp_status.fAngle);
			Control.SendData_state = 7;
			temp_normal_time = normal_time;
			intention_num = 3;
		}
		break;

		/********************************** 中圈识别球 ***********************************/
	case 3: //开启识别
		ROS_INFO_STREAM("case 333333333333333333333333333333333333333333333333333333333333333333\n");
		Control.SendData_state = 0;
		//TODO:发送接收数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
						TheObject.whatObject =(objectType)3;//找什么东西在这里改
						if(TheObject.whatObject==(objectType)1)
							ROS_INFO_STREAM("send command to find calibration\n");
						if(TheObject.whatObject==(objectType)2)
							ROS_INFO_STREAM("send command to find reddish_brown_basketball\n");
						if(TheObject.whatObject==(objectType)3)
							ROS_INFO_STREAM("send command to find basketball\n");
						if(TheObject.whatObject==(objectType)4)
							ROS_INFO_STREAM("send command to find blue_gray_basketball\n");
						if(TheObject.whatObject==(objectType)5)
							ROS_INFO_STREAM("send command to find calibration_plus\n");
						if(TheObject.whatObject==(objectType)6)
							ROS_INFO_STREAM("send command to find orange_red_volleyball\n");
						if(TheObject.whatObject==(objectType)7)
							ROS_INFO_STREAM("send command to find red_blue_volleyball\n");
						if(TheObject.whatObject==(objectType)8)
							ROS_INFO_STREAM("send command to find volleyball\n");

						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to send command\n");
							TheObject.whatObject = (objectType)0;//unkownObject变量重新初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}
		*/
		if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				robotstraightrightlow(); //开环横向低速右移
				ROS_INFO_STREAM(" 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 \n");
				if ((dsp_status.YDist) > 3500) //找了好久，一个都没找到，就回去了
				{
					ROS_INFO_STREAM("中圈走完了，也没找到球！！！那就回家吧！！！\n");
					Stop();
					temp_normal_time = normal_time;
					intention_num = 85;
				}
			}
			if (2 == place_num)
			{
				robotstraightleftlow(); //开环横向低速右移
				ROS_INFO_STREAM(" 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 \n");
				if ((dsp_status.YDist) < -3500) //找了好久，一个都没找到，就回去了
				{
					ROS_INFO_STREAM("中圈走完了，也没找到球！！！那就回家吧！！！\n");
					Stop();
					temp_normal_time = normal_time;
					intention_num = 85;
				}
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM(" find_objectflag find_objectflag find_objectflag find_objectflag find_objectflag find_objectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			ROS_INFO_STREAM(" cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
			if (ToAimBallByVision()) //是停下的时候通过摄像头返回的角度偏差对准球
			{
				ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{
					findballflag = 0;
					aimballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 3; //找球的时候，丢球，再去找
					ROS_INFO("丢了 丢了 丢了 ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("！！！ 中圈球找到了 中圈球找到了 中圈球找到了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1) && (armupflag == 0))
		{
			ROS_INFO_STREAM(" ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n");
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					ROS_INFO_STREAM(" can not find now...can not find now...can not find now...can not find now...\n");
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 3;
					ROS_INFO_STREAM(" 又丢了 又丢了 又丢了 ！！！\n");
				}
				else
				{
					armupflag = 1; //抬臂标识位置1
					ROS_INFO_STREAM("!!!!!!!!!!I have walked to the ball...I have walked to the ball...I have walked to the ball...I have walked to the ball!!!!!!!!!!\n");
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 4;
					ROS_INFO_STREAM("捡球 捡球 捡球 ！！！\n");
				}
			}
		}
		break;

	case 4:
		ROS_INFO_STREAM("case 444444444444444444444444444444444444444444444444444444444444444444444444444444444\n");
		temp_normal_time = normal_time; //为延时作准备
		if ((armupflag == 1) && (stopflag == 0))
		{
			Control.SendData_state = 1; //执行抬臂动作
			ROS_INFO_STREAM("case uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuppppppppppppppppppppppppppppppppppppp\n");
			temp_normal_time = normal_time;
			//stopflag=1;		//停止标识位置1
			intention_num = 5;
		}
		break;

	case 5: //判断是否捡到球
		ROS_INFO_STREAM("case 55555555555555555555555555555555555555555555555555555555555555555555555555555\n");
		Control.SendData_state = 0;
		if ((normal_time - temp_normal_time) > 40) //判断是否捡上来，2秒足够了，不管他放不放下
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一位立即置位0
			ROS_INFO_STREAM("第一次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 6;
		}
		break;

	case 6:
		ROS_INFO_STREAM("case 6666666666666666666666666666666666666666666666666666666666666666666666666666\n");
		Control.SendData_state = 0;
		if (('y' == dsp_status.RecData_state) || ('n' == dsp_status.RecData_state))
		{
			ROS_INFO_STREAM("第一次，判断架子上有没有球！！！！\n");
			if ('y' == dsp_status.RecData_state)
			{
				temp_normal_time = normal_time;
				intention_num = 80;
				ROS_INFO_STREAM("发现已经捡到球了，架子上有球！！！\n");
			}
			else //如果没捡到球，放下机械臂，重新捡球
			{
				findballflag = 0;
				aimballflag = 0;
				temp_normal_time = normal_time;
				intention_num = 81;
				ROS_INFO_STREAM("判断没有球啊！等待抬臂稳定后在判断一遍！！！\n");
			}
		}
		else
		{
			Control.SendData_state = 5;
			temp_normal_time = normal_time;
			intention_num = 6;
			ROS_INFO_STREAM("既没有收到Y也没有收到N，没接受到？还是没法出去？不管了，再问一遍\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 80:
		ROS_INFO_STREAM("case 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80\n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 820;
		}
		break;

	case 81:
		ROS_INFO_STREAM("case 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81\n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("第二次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 82;
		}
		break;

	case 82:
		ROS_INFO_STREAM("case 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 \n");
		Control.SendData_state = 0; //单片机位置0
		ROS_INFO_STREAM("第二次，判断架子上有没有球！！！！\n");
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("发现有球啊！！！第一次误判！！！\n");
			temp_normal_time = normal_time;
			intention_num = 820;
		}
		else //判断两次都没球，可以再去找了
		{
			findballflag = 0;
			aimballflag = 0;
			temp_normal_time = normal_time;
			intention_num = 3;
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 820:
		ROS_INFO_STREAM("case 820 820 820 820 820 820 820 820 820 820 820 820 820 820 820 820 820 820 \n");
		ROS_INFO_STREAM("正在对准传球定位点！！！！\n");
		Control.SendData_state = 6;
		if (ToAngleByMPU6050(ptN))
		{
			ROS_INFO_STREAM("已经对准了传球定位点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停到了投球定位点，再旋转调整！！！！\n");
				Control.SendData_state = 7;
				temp_normal_time = normal_time;
				intention_num = 821;
			}
		}
		break;

	case 821:
		ROS_INFO_STREAM("case 821 821 821 821 821 821 821 821 821 821 821 821 820 820 820 820 820 820 \n");
		Control.SendData_state = 0;
		temp_normal_time = normal_time;
		if (ToPostureByMilemeter_Pid1(ptN)) //到达N点
		{
			Stop();
			ROS_INFO_STREAM("到 NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN啦啦啦啦啦\n");
			intention_num = 83;
		}
		break;

	case 83:
		ROS_INFO_STREAM("case 83 83 83 83 83 83 83 83 83 83 83 83 83 83 83 83 83 83 83 83 83 83 \n");
		ROS_INFO_STREAM("正在对准传球标定柱！！！！\n");
		Control.SendData_state = 6;
		if (ToAngleByMPU6050(ptM))
		{
			ROS_INFO_STREAM("已经对准了传球标定柱！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，可以发射了！！！！\n");
				Control.SendData_state = 7;
				temp_normal_time = normal_time;
				intention_num = 84;
			}
		}
		break;

	case 84: //发射
		ROS_INFO_STREAM("case 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 84 \n");
		Control.SendData_state = 0;
		if ((normal_time - temp_normal_time) > 2)
		{
			Control.SendData_state = 2; //发送发射指令,下一步立即置位0
			ROS_INFO_STREAM("发射 发射 发射 ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 85; //用G点走折线到三分线时，下一步是79
		}
		break;

	case 85: //转向目的点，去找三分球
		ROS_INFO_STREAM("case 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 85 \n");
		Control.SendData_state = 0; //单片机位置0
		ROS_INFO_STREAM("调整角度，对准三分线目标点——调整角度，对准三分线目标点——调整角度，对准三分线目标点 ！！！！\n");
		Control.SendData_state = 6;
		if (ToAngleByMPU6050(ptC))
		{
			ROS_INFO_STREAM("对准了 对准了！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("我停下了！！！！\n");
				Control.SendData_state = 7;
				temp_normal_time = normal_time;
				intention_num = 86;
			}
		}
		break;

	case 86:
		ROS_INFO_STREAM("case 86 86 86 86 86 86 86 86 86 86 86 86 86 86 86 86 86 86 86 86 \n");
		ROS_INFO_STREAM("避障程序，走向三分点——避障程序，走向三分点——避障程序，走向三分点！！！！\n");
		Control.SendData_state = 0;
		if (ToPostureByAvoidance(ptC)) //避障
		{
			ROS_INFO_STREAM("避障完成了 避障完成了 避障完成了！！！！\n");
			Stop();
			temp_normal_time = normal_time;
			intention_num = 860;
		}
		ROS_INFO_STREAM("避障和延时之间！！！！\n");
		if ((normal_time - temp_normal_time) > 100)
		{
			ROS_INFO_STREAM("避障延时180之内！！！！\n");
			Stop();
			temp_normal_time = normal_time;
			intention_num = 861;
		}
		break;

	case 860:
		ROS_INFO_STREAM("case 810 810 810 810 810 810 810 810 810 810 810 810 810 810 810 810\n");
		if ((normal_time - temp_normal_time) > 33) //以开环速度向前2s
		{
			robotforward();
			ROS_INFO_STREAM("开环往前走2秒！！！！\n");
		}
		else
		{
			temp_normal_time = normal_time;
			intention_num = 861;
		}
		break;

	case 861: //避障后再次对准目标点
		ROS_INFO_STREAM("case 861 861 861 861 861 861 861 861 861 861 861 861 861 861 861 861 861 861 \n");

		if ((normal_time - temp_normal_time) > 2)
		{
			Control.SendData_state = 6;
			if (ToAngleByMPU6050(ptC))
			{
				ROS_INFO_STREAM("避障后再次对准目标点！！！！\n");
				if (stop())
				{
					Control.SendData_state = 7;
					temp_normal_time = normal_time;
					intention_num = 87;
				}
			}
		}
		break;

	case 87:
		ROS_INFO_STREAM("case 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 87 \n");
		Control.SendData_state = 0;
		ROS_INFO_STREAM("避障之后，再次对准，ToPostureByMilemeter函数，走向C点！！！！\n");
		if (ToPostureByMilemeter(ptC))
		{
			ROS_INFO_STREAM("到达三分点！！！！\n");
			Stop();
			temp_normal_time = normal_time;
			intention_num = 88;
		}
		break;

	case 88:
		ROS_INFO_STREAM("case 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88 88\n");
		if (1 == place_num)
		{
			ROS_INFO_STREAM("角度调整过程ing................\n");
			if (fabs(dsp_status.fAngle - 63.00) > 2)
			{
				ROS_INFO_STREAM("和66度相差大于2度................\n");
				if (dsp_status.fAngle > 65)
				{
					ROS_INFO_STREAM("角度为正，需要左旋................\n");
					ToFindObjectByturnleft();
				}
				if (dsp_status.fAngle < 61)
				{
					ROS_INFO_STREAM("角度为负，需要右旋................\n");
					ToFindObjectByturnright();
				}
			}
			else
			{
				ROS_INFO_STREAM("角度为63度，不要动了！！！................\n");
				temp_normal_time = normal_time;
				intention_num = 90;
			}
		}
		if (2 == place_num)
		{
			ROS_INFO_STREAM("角度调整过程ing................\n");
			if (fabs(dsp_status.fAngle - (-63.00)) > 2)
			{
				ROS_INFO_STREAM("和-66度相差大于2度................\n");
				if (dsp_status.fAngle > -61)
				{
					ROS_INFO_STREAM("角度为正，需要左旋................\n");
					ToFindObjectByturnleft();
				}
				if (dsp_status.fAngle < -65)
				{
					ROS_INFO_STREAM("角度为负，需要右旋................\n");
					ToFindObjectByturnright();
				}
			}
			else
			{
				ROS_INFO_STREAM("角度为-63度，不要动了！！！................\n");
				temp_normal_time = normal_time;
				intention_num = 90;
			}
		}
		break;

		/********************************** 三分线识别球 ***********************************/
	case 90: //拾球（当视觉识别出目标即视为到达点，开始使用视觉返回的距离与角度）
		ROS_INFO_STREAM("case 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 \n");
		if ((normal_time - temp_normal_time) > 5)
		{
			//TODO:发送接收数据
			/*
			if(!Vision.create_communication_RAM_2())
			{
				ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
				if(!Vision.open_file_mapping_2())
				{
					ROS_INFO_STREAM("fail to open_file_mapping_2\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to open_file_mapping_2\n");
					if(!Vision.create_communication_RAM_4())
					{
						ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
						if(!Vision.open_file_mapping_4())
						{
							ROS_INFO_STREAM("fail to open_file_mapping_4\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to open_file_mapping_4\n");
							TheObject.whatObject =(objectType)3;//找什么东西在这里改
							if(TheObject.whatObject==(objectType)1)
								ROS_INFO_STREAM("send command to find calibration\n");
							if(TheObject.whatObject==(objectType)2)
								ROS_INFO_STREAM("send command to find reddish_brown_basketball\n");
							if(TheObject.whatObject==(objectType)3)
								ROS_INFO_STREAM("send command to find basketball\n");
							if(TheObject.whatObject==(objectType)4)
								ROS_INFO_STREAM("send command to find blue_gray_basketball\n");
							if(TheObject.whatObject==(objectType)5)
								ROS_INFO_STREAM("send command to find calibration_plus\n");
							if(TheObject.whatObject==(objectType)6)
								ROS_INFO_STREAM("send command to find orange_red_volleyball\n");
							if(TheObject.whatObject==(objectType)7)
								ROS_INFO_STREAM("send command to find red_blue_volleyball\n");
							if(TheObject.whatObject==(objectType)8)
								ROS_INFO_STREAM("send command to find volleyball\n");

							if(!Vision.send_find_whatobject(TheObject.whatObject))
							{
								ROS_INFO_STREAM("Fail to send command\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to send command\n");
								TheObject.whatObject = (objectType)0;//unkownObject变量重新初始化，为下一次找球做准备
								if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
								{
									ROS_INFO_STREAM("Fail to receive final object result\n");
								}
								else
								{
									ROS_INFO_STREAM("Success to receive final object result\n");
									ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
									ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
								}
							}
						}
					}
				}
			}
			*/

			if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
			{
				findballflag = 0;
				aimballflag = 0;
				if (1 == place_num)
				{
					if (abs(dsp_status.XDist - 600) > 50)
					{
						ROS_INFO_STREAM("66度角，向右横向移动................\n");
						robotstraightrightlow();
					}
					else
					{
						ROS_INFO_STREAM("到边界了，没找到！！！................\n");
						Stop();
						temp_normal_time = normal_time;
						intention_num = 110;
					}
				}
				if (2 == place_num)
				{
					if (abs(dsp_status.XDist - 600) > 50)
					{
						ROS_INFO_STREAM("-66度角，向左横向移动................\n");
						robotstraightleftlow();
					}
					else
					{
						ROS_INFO_STREAM("到边界了，没找到！！！................\n");
						Stop();
						temp_normal_time = normal_time;
						intention_num = 110;
					}
				}
			}
			else
			{
				findballflag = 1;
				ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
			}
			if ((findballflag == 1) && (aimballflag == 0))
			{

				ROS_INFO_STREAM(" cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
				if (ToAimBallByVision())
				{
					ROS_INFO_STREAM(" eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee\n");
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //对准球的过程中突然丢了
						findballflag = 0;
						aimballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 90;
						ROS_INFO_STREAM("对准球的过程中突然丢了！！！................\n");
					}
					else
					{
						aimballflag = 1;
						ROS_INFO("!!!!!!!!三分球:objectAngle aimedaimedaimed is %lf;三分球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					}
				}
			}
			if ((aimballflag == 1))
			{
				ROS_INFO_STREAM(" ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n");
				if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
				{
					ROS_INFO_STREAM("前往球的位置 前往球的位置 前往球的位置 ！！！................\n");
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //前往球的跟前过程中，又没看到球，重新去找
						ROS_INFO_STREAM("又丢了 又丢了 又丢了 ！！！................\n");
						aimballflag = 0;
						findballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 90;
					}
					else
					{
						ROS_INFO_STREAM("可以捡球了 可以捡球了 可以捡球了 ！！！................\n");
						aimballflag = 0;
						findballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 91;
					}
				}
			}
		}
		break;
	case 91:
		ROS_INFO_STREAM("case 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 \n");
		if (normal_time - temp_normal_time > 2)
		{
			Stop();						//停车并立即捡球
			Control.SendData_state = 1; //球已持住，发送抬机械臂命令，下一步立即置位0
			ROS_INFO_STREAM("发送抬臂命令！！！................\n");
			temp_normal_time = normal_time;
			intention_num = 93; //抬机械臂命令已发送
		}
		break;
	case 93: //等待发送抬臂指令收到停车，等待机械臂抬起,然后落下
		ROS_INFO_STREAM("case 93 93 93 93 93 93 93 93 93 93 93 93 93 93 93 93 93 93 93 93 \n");
		Control.SendData_state = 0; //单片机位置0
		ROS_INFO_STREAM("等待抬杆放下！！！................\n");
		if ((normal_time - temp_normal_time) > 40) //1.5s时间抬臂、放臂
		{
			temp_normal_time = normal_time;
			intention_num = 94;
		}
		break;
	case 94:
		ROS_INFO_STREAM("case 94 94 94 94 94 94 94 94 94 94 94 94 94 94 94 94 94 94 94 94 \n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中就发第一次判断命令
		{
			//m_bSend=1;    //TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			ROS_INFO_STREAM("有没有球！！！发送判断指令！！！................\n");
			temp_normal_time = normal_time;
			intention_num = 95;
		}
		break;
	case 95:
		ROS_INFO_STREAM("case 95 95 95 95 95 95 95 95 95 95 95 95 95 95 95 95 95 95 95 95 95 \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			ROS_INFO_STREAM("抬臂过程中就检测到有球！！！................\n");
			temp_normal_time = normal_time;
			intention_num = 950;
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			ROS_INFO_STREAM("抬臂过程中没看到，准备持球机构稳定了再判断一次！！！................\n");
			temp_normal_time = normal_time;
			intention_num = 951;
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;
	case 950:
		ROS_INFO_STREAM("case 950 950 950 950 950 950 950 950 950 950 950 950 950 950 950 950 \n");
		ROS_INFO_STREAM("等待持球和球机构稳定！！！................\n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 97;
		}
		break;
	case 951:
		ROS_INFO_STREAM("case 951 951 951 951 951 951 951 951 951 951 951 951 951 951 951 951 \n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			//m_bSend=1;    //TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("有没有球！！！第二次发送判断指令！！！................\n");
			temp_normal_time = normal_time;
			intention_num = 96;
		}
		break;
	case 96:
		ROS_INFO_STREAM("case 96 96 96 96 96 96 96 96 96 96 96 96 96 96 96 96 96 96 96 96 96 \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("第二次，抬臂过程中就检测到有球！！！................\n");
			temp_normal_time = normal_time;
			intention_num = 97;
		}
		else //判断两次都没球，可以再去找了
		{
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！................\n");
			findballflag = 0;
			aimballflag = 0;
			temp_normal_time = normal_time;
			intention_num = 90;
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 97:
		ROS_INFO_STREAM("case 97 97 97 97 97 97 97 97 97 97 97 97 97 97 97 97 97 97 97 97 97 97 \n");
		ROS_INFO_STREAM("对准传球标定柱ing ！！！................\n");
		Control.SendData_state = 6;
		if (ToAngleByMPU6050(ptM))
		{
			if (stop())
			{
				ROS_INFO_STREAM("对准，停下！！！................\n");
				Control.SendData_state = 7;
				temp_normal_time = normal_time;
				intention_num = 99;
			}
		}
		break;

	case 99:
		ROS_INFO_STREAM("case 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 99 \n");
		Control.SendData_state = 0;
		if ((normal_time - temp_normal_time) > 2)
		{
			if (dsp_status.XDist > 3600)
			{
				Control.SendData_state = 2; //发送发射指令，下一步立即置位0 //gaile gaile
				ROS_INFO_STREAM("发射 发射 发射 ！！！................\n");
				temp_normal_time = normal_time;
				intention_num = 110;
			}
			else
			{
				Control.SendData_state = 4; //发送发射指令，下一步立即置位0 //gaile gaile
				ROS_INFO_STREAM("发射 发射 发射 ！！！................\n");
				temp_normal_time = normal_time;
				intention_num = 110;
			}
		}
		break;

		/************************************************  归位 归位 归位 ******************************************************/
	case 110: //旋转对准pt00点
		ROS_INFO_STREAM("case 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 3)
		{
			ROS_INFO_STREAM("走向 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ing\n");
			Control.SendData_state = 6;
			if (ToAngleByMPU6050(ptOO)) //先转向00
			{
				ROS_INFO_STREAM("到 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 啦啦啦啦啦\n");
				if (stop())
				{
					Control.SendData_state = 7;
					temp_normal_time = normal_time;
					intention_num = 111;
				}
			}
		}
		break;

	case 111: //旋转对准pt00点
		ROS_INFO_STREAM("case 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 3)
		{
			ROS_INFO_STREAM("走向 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ing\n");
			if (ToPostureByMilemeter_Pid3(ptOO)) //走向00
			{
				ROS_INFO_STREAM("到 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 啦啦啦啦啦\n");
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 220;
				}
			}
		}
		break;

	case 220:
		ROS_INFO_STREAM("case 220 220 220 220 220 220 220 220 220 220 220 220 220 220 220 220 220 \n");
		Control.SendData_state = 6;
		if ((normal_time - temp_normal_time) > 20)
		{
			if (1 == place_num)
			{
				if (ToAngleByMPU(-4))
				{
					ROS_INFO("角度对准了，为0了！！！jiaodu jiaodu .....................:%lf\n", dsp_status.fAngle); //就这里，总是为0
					Stop();
					Control.SendData_state = 7;
					temp_normal_time = normal_time;
					intention_num = 330;
				}
			}
			if (2 == place_num)
			{
				if (ToAngleByMPU(180))
				{
					ROS_INFO("角度对准了，为0了！！！jiaodu jiaodu .....................:%lf\n", dsp_status.fAngle); //就这里，总是为0
					Stop();
					Control.SendData_state = 7;
					temp_normal_time = normal_time;
					intention_num = 330;
				}
			}
		}
		break;

	case 330:
		ROS_INFO_STREAM("case 330 330 330 330 330 330 330 330 330 330 330 330 330 330 330 330 330 \n");
		Control.SendData_state = 0;
		if ((normal_time - temp_normal_time) > 5)
		{
			if (1 == place_num)
			{
				if (abs(dsp_status.YDist - 400) >= 30)
				{
					ROS_INFO("入库时Y轴没对准！！！需要左右调整！！！zuobiao zuobiao ...................: %ld, %ld\n", dsp_status.XDist, dsp_status.YDist);
					if ((dsp_status.YDist - 400) > 0)
					{
						ROS_INFO_STREAM("GO left ... GO left ... GO left ... GO left ... GO left ... GO left ... \n");
						robotstraightleftlow();
					}
					else
					{
						ROS_INFO_STREAM("GO right ... GO right ... GO right ... GO right ... GO right ... GO right ... \n");
						robotstraightrightlow();
					}
				}
				else
				{
					ROS_INFO_STREAM("入库时左右对准了！！！不要动了！！！stop stop .....................................................");
					Stop();
					temp_normal_time = normal_time;
					intention_num = 440;
				}
			}
			if (2 == place_num)
			{
				if (abs(dsp_status.YDist - 200) >= 30)
				{
					ROS_INFO("入库时Y轴没对准！！！需要左右调整！！！zuobiao zuobiao ...................: %ld, %ld\n", dsp_status.XDist, dsp_status.YDist);
					if ((dsp_status.YDist - 200) > 0)
					{
						ROS_INFO_STREAM("GO left ... GO left ... GO left ... GO left ... GO left ... GO left ... \n");
						robotstraightright();
					}
					else
					{
						ROS_INFO_STREAM("GO right ... GO right ... GO right ... GO right ... GO right ... GO right ... \n");
						robotstraightleft();
					}
				}
				else
				{
					ROS_INFO_STREAM("入库时左右对准了！！！不要动了！！！stop stop .....................................................");
					Stop();
					temp_normal_time = normal_time;
					intention_num = 440;
				}
			}
		}
		break;

	case 440:
		ROS_INFO_STREAM("case 440 440 440 440 440 440 440 440 440 440 440 440 440 440 440 440 \n");
		if ((normal_time - temp_normal_time) > 5)
		{
			if (1 == place_num)
			{
				if (abs(dsp_status.XDist - (-240)) < 10)
				{
					ROS_INFO_STREAM("stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop \n");
					Stop();
					intention_num = 444;
				}
				else
				{
					ROS_INFO_STREAM("我在后退！！\n");
					robotbacklow();
				}
			}
			if (2 == place_num)
			{
				if (ToPostureByMilemeter_Pid2(ptxy))
				{
					ROS_INFO_STREAM("stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop \n");
					Stop();
					intention_num = 444;
				}
			}
		}
		break;

	case 444:
		ROS_INFO_STREAM("case 444 444 444 444 444 444 444 444 444 444 444 444 444 444 444 444 444 \n");
		ROS_INFO_STREAM("停下了！！！ \n");
		Stop();
		break;

	default:
		break;
	}
}

void DecisionMaking::Normal_pass3() //**********传球-回合3**********
{
	//一些决策相关的标志
	static int intention_num = 0;		  //回合过程的意图顺序号
	static bool findballflag = 0;		  //找到目标标志位,每找完一次目标后都要记得清零
	static bool aimballflag = 0;		  //对准目标标志位，每对准一次目标后都要记得清零
	static bool Isthreescoreunnormal = 0; //捡三分球是否出现异常
	static int normal_time = 0;			  //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0;	  //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;						  //DSP 45ms发送一次数据

	Posture ptOO, ptAA, ptA0, ptBB, ptCC, ptM, ptD, ptZ, ptT, ptQQ;
	static Posture ptN = {0, 0, 0};

	ptA0.x = 1000;
	ptA0.y = 0;

	if (1 == place_num)
	{
		ptOO.x = 1200; //终点前，用于回位定点
		ptOO.y = 0;

		ptAA.x = 3900; //中点，扫描起点
		ptAA.y = 4750;

		ptM.x = 875;
		ptM.y = 4800; //投球目标点

		ptBB.x = 1575; //第二个分区开始扫描点，45度开始扫
		ptBB.y = 5600;

		ptCC.x = 800;
		ptCC.y = 7500; //边界点（临界主要用到X）

		ptD.x = 3300; //内圈扫描定点，位于三分线和内圈之间的一个辅助定位点，用于定点旋转找球
		ptD.y = 6700;

		ptZ.x = 2800;
		ptZ.y = 4000; //归位辅助点

		ptT.x = 2600;
		ptT.y = 4800;

		ptQQ.x = 800; //终点前，用于回位定点
		ptQQ.y = 8300;
	}
	if (2 == place_num)
	{
		ptOO.x = 1600; //终点前，用于回位定点
		ptOO.y = 100;

		ptAA.x = 3900; //中点，扫描起点
		ptAA.y = -4700;

		ptM.x = 875;
		ptM.y = -4800; //投球目标点

		ptBB.x = 1575; //第二个分区开始扫描点，45度开始扫
		ptBB.y = -5600;

		ptCC.x = 800;
		ptCC.y = -7500; //边界点（临界主要用到X）

		ptD.x = 3300; //内圈扫描定点
		ptD.y = -6700;

		ptZ.x = 2800;
		ptZ.y = -4000; //归位辅助点

		ptT.x = 3000;
		ptT.y = -4800;

		ptQQ.x = 800; //终点前，用于回位定点
		ptQQ.y = -8300;
	}

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 00000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		if ((normal_time - temp_normal_time) > 200) //延时大约9s
		{
			temp_normal_time = normal_time;
			intention_num = 1;
		}
		break;

	case 1: //从起点0到，转向、前往第一个三分捡球区点AA
		ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA0))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 100;
		}
		break;

	case 100:
		ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111111111111\n");
		if (ToAngleByMPU6050(ptAA))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 22222222222222222222222222222222222222222222222222222222222222222222222222222\n");
		if (ToPostureByMilemeter_Pid3(ptAA))
		{
			Stop();
			ROS_INFO_STREAM("走到AA点处！！！\n");
			temp_normal_time = normal_time;
			intention_num = 201;
		}
		break;

	case 201:
		ROS_INFO_STREAM("case 201 2222222222222222222222222222222222222222222222222222222222222222222222222222\n");
		if (ToPostureByMilemeter_Pid2(ptAA))
		{
			Stop();
			ROS_INFO_STREAM("走到AA点处！！！\n");
			temp_normal_time = normal_time;
			intention_num = 202;
		}
		break;

	case 202:
		ROS_INFO_STREAM("case 202 2222222222222222222222222222222222222222222222222222222222222222222222222222\n");
		if (ToPostureByMilemeter_Pid6(ptAA))
		{
			Stop();
			ROS_INFO_STREAM("走到AA点处！！！\n");
			temp_normal_time = normal_time;
			intention_num = 3;
		}
		break;

	case 3: //使车子成66度倾斜，为横向移动找三分球做准备
		ROS_INFO_STREAM("case 3333333333333333333333333333333333333333333333333333333333333333333333333\n");
		/*ROS_INFO_STREAM("正在旋转！！！\n");
		if(ToAngleByMPU(66))
		{
			ROS_INFO_STREAM("转到66度了！！！\n");
			if(stop())
			{
				ROS_INFO_STREAM("停稳！！！\n");
				temp_normal_time=normal_time;
				intention_num=4;
			}							
		}*/
		if (1 == place_num)
		{
			ROS_INFO_STREAM("角度调整过程ing................\n");
			if (fabs(dsp_status.fAngle - 66.00) > 2)
			{
				ROS_INFO_STREAM("和66度相差大于2度................\n");
				if (dsp_status.fAngle > 68)
				{
					ROS_INFO_STREAM("角度为正，需要左旋................\n");
					ToFindObjectByturnleft();
				}
				if (dsp_status.fAngle < 64)
				{
					ROS_INFO_STREAM("角度为负，需要右旋................\n");
					ToFindObjectByturnright();
				}
			}
			else
			{
				if (stop())
				{
					ROS_INFO_STREAM("角度为68度，不要动了！！！................\n");
					temp_normal_time = normal_time;
					intention_num = 4;
				}
			}
		}
		if (2 == place_num)
		{
			ROS_INFO_STREAM("角度调整过程ing................\n");
			if (fabs(dsp_status.fAngle - (-66.00)) > 2)
			{
				ROS_INFO_STREAM("和66度相差大于2度................\n");
				if (dsp_status.fAngle > -64)
				{
					ROS_INFO_STREAM("角度为正，需要左旋................\n");
					ToFindObjectByturnleft();
				}
				if (dsp_status.fAngle < -68)
				{
					ROS_INFO_STREAM("角度为负，需要右旋................\n");
					ToFindObjectByturnright();
				}
			}
			else
			{
				if (stop())
				{
					ROS_INFO_STREAM("角度为-66度，不要动了！！！................\n");
					temp_normal_time = normal_time;
					intention_num = 4;
				}
			}
		}
		break;

		/********************************* 第一次找球 ************************************/
	case 4:
		ROS_INFO_STREAM("case 44444444444444444444444444444444444444444444444444444444444444444444444444444\n");
		if ((normal_time - temp_normal_time) > 5)
		{
			//TODO:发送接收数据
			/*
			if(!Vision.create_communication_RAM_2())
			{
				ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
				if(!Vision.open_file_mapping_2())
				{
					ROS_INFO_STREAM("fail to open_file_mapping_2\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to open_file_mapping_2\n");
					if(!Vision.create_communication_RAM_4())
					{
						ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
						if(!Vision.open_file_mapping_4())
						{
							ROS_INFO_STREAM("fail to open_file_mapping_4\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to open_file_mapping_4\n");
							TheObject.whatObject =(objectType)3;//找什么东西在这里改
							if(!Vision.send_find_whatobject(TheObject.whatObject))
							{
								ROS_INFO_STREAM("Fail to send command\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to send command\n");
								TheObject.whatObject = (objectType)0;//unkownObject变量重新初始化，为下一次找球做准备
								if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
								{
									ROS_INFO_STREAM("Fail to receive final object result\n");
								}
								else
								{
									ROS_INFO_STREAM("Success to receive final object result\n");
									ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
									ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
								}
							}
						}
					}
				}
			}
			*/

			if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
			{
				findballflag = 0;
				aimballflag = 0;
				if (1 == place_num)
				{
					if (abs((dsp_status.XDist) - (ptCC.x)) < 50)
					{
						Stop();
						ROS_INFO_STREAM("已经到边界了，但啥也没找着，算了，去找内圈球吧！！！\n");

						ptN.x = dsp_status.XDist; //把当前坐标赋给点N
						ptN.y = dsp_status.YDist;
						ROS_INFO("点N的坐标为：%ld , %ld\n", ptN.x, ptN.y);

						temp_normal_time = normal_time;
						intention_num = 103;
					}
					else
					{
						ROS_INFO_STREAM("以66度方向行进...\n");
						robotstraightright(); //继续右走
					}
				}
				if (2 == place_num)
				{
					if (abs((dsp_status.XDist) - (ptCC.x)) < 50)
					{
						Stop();
						ROS_INFO_STREAM("已经到边界了，但啥也没找着，算了，去找内圈球吧！！！\n");

						ptN.x = dsp_status.XDist; //把当前坐标赋给点N
						ptN.y = dsp_status.YDist;
						ROS_INFO("点N的坐标为：%ld , %ld\n", ptN.x, ptN.y);

						temp_normal_time = normal_time;
						intention_num = 103;
					}
					else
					{
						ROS_INFO_STREAM("以-66度方向行进...\n");
						robotstraightleft(); //继续左走
					}
				}
			}
			else
			{
				findballflag = 1;
				ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
			}
			if ((findballflag == 1) && (aimballflag == 0))
			{
				ROS_INFO_STREAM(" cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
				if (ToAimBallByVision())
				{
					ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //对准球的过程中突然丢了
						findballflag = 0;
						aimballflag = 0;
						ROS_INFO("丢了 丢了 丢了 ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 4;
						}
					}
					else
					{
						aimballflag = 1;
						ROS_INFO("!!!!!!!!三分线球:objectAngle aimedaimedaimed is %lf;三分线球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					}
				}
			}
			if ((findballflag == 1) && (aimballflag == 1))
			{
				ROS_INFO_STREAM(" ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n");
				if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
				{
					ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //前往球的跟前过程中，又没看到球，重新去找
						aimballflag = 0;
						findballflag = 0;
						ROS_INFO_STREAM(" 又丢了 又丢了 又丢了 ！！！\n");
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 4;
						}
					}
					else
					{
						aimballflag = 0;
						findballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 5;
						ROS_INFO_STREAM("捡球 捡球 捡球 ！！！\n");
					}
				}
			}
		}
		break;

	case 5:
		ROS_INFO_STREAM("case 55555555555555555555555555555555555555555555555555555555555555555555555555555\n");
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令
		ROS_INFO_STREAM("case uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuppppppppppppppppppppppppppppppppppppp\n");

		ptN.x = dsp_status.XDist; //把当前坐标赋给点N
		ptN.y = dsp_status.YDist;
		ROS_INFO("点N的坐标为：%ld , %ld\n", ptN.x, ptN.y);

		temp_normal_time = normal_time;
		intention_num = 6; //抬机械臂命令已发送
		break;

	case 6: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 66666666666666666666666666666666666666666666666666666666666666666666666666666\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 40)
		{
			ROS_INFO_STREAM("机械臂抬起，等待延时...\n");
			temp_normal_time = normal_time;
			intention_num = 7;
		}
		break;

	case 7:
		ROS_INFO_STREAM("case 77777777777777777777777777777777777777777777777777777777777777777777777777777\n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中发第一次判断命令
		{
			//m_bSend=1;    //TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			ROS_INFO_STREAM("第一次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 8;
		}
		break;

	case 8:
		ROS_INFO_STREAM("case 88888888888888888888888888888888888888888888888888888888888888888888888888888\n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			temp_normal_time = normal_time;
			intention_num = 80;
			ROS_INFO_STREAM("发现已经捡到球了，架子上有球！！！\n");
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			temp_normal_time = normal_time;
			intention_num = 81;
			ROS_INFO_STREAM("判断没有球啊！等待抬臂稳定后在判断一遍！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 80:
		ROS_INFO_STREAM("case 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80\n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 820;
		}
		break;

	case 81:
		ROS_INFO_STREAM("case 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81\n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			//m_bSend=1;    //TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("第二次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 82;
		}
		break;

	case 82:
		ROS_INFO_STREAM("case 820 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82\n");
		Control.SendData_state = 0; //单片机位置0
		ROS_INFO_STREAM("第二次，判断架子上有没有球！！！！\n");
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("发现有球啊！！！第一次误判！！！\n");
			temp_normal_time = normal_time;
			intention_num = 820;
		}
		else //判断两次都没球，可以再去找了
		{
			findballflag = 0;
			aimballflag = 0;
			temp_normal_time = normal_time;
			intention_num = 4;
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 820:
		ROS_INFO_STREAM("case 820 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82\n");
		if ((normal_time - temp_normal_time) < 15)
		{
			robotbacklow();
		}
		else
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 83;
		}
		break;

	case 83:
		ROS_INFO_STREAM("case 83 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82\n");
		if (ToPostureByMilemeter_Pid6(ptT))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 101;
		}
		break;

	case 101: //瞄准，准备发射篮球
		ROS_INFO_STREAM("case 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101\n");
		ROS_INFO_STREAM("正在对准传球目标点！！！！\n");
		Control.SendData_state = 6; //关闭里程计
		if (ToAngleByMPU6050(ptM))
		{
			ROS_INFO_STREAM("已经对准了传球目标点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，准备弹射！！！！\n");
				Control.SendData_state = 7; //旋转停稳之后，再打开里程计
				temp_normal_time = normal_time;
				intention_num = 1010;
			}
		}
		break;

	case 1010:
		ROS_INFO_STREAM("case 1010 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (abs(dsp_status.XDist - ptT.x) > 50)
			{
				if ((dsp_status.XDist - ptT.x) > 50)
					robotforwardtoolow();
				else
					robotbacktoolow();
			}
			else
			{
				Stop();
				temp_normal_time = normal_time;
				intention_num = 102;
			}
		}
		break;

	case 102:
		ROS_INFO_STREAM("case 102 102 102 102 102 102 102 102 102 102 102 102 102 102 102 102 \n");
		Control.SendData_state = 0;
		if ((normal_time - temp_normal_time) > 2)
		{
			if (dsp_status.XDist > 3600)
			{
				Control.SendData_state = 2; //发送发射指令，下一步立即置位0 //gaile gaile
				ROS_INFO_STREAM("发射 发射 发射 ！！！................\n");
				temp_normal_time = normal_time;
				intention_num = 103;
			}
			else
			{
				Control.SendData_state = 4; //发送发射指令，下一步立即置位0 //gaile gaile
				ROS_INFO_STREAM("发射 发射 发射 ！！！................\n");
				temp_normal_time = normal_time;
				intention_num = 1020;
			}
		}
		break;

	case 1020:
		ROS_INFO_STREAM("case 1020 102 102 102 102 102 102 102 102 102 102 102 102 102 102 102 \n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid2(ptN))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 103;
		}
		break;

	case 103: //再转回来，对准内圈定位点
		ROS_INFO_STREAM("case 103 103 103 103 103 103 103 103 103 103 103 103 103 103 103 103 \n");
		ROS_INFO_STREAM("正在对准内圈定位点！！！！\n");
		Control.SendData_state = 0;
		Control.SendData_state = 6; //关闭里程计
		if (ToAngleByMPU6050(ptD))
		{
			ROS_INFO_STREAM("已经对准了内圈定位点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，下一步要往前走！！！！\n");
				Control.SendData_state = 7; //打开里程计
				temp_normal_time = normal_time;
				intention_num = 104;
			}
		}
		break;

	case 104:
		ROS_INFO_STREAM("case 104 104 104 104 104 104 104 104 104 104 104 104 104 104 104 104 104 104 \n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid2(ptD)) //到达D点
		{
			Stop();
			ROS_INFO_STREAM("到 DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD啦啦啦啦啦\n");
			intention_num = 10401;
		}
		break;

		//ZAIZHEZAIZHE
	case 10401:
		ROS_INFO_STREAM("case 10401 104 104 104 104 104 104 104 104 104 104 104 104 104 104 104 104 104 \n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid6(ptD)) //到达D点
		{
			Stop();
			ROS_INFO_STREAM("到 DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD啦啦啦啦啦\n");
			intention_num = 1040;
		}
		break;

	case 1040:
		ROS_INFO_STREAM("case 1040 1040 1040 104 104 104 104 104 104 104 104 104 104 104 104 104 104 104 \n");
		if ((normal_time - temp_normal_time) > 5)
		{
			if (1 == place_num)
			{
				if (ToAngleByMPU(30))
				{
					Stop();
					ROS_INFO_STREAM("旋转到30 啦！！！\n");
					temp_normal_time = normal_time;
					intention_num = 105;
				}
			}
			if (2 == place_num)
			{
				if (ToAngleByMPU(-30))
				{
					Stop();
					ROS_INFO_STREAM("旋转到-30 啦！！！\n");
					temp_normal_time = normal_time;
					intention_num = 105;
				}
			}
		}
		break;

		/***********************************  旋转识球  ***********************************/
	case 105:
		ROS_INFO_STREAM("case 105 105 105 105 105 105 105 105 105 105 105 105 105 105 105 105 105 105 \n");
		temp_normal_time = normal_time; //为延时作准备
		//TODO:发送接收数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");

						//视觉修改版找球测试start
						TheObject.whatObject =(objectType)3;//找什么东西在这里改
						if(TheObject.whatObject==(objectType)1)
							ROS_INFO_STREAM("send command to find calibration\n");
						if(TheObject.whatObject==(objectType)2)
							ROS_INFO_STREAM("send command to find reddish_brown_basketball\n");
						if(TheObject.whatObject==(objectType)3)
							ROS_INFO_STREAM("send command to find basketball\n");
						if(TheObject.whatObject==(objectType)4)
							ROS_INFO_STREAM("send command to find blue_gray_basketball\n");
						if(TheObject.whatObject==(objectType)5)
							ROS_INFO_STREAM("send command to find calibration_plus\n");
						if(TheObject.whatObject==(objectType)6)
							ROS_INFO_STREAM("send command to find orange_red_volleyball\n");
						if(TheObject.whatObject==(objectType)7)
							ROS_INFO_STREAM("send command to find red_blue_volleyball\n");
						if(TheObject.whatObject==(objectType)8)
							ROS_INFO_STREAM("send command to find volleyball\n");

						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM(" Success to send command Success to send command Success to send command Success to send command\n");
							TheObject.whatObject = (objectType)0;//unkownObject变量重新初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}
		*/
		if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				if ((normal_time - temp_normal_time) < 200)
				{
					ToFindObjectByturnright(); //右旋找球测试（需要测试旋转找球时请将该函数释放）
					ROS_INFO_STREAM(" 我在右转 我在右转 我在右转 我在右转 我在右转 我在右转 我在右转 我在右转 我在右转 我在右转 \n");
					//robotstraightrightlow();	//右移找球测试
					//ROS_INFO_STREAM(" 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 \n");
				}
				else
				{
					temp_normal_time = normal_time; //为延时作准备
					intention_num = 11502;
				}
			}
			if (2 == place_num)
			{
				if ((normal_time - temp_normal_time) < 200)
				{
					ToFindObjectByturnleft(); //右旋找球测试（需要测试旋转找球时请将该函数释放）
					ROS_INFO_STREAM(" 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转 \n");
					//robotstraightrightlow();	//右移找球测试
					//ROS_INFO_STREAM(" 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 \n");
				}
				else
				{
					temp_normal_time = normal_time; //为延时作准备
					intention_num = 11502;
				}
			}
		}
		else
		{
			//Control.SendData_state=7;	//打开里程计
			findballflag = 1;
			ROS_INFO_STREAM(" findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			//Control.SendData_state=6;	//关闭里程计
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					ROS_INFO("丢了 丢了 丢了 : %lf \n", objectAngleDistance_Y.first);
					if ((normal_time - temp_normal_time) > 2)
					{
						temp_normal_time = normal_time; //为延时作准备
						intention_num = 105;
					}
				}
				else
				{
					//Control.SendData_state=7;	//打开里程计
					aimballflag = 1;
					ROS_INFO("!!!!!!!!中圈球:objectAngle aimedaimedaimed is %lf;中圈球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1))
		{
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("又丢了 又丢了 又丢了 \n");
					if ((normal_time - temp_normal_time) > 2)
					{
						temp_normal_time = normal_time; //为延时作准备
						intention_num = 105;
					}
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 106;
					ROS_INFO_STREAM("捡球 捡球 捡球 \n");
				}
			}
		}
		break;

	case 106:
		ROS_INFO_STREAM("case 106 106 106 106 106 106 106 106 106 106 106 106 106 106 106 106 106 106 106 \n");
		//立即捡球，不要延时
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		ROS_INFO_STREAM("发送抬臂指令！！！");
		temp_normal_time = normal_time;
		intention_num = 107; //抬机械臂命令已发送
		break;

	case 107: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 107 107 107 107 107 107 107 107 107 107 107 107 107 107 107 107 107 107 107 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 40)
		{
			temp_normal_time = normal_time;
			intention_num = 108;
		}
		break;

	case 108:
		ROS_INFO_STREAM("case 108 108 108 108 108 108 108 108 108 108 108 108 108 108 108 108 108 108 108 \n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中就发第一次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			ROS_INFO_STREAM("第一次，发送捡球判断！！！");
			temp_normal_time = normal_time;
			intention_num = 109;
		}
		break;

	case 109:
		ROS_INFO_STREAM("case 109 109 109 109 109 109 109 109 109 109 109 109 109 109 109 109 109 109 109 \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			ROS_INFO_STREAM("检测到有球！！！");
			temp_normal_time = normal_time;
			intention_num = 110;
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			ROS_INFO_STREAM("没检测到有球，在判断一次！！！");
			temp_normal_time = normal_time;
			intention_num = 111;
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 110:
		ROS_INFO_STREAM("case 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 110 \n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 114;
		}
		break;

	case 111:
		ROS_INFO_STREAM("case 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111 \n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("第二次，发送捡球判断！！！");
			temp_normal_time = normal_time;
			intention_num = 113;
		}
		break;

	case 113:
		ROS_INFO_STREAM("case 113 113 113 113 113 113 113 113 113 113 113 113 113 113 113 113 113 113 113 \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("第一次为误判，明明有球！！！");
			temp_normal_time = normal_time;
			intention_num = 114;
		}
		else //判断两次都没球，可以再去找了
		{
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！");
			findballflag = 0;
			aimballflag = 0;
			temp_normal_time = normal_time;
			intention_num = 105;
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 114:
		ROS_INFO_STREAM(" 114 114 114 114 114 114 114 114 114 114 114 114 114 114 114 114 114 114 114 114 \n");
		Control.SendData_state = 6; //旋转时关闭里程计
		ROS_INFO_STREAM("正在对准投球定位点N ！！！！\n");
		ROS_INFO("点N的坐标为：%ld , %ld\n", ptN.x, ptN.y);
		if (ToAngleByMPU6050(ptN))
		{
			ROS_INFO_STREAM("已经对准了投球定位点N ！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，下一步要往前走！！！！\n");
				Control.SendData_state = 7; //打开里程计
				temp_normal_time = normal_time;
				intention_num = 115;
			}
		}
		break;

	case 115:
		ROS_INFO_STREAM("case 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 \n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid6(ptN)) //到达N点
		{
			Stop();
			ROS_INFO_STREAM("到 NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN啦啦啦啦啦\n");
			temp_normal_time = normal_time;
			intention_num = 11501;
		}
		break;

	case 11501:
		ROS_INFO_STREAM("case 11501 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 \n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid2(ptN)) //到达N点
		{
			Stop();
			ROS_INFO_STREAM("到 NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN啦啦啦啦啦\n");
			temp_normal_time = normal_time;
			intention_num = 1150;
		}
		break;

	case 1150:
		ROS_INFO_STREAM("case 1150 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 \n");
		if ((normal_time - temp_normal_time) < 11) //以开环速度移动1.5s
		{
			ROS_INFO_STREAM("开环走2秒...\n");
			robotforward();
		}
		else
		{
			Stop();
			ROS_INFO_STREAM("走到2秒处！！！\n");
			temp_normal_time = normal_time;
			intention_num = 1151;
		}
		break;

	case 11502:
		ROS_INFO_STREAM("case 11502 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 \n");
		if (ToPostureByMilemeter_Pid2(ptQQ))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 11503;
		}
		break;

	case 11503:
		ROS_INFO_STREAM("case 11503 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 115 \n");
		if (1 == place_num)
		{
			if (ToAngleByMPU(175))
			{
				Stop();
				temp_normal_time = normal_time;
				intention_num = 105;
			}
		}
		if (2 == place_num)
		{
			if (ToAngleByMPU(-175))
			{
				Stop();
				temp_normal_time = normal_time;
				intention_num = 105;
			}
		}
		break;

	case 1151:
		ROS_INFO_STREAM("case 1151 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82\n");
		if (ToPostureByMilemeter_Pid2(ptT))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 1152;
		}
		break;

	case 1152:
		ROS_INFO_STREAM("case 1152 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82\n");
		if (ToPostureByMilemeter_Pid6(ptT))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 1153;
		}
		break;

	case 1153: //瞄准，准备发射篮球
		ROS_INFO_STREAM("case 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101\n");
		ROS_INFO_STREAM("正在对准传球目标点！！！！\n");
		Control.SendData_state = 6; //关闭里程计
		if (ToAngleByMPU6050(ptM))
		{
			ROS_INFO_STREAM("已经对准了传球目标点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，准备弹射！！！！\n");
				Control.SendData_state = 7; //旋转停稳之后，再打开里程计
				temp_normal_time = normal_time;
				intention_num = 1154;
			}
		}
		break;

	case 1154:
		ROS_INFO_STREAM("case 1010 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (abs(dsp_status.XDist - ptT.x) > 50)
			{
				if ((dsp_status.XDist - ptT.x) > 50)
					robotforwardtoolow();
				else
					robotbacktoolow();
			}
			else
			{
				Stop();
				temp_normal_time = normal_time;
				intention_num = 116;
			}
		}
		break;

	case 116: //瞄准，准备发射篮球
		ROS_INFO_STREAM("case 116 116 116 116 116 116 116 116 116 116 116 116 116 116 116 116 116 116 116 \n");
		Control.SendData_state = 6; //旋转时关闭里程计
		ROS_INFO_STREAM("正在对准传球目标点！！！！\n");
		if (ToAngleByMPU6050(ptM))
		{
			ROS_INFO_STREAM("已经对准了传球目标点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，准备弹射！！！！\n");
				Control.SendData_state = 7; //打开里程计
				temp_normal_time = normal_time;
				intention_num = 117;
			}
		}
		break;

	case 117:
		ROS_INFO_STREAM("case 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 \n");
		Control.SendData_state = 0;
		if ((normal_time - temp_normal_time) > 2)
		{
			if (dsp_status.XDist > 3600)
			{
				Control.SendData_state = 2; //发送发射指令，下一步立即置位0 //gaile gaile
				ROS_INFO_STREAM("发射 发射 发射 ！！！................\n");
				temp_normal_time = normal_time;
				intention_num = 1170;
			}
			else
			{
				Control.SendData_state = 4; //发送发射指令，下一步立即置位0 //gaile gaile
				ROS_INFO_STREAM("发射 发射 发射 ！！！................\n");
				temp_normal_time = normal_time;
				intention_num = 1170;
			}
		}
		break;

	case 1170:
		ROS_INFO_STREAM("case 1170 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 3)
		{
			ROS_INFO_STREAM("在去Z点途中 ！！！！\n");
			if (ToPostureByMilemeter_Pid2(ptZ))
			{
				ROS_INFO_STREAM("已经到达Z点diyici！！！！\n");
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 1171;
				}
			}
		}
		break;

	case 1171:
		ROS_INFO_STREAM("case 1171 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 117 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 3)
		{
			ROS_INFO_STREAM("在去Z点途中 ！！！！\n");
			if (ToPostureByMilemeter_Pid6(ptZ))
			{
				ROS_INFO_STREAM("已经到达Z点！！！！\n");
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 118;
				}
			}
		}
		break;

	case 118: //**********车子归位开始**********
		ROS_INFO_STREAM("case 118 118 118 118 118 118 118 118 118 118 118 118 118 118 118 118 118 118 118 \n");
		if ((normal_time - temp_normal_time) > 3)
		{
			ROS_INFO_STREAM("在去00点途中 ！！！！\n");
			if (ToPostureByMilemeter_Pid2(ptOO))
			{
				ROS_INFO_STREAM("已经到达00点！！！！\n");
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 220;
				}
			}
		}
		break;

	case 220:
		ROS_INFO_STREAM("case 220 220 220 220 220 220 220 220 220 220 220 220 220 220 220 220 220 220 220 \n");
		if (fabs(dsp_status.fAngle) > 2.5)
		{
			Control.SendData_state = 6; //旋转时关闭里程计
			ROS_INFO("当前角度为：%lf \n", dsp_status.fAngle);
			ToAngleByMPU(0);
		}
		else
		{
			ROS_INFO("调整之后角度为：%lf \n", dsp_status.fAngle);
			Stop();
			Control.SendData_state = 7; //打开里程计
			intention_num = 330;
		}
		break;

	case 330:
		ROS_INFO_STREAM("case 330 330 330 330 330 330 330 330 330 330\n");
		Control.SendData_state = 0;
		if (1 == place_num)
		{
			if (abs(dsp_status.YDist - (-70)) > 30)
			{
				ROS_INFO("当前坐标为：%ld, %ld \n", dsp_status.XDist, dsp_status.YDist);
				if ((dsp_status.YDist - (-70)) > 0)
				{
					ROS_INFO_STREAM("左直走 ！！！！\n");
					robotstraightleft();
				}
				else
				{
					ROS_INFO_STREAM("右直走 ！！！！\n");
					robotstraightright();
				}
			}
			else
			{
				ROS_INFO_STREAM("左右对准了 ！！！！\n");
				Stop();
				intention_num = 440;
			}
		}
		if (2 == place_num)
		{
			if (abs(dsp_status.YDist - 100) > 30)
			{
				ROS_INFO("当前坐标为：%ld, %ld \n", dsp_status.XDist, dsp_status.YDist);
				if ((dsp_status.YDist - 100) > 0)
				{
					ROS_INFO_STREAM("左直走 ！！！！\n");
					robotstraightleft();
				}
				else
				{
					ROS_INFO_STREAM("右直走 ！！！！\n");
					robotstraightright();
				}
			}
			else
			{
				ROS_INFO_STREAM("左右对准了 ！！！！\n");
				Stop();
				intention_num = 440;
			}
		}
		break;

	case 440:
		ROS_INFO_STREAM("case 440 440 440 440 440 440 440 440 440 440 440 440 440 440 440 440\n");
		Control.SendData_state = 6;
		if (1 == place_num)
		{
			if (ToAngleByMPU(-5)) //实测数据_201808081709
			{
				ROS_INFO_STREAM("停 ！！！ \n");
				Control.SendData_state = 7;
				Stop();
				intention_num = 660;
			}
		}
		if (2 == place_num)
		{
			if (ToAngleByMPU(-3)) //这个值有待实测
			{
				ROS_INFO_STREAM("停 ！！！ \n");
				Control.SendData_state = 7;
				Stop();
				intention_num = 660;
			}
		}
		break;

		/*case 550:
ROS_INFO_STREAM("case 550 550 550 550 550 550 550 550 550 550 550 550 550 550 550 550\n");
		Control.SendData_state=0;
		if((normal_time-temp_normal_time)>5)
		{
			Control.SendData_state=8;
			temp_normal_time=normal_time;
			intention_num=660;
		}
		break;*/

	case 660:
		ROS_INFO_STREAM("case 660 660 660 660 660 660 660 660 660 660 660 660 660 660 660 660 660 \n");
		//Control.SendData_state=0;
		if ((normal_time - temp_normal_time) > 5)
		{
			if (1 == place_num)
			{
				if (abs(dsp_status.XDist - (-300)) > 10)
				{
					ROS_INFO_STREAM("后退 ！！！ \n");
					robotbacklow();
				}
				else
				{
					ROS_INFO_STREAM("停 ！！！ \n");
					Stop();
					intention_num = 666;
				}
			}
			if (2 == place_num)
			{
				if (abs(dsp_status.XDist - 350) > 10) //应该是实测值_201808081710
				{
					ROS_INFO_STREAM("后退 ！！！ \n");
					robotbacklow();
				}
				else
				{
					ROS_INFO_STREAM("停 ！！！ \n");
					Stop();
					intention_num = 666;
				}
			}
		}
		break;

	case 666:
		ROS_INFO_STREAM("case 666 666 666 666 666 666 666 666 666 666 666 666 666 666 666 666 666 666 \n");
		Stop(); //停车
		break;

	default:
		normal_time = 0;
		break;
		//**********车子归位结束**********
	}
}

void DecisionMaking::Normal_bat1() //**********投篮-回合1**********
{
	//一些决策相关的标志
	static int intention_num = 0;	 //回合过程的意图顺序号
	static bool findballflag = 0;	 //找到目标标志位,每找完一次目标后都要记得清零
	static bool aimballflag = 0;	 //对准目标标志位，每对准一次目标后都要记得清零
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;

	static bool Iscircleavoidancecorner = 0; //中圈避障是否向边缘靠近了

	Posture ptA0, ptA, ptC, ptD, ptD2, ptE, ptZ, ptD3, ptD4; //ptZ是中圈有障碍球时临时确定的辅助点
	//Posture pt00 = {1000,0};
	ptA0.x = 1400;
	ptA0.y = 0;

	if (1 == place_num)
	{
		ptA.x = 3000; //第二个球扫描起始定位点
		ptA.y = 3200;

		ptC.x = 4250; //投篮标定柱位置
		ptC.y = 10925;

		ptD.x = 3200; //投篮正中定位点（激光距离篮筐2400）（钢板400计，激光超出30计）
		ptD.y = 8100;

		ptE.x = 1890; //投篮45偏角位置（预留出车身宽度和激光距离，分别400和30计）（2630处）
		ptE.y = 9065;

		ptD2.x = 3200;
		ptD2.y = 8100;
	}
	if (2 == place_num)
	{
		ptA.x = 3100; //第二个球扫描起始定位点
		ptA.y = -3200;

		ptC.x = 4250; //投篮标定柱位置
		ptC.y = -10925;

		ptD.x = 3280; //投篮正中定位点（激光距离篮筐2400）（钢板400计，激光超出30计）
		ptD.y = -8100;

		ptD2.x = 3240;
		ptD2.y = -8100;

		ptE.x = 1890; //投篮45偏角位置（预留出车身宽度和激光距离，分别400和30计）（2630处）
		ptE.y = -9065;
	}
	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 00000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;

	case 120:
		if ((normal_time - temp_normal_time) > 200) //延时17.1s
		{
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA0))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 101;
		}
		break;

	case 101:
		ROS_INFO_STREAM("case 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 \n");
		if (ToAngleByMPU6050(ptD))
		{
			if (stop())
			{
				ROS_INFO_STREAM("对准D点！！\n");
				temp_normal_time = normal_time;
				intention_num = 102;
			}
		}
		break;

	case 102:
		ROS_INFO_STREAM("case 102 102 102 102 102 102 102 102 102 102 102 102 102 102  \n");
		if ((normal_time - temp_normal_time) > 3)
		{
			if (ToPostureByMilemeter_Pid3(ptD))
			{
				ROS_INFO_STREAM("前往D点，第一次停下 ！！\n");
				Stop();
				temp_normal_time = normal_time;
				intention_num = 103;
			}
		}
		break;

	case 103:
		ROS_INFO_STREAM("case 103 103 103 103 103 103 103 103 103 103 103 103 103 103 103 \n");
		if ((normal_time - temp_normal_time) > 3)
		{
			if ((abs(dsp_status.YDist - ptD.y) > 50) || (abs(dsp_status.XDist - ptD.x) > 50))
			{
				if (ToPostureByMilemeter_Pid6(ptD))
				{
					ROS_INFO_STREAM("前往D点，第二次停下 ！！\n");
					Stop();
					temp_normal_time = normal_time;
					intention_num = 104;
				}
			}
		}
		break;

	case 104:
		ROS_INFO_STREAM("case 104 103 103 103 103 103 103 103 103 103 103 103 103 103 103 \n");
		if ((normal_time - temp_normal_time) > 3)
		{
			if (1 == place_num)
			{
				if (ToAngleByMPU(100))
				{
					Stop();
					temp_normal_time = normal_time;
					intention_num = 2;
				}
			}
			if (2 == place_num)
			{
				if (ToAngleByMPU(-100))
				{
					Stop();
					temp_normal_time = normal_time;
					intention_num = 2;
				}
			}
		}
		break;

		/********************************* 第一次识别标定柱 *********************************/
	case 2:
		ROS_INFO_STREAM("case 22222222222222222222222222222222222222222222222222222222222222222222222222222\n ");
		//TODO:发送接受数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
						TheObject.whatObject = (objectType)1;	//找什么东西在这里改
						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to send command\n");
							TheObject.whatObject = (objectType)0;//变量初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");	
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}
		*/
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				if ((normal_time - temp_normal_time) < 100)
					ToFindObjectByturnleft();
				else if (stop())
					ToFindObjectByturnright();
			}
			if (2 == place_num)
			{
				if ((normal_time - temp_normal_time) < 100)
					ToFindObjectByturnright();
				else if (stop())
					ToFindObjectByturnleft();
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((00.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 2;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!标注杆:objectAngle aimedaimedaimed is %lf;标注杆:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 40)) && (aimballflag == 1))
				{
					Stop();
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("!!!!!!!!标定柱:The distance has been to the min error;The distance has been to the min error!!!!!!!!\n");
					temp_normal_time = normal_time;
					intention_num = 3;
				}
			}
		}
		break;

	case 3: //发射
		ROS_INFO_STREAM("case 33333333333333333333333333333333333333333333333333333333333333333333333333333\n");
		if ((normal_time - temp_normal_time) > 22)
		{
			Stop();
			Control.SendData_state = 3; //发送发射指令，下一步要立即置位0
			ROS_INFO_STREAM("发送弹球指令 ！！！\n");
			temp_normal_time = normal_time;
			intention_num = 5;
		}
		break;

	case 5:
		ROS_INFO_STREAM("case 5555555555555555555555555555555555555555555555555555555555555555555555555555\n");
		Control.SendData_state = 0; //单片机位置0
		if (normal_time - temp_normal_time > 10)
		{
			if (ToAngleByMPU6050(ptA))
			{
				Stop();
				ROS_INFO_STREAM("已经对准了A点 ！！！\n");
				temp_normal_time = normal_time;
				intention_num = 6;
			}
		}
		break;

	case 6:
		ROS_INFO_STREAM("case 6666666666666666666666666666666666666666666666666666666666666666666666666666\n");
		if (ToPostureByMilemeter_Pid3(ptA))
		{
			Stop();
			ROS_INFO_STREAM("已经到了A点 ！！！\n");
			if (abs(dsp_status.YDist - ptA.y) < 80)
			{
				temp_normal_time = normal_time;
				intention_num = 7;
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 61;
			}
		}
		break;

		/*case 60:
ROS_INFO_STREAM("case 60 112 112 112 112 112 112 112 112 112 112 112 112 112 112\n");
		if(ToAngleByMPU6050(ptA))
		{
			Stop();
			temp_normal_time=normal_time;
			intention_num=61;
		}
		break;*/

	case 61:
		ROS_INFO_STREAM("case 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 \n");
		if (ToPostureByMilemeter_Pid6(ptA))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 7;
		}
		break;

	case 7:
		ROS_INFO_STREAM("case 7777777777777777777777777777777777777777777777777777777777777777777777777777\n");
		Control.SendData_state = 0; //单片机位置0
		if (1 == place_num)
		{
			if (ToAngleByMPU(0))
			{
				Stop();
				ROS_INFO_STREAM("已经调整到了0度 ！！！\n");
				temp_normal_time = normal_time;
				intention_num = 8;
			}
		}
		if (2 == place_num)
		{
			if (ToAngleByMPU(-1))
			{
				Stop();
				ROS_INFO_STREAM("已经调整到了-4度 ！！！\n");
				temp_normal_time = normal_time;
				intention_num = 8;
			}
		}
		break;

		/*********************** 中线识别球 ******************************/
	case 8:
		ROS_INFO_STREAM("case 8888888888888888888888888888888888888888888888888888888888888888888888888888\n");
		if ((normal_time - temp_normal_time) > 5)
		{
			//TODO:发送接收数据
			/*
			if(!Vision.create_communication_RAM_2())
			{
				ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
				if(!Vision.open_file_mapping_2())
				{
					ROS_INFO_STREAM("fail to open_file_mapping_2\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to open_file_mapping_2\n");
					if(!Vision.create_communication_RAM_4())
					{
						ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
						if(!Vision.open_file_mapping_4())
						{
							ROS_INFO_STREAM("fail to open_file_mapping_4\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to open_file_mapping_4\n");
							TheObject.whatObject =(objectType)8;//找什么东西在这里改
							if(!Vision.send_find_whatobject(TheObject.whatObject))
							{
								ROS_INFO_STREAM("Fail to send command\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to send command\n");
								TheObject.whatObject = (objectType)0;//变量重新初始化，为下一次找球做准备
								if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
								{
									ROS_INFO_STREAM("Fail to receive final object result\n");
								}
								else
								{
									ROS_INFO_STREAM("Success to receive final object result\n");
									ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
									ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
								}
							}
						}
					}
				}
			}*/

			if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
			{
				findballflag = 0;
				aimballflag = 0;
				if (1 == place_num)
				{
					if (abs((dsp_status.YDist) - (-800)) < 50)
					{
						Stop();
						ROS_INFO_STREAM("已经到边界了，但啥也没找着 ！！！\n");
						temp_normal_time = normal_time;
						//intention_num=103;
					}
					else
					{
						ROS_INFO_STREAM("横向左移找球...\n");
						robotstraightleft(); //继续走
					}
				}
				if (2 == place_num)
				{
					if (abs((dsp_status.YDist) - 800) < 50)
					{
						Stop();
						ROS_INFO_STREAM("已经到边界了，但啥也没找着 ！！！\n");
						temp_normal_time = normal_time;
						//intention_num=103;
					}
					else
					{
						ROS_INFO_STREAM("横向右移找球...\n");
						robotstraightright(); //继续走
					}
				}
			}
			else
			{
				findballflag = 1;
				ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
			}
			if ((findballflag == 1) && (aimballflag == 0))
			{
				ROS_INFO_STREAM(" cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
				if (ToAimBallByVision())
				{
					ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //对准球的过程中突然丢了
						ROS_INFO("丢了 丢了 丢了 ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
						findballflag = 0;
						aimballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 8;
					}
					else
					{
						aimballflag = 1;
						ROS_INFO("!!!中圈球:objectAngle aimedaimedaimed is %lf ; 中圈球:distance aimedaimedaimed is %ld!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					}
				}
			}
			if ((findballflag == 1) && (aimballflag == 1))
			{
				ROS_INFO_STREAM(" ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n");
				if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
				{
					ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //前往球的跟前过程中，又没看到球，重新去找
						aimballflag = 0;
						findballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 8;
						ROS_INFO_STREAM(" 又丢了 又丢了 又丢了 ！！！\n");
					}
					else
					{
						aimballflag = 0;
						findballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 9;
						ROS_INFO_STREAM("捡球 捡球 捡球 ！！！\n");
					}
				}
			}
		}
		break;

	case 9:
		ROS_INFO_STREAM("case 9999999999999999999999999999999999999999999999999999999999999999999999999999\n");
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令
		ROS_INFO_STREAM("case uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuppppppppppppppppppppppppppppppppppppp\n");
		temp_normal_time = normal_time;
		intention_num = 10; //抬机械臂命令已发送
		break;

	case 10: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 40)
		{
			ROS_INFO_STREAM("机械臂抬起，等待延时...\n");
			temp_normal_time = normal_time;
			intention_num = 11;
		}
		break;

	case 11:
		ROS_INFO_STREAM("case 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 \n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中发第一次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			ROS_INFO_STREAM("第一次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 12;
		}
		break;

	case 12:
		ROS_INFO_STREAM("case 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			temp_normal_time = normal_time;
			intention_num = 13;
			ROS_INFO_STREAM("发现已经捡到球了，架子上有球！！！\n");
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			temp_normal_time = normal_time;
			intention_num = 14;
			ROS_INFO_STREAM("判断没有球啊！等待抬臂稳定后在判断一遍！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 13:
		ROS_INFO_STREAM("case 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 \n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 16;
		}
		break;

	case 14:
		ROS_INFO_STREAM("case 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 \n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("第二次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 15;
		}
		break;

	case 15:
		ROS_INFO_STREAM("case 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 \n");
		Control.SendData_state = 0; //单片机位置0
		ROS_INFO_STREAM("第二次，判断架子上有没有球！！！！\n");
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("发现有球啊！！！第一次误判！！！\n");
			temp_normal_time = normal_time;
			intention_num = 16;
		}
		else //判断两次都没球，可以再去找了
		{
			findballflag = 0;
			aimballflag = 0;
			temp_normal_time = normal_time;
			intention_num = 8;
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 16:
		ROS_INFO_STREAM("case 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 \n");
		if (ToPostureByMilemeter_Pid2(ptA)) //捡了球之后直接漂移到A点
		{
			Stop();
			ROS_INFO_STREAM("已经到了A点 ！！！\n");
			temp_normal_time = normal_time;
			intention_num = 17;
		}
		break;

	case 17:
		ROS_INFO_STREAM("case 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 \n");
		if (ToAngleByMPU6050(ptD2))
		{
			if (stop())
			{
				ROS_INFO_STREAM("已经到达D2点！！！ \n");
				temp_normal_time = normal_time;
				intention_num = 171;
			}
		}
		break;

	case 171:
		ROS_INFO_STREAM("case 171 171 171 171 171 171 171 171 171 171 171 171 171 171 171 \n");
		if (ToPostureByMilemeter_Pid3(ptD2))
		{
			Stop();
			if ((abs(dsp_status.YDist - ptD2.y) > 80))
			{
				ROS_INFO_STREAM("y 没有达到，需要向y再走点\n");
				temp_normal_time = normal_time;
				intention_num = 175;
			}
			else
			{
				ROS_INFO_STREAM("y 到了！！！ \n");
				temp_normal_time = normal_time;
				intention_num = 172;
			}
		}
		break;

		/*case 173:
ROS_INFO_STREAM("case 173 112 112 112 112 112 112 112 112 112 112 112 112 112 112\n");
		if(ToAngleByMPU6050(ptD2))
		{
			Stop();
			temp_normal_time=normal_time;
			intention_num=175;
		}
		break;*/

	case 175:
		ROS_INFO_STREAM("case 175 175 175 175 175 175 175 175 175 175 175 175 175 175 175 175 \n");
		if (ToPostureByMilemeter_Pid6(ptD2))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 172;
		}
		break;

	case 172:
		ROS_INFO_STREAM("case 172 172 172 172 172 172 172 172 172 172 172 172 172 172 172 172 \n");
		if (1 == place_num)
		{
			if (ToAngleByMPU(100))
			{
				Stop();
				temp_normal_time = normal_time;
				intention_num = 18;
			}
		}
		if (2 == place_num)
		{
			if (ToAngleByMPU(-100))
			{
				Stop();
				temp_normal_time = normal_time;
				intention_num = 18;
			}
		}
		break;

		/************************** 第二次找标定柱 **************************/
	case 18:
		ROS_INFO_STREAM("case 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 \n ");
		//TODO:发送接收数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
						TheObject.whatObject = (objectType)1;	//找什么东西在这里改
						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to send command\n");
							TheObject.whatObject = (objectType)0;//变量初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");	
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}*/

		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				if ((normal_time - temp_normal_time) < 100)
					ToFindObjectByturnleft();
				else
					ToFindObjectByturnright();
			}
			if (2 == place_num)
			{
				if ((normal_time - temp_normal_time) < 100)
					ToFindObjectByturnright();
				else
					ToFindObjectByturnleft();
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((00.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 18;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!标注杆:objectAngle aimedaimedaimed is %lf;标注杆:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 40)) && (aimballflag == 1))
				{
					Stop();
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("!!!!!!!!标定柱:The distance has been to the min error;The distance has been to the min error!!!!!!!!\n");
					temp_normal_time = normal_time;
					intention_num = 19;
				}
			}
		}
		break;

	case 19: //发射
		ROS_INFO_STREAM("case 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 \n");
		if ((normal_time - temp_normal_time) > 22)
		{
			Stop();
			Control.SendData_state = 3; //发送发射指令，下一步要立即置位0
			ROS_INFO_STREAM("发送弹球指令 ！！！\n");
			temp_normal_time = normal_time;
			intention_num = 20;
		}
		break;

	case 20:
		ROS_INFO_STREAM("case 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 ");
		Control.SendData_state = 0; //单片机位置0
		Stop();
		break;

	default:
		normal_time = 0;
		break;
	}
}

void DecisionMaking::Normal_bat1_new()
{
	//一些决策相关的标志
	static int intention_num = 0;	 //回合过程的意图顺序号
	static bool findballflag = 0;	 //找到目标标志位,每找完一次目标后都要记得清零
	static bool aimballflag = 0;	 //对准目标标志位，每对准一次目标后都要记得清零
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;

	static bool Iscircleavoidancecorner = 0; //中圈避障是否向边缘靠近了

	Posture ptA0, ptA, ptC, ptD, ptD2, ptE, ptZ, ptD3, ptD4; //ptZ是中圈有障碍球时临时确定的辅助点
	//Posture pt00 = {1000,0};
	ptA0.x = 1000;
	ptA0.y = 0;

	if (1 == place_num)
	{
		ptA.x = 3200; //第二个球扫描起始定位点
		ptA.y = 3200;

		ptC.x = 4600; //投篮标定柱位置
		ptC.y = 10925;

		ptD.x = 4600; //投篮正中定位点（激光距离篮筐2400）（钢板400计，激光超出30计）
		ptD.y = 7500;

		ptE.x = 1890; //投篮45偏角位置（预留出车身宽度和激光距离，分别400和30计）（2630处）
		ptE.y = 9065;

		//ptE.x=2053;			//投篮45偏角位置（2400处）
		//ptE.y=9228;
	}
	if (2 == place_num)
	{
		ptA.x = 3300; //第二个球扫描起始定位点
		ptA.y = -3200;

		ptC.x = 4000; //投篮标定柱位置
		ptC.y = -10925;

		ptD.x = 3400; //投篮正中定位点（激光距离篮筐2400）（钢板400计，激光超出30计）
		ptD.y = -8100;

		ptD2.x = 3300;
		ptD2.y = -8100;

		/*ptD3.x=3600;
		ptD3.y=-7600;*/

		ptE.x = 1890; //投篮45偏角位置（预留出车身宽度和激光距离，分别400和30计）（2630处）
		ptE.y = -9065;
	}
	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 00000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;

	case 120:
		if ((normal_time - temp_normal_time) > 200) //延时17.1s
		{
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA0))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 101;
		}
		break;

	case 101:
		ROS_INFO_STREAM("case 111 111 111 111 111 111 111 111 111 111 111 111 111 111 111\n");
		if (ToAngleByMPU6050(ptD))
		{
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 102;
			}
		}
		break;

	case 102:
		ROS_INFO_STREAM("case 112 112 112 112 112 112 112 112 112 112 112 112 112 112 112\n");
		if (ToPostureByMilemeter_Pid3(ptD))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

		/********************************* 第一次识别标定柱 *********************************/
	case 2:
		ROS_INFO_STREAM("case 22222222222222222222222222222222222222222222222222222222222222222222222222222\n ");
		//TODO:发送接收数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
						TheObject.whatObject = (objectType)1;	//找什么东西在这里改
						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to send command\n");
							TheObject.whatObject = (objectType)0;//变量初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");	
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}*/

		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				ToFindObjectByturnright();
			}
			if (2 == place_num)
			{
				ToFindObjectByturnleft();
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((00.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 2;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!标注杆:objectAngle aimedaimedaimed is %lf;标注杆:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					Stop();
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("!!!!!!!!标定柱:The distance has been to the min error;The distance has been to the min error!!!!!!!!\n");
					temp_normal_time = normal_time;
					intention_num = 3;
				}
			}
		}
		break;

	case 3: //发射
		ROS_INFO_STREAM("case 33333333333333333333333333333333333333333333333333333333333333333333333333333\n");
		if ((normal_time - temp_normal_time) > 22)
		{
			Stop();
			Control.SendData_state = 3; //发送发射指令，下一步要立即置位0
			ROS_INFO_STREAM("发送弹球指令 ！！！\n");
			temp_normal_time = normal_time;
			intention_num = 5;
		}
		break;

	case 5:
		ROS_INFO_STREAM("case 5555555555555555555555555555555555555555555555555555555555555555555555555555\n");
		Control.SendData_state = 0; //单片机位置0
		if (normal_time - temp_normal_time > 10)
		{
			if (ToAngleByMPU6050(ptA))
			{
				Stop();
				ROS_INFO_STREAM("已经对准了A点 ！！！\n");
				temp_normal_time = normal_time;
				intention_num = 6;
			}
		}
		break;

	case 6:
		ROS_INFO_STREAM("case 6666666666666666666666666666666666666666666666666666666666666666666666666666\n");
		if (ToPostureByMilemeter_Pid3(ptA))
		{
			Stop();
			ROS_INFO_STREAM("已经到了A点 ！！！\n");
			if (abs(dsp_status.YDist - ptA.y) < 80)
			{
				temp_normal_time = normal_time;
				intention_num = 7;
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 61;
			}
		}
		break;

		/*case 60:
ROS_INFO_STREAM("case 60 112 112 112 112 112 112 112 112 112 112 112 112 112 112\n");
		if(ToAngleByMPU6050(ptA))
		{
			Stop();
			temp_normal_time=normal_time;
			intention_num=61;
		}
		break;*/

	case 61:
		ROS_INFO_STREAM("case 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 \n");
		if (ToPostureByMilemeter_Pid6(ptA))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 7;
		}
		break;

	case 7:
		ROS_INFO_STREAM("case 7777777777777777777777777777777777777777777777777777777777777777777777777777\n");
		Control.SendData_state = 0; //单片机位置0
		if (1 == place_num)
		{
			if (ToAngleByMPU(0))
			{
				Stop();
				ROS_INFO_STREAM("已经调整到了0度 ！！！\n");
				temp_normal_time = normal_time;
				intention_num = 8;
			}
		}
		if (2 == place_num)
		{
			if (ToAngleByMPU(-4))
			{
				Stop();
				ROS_INFO_STREAM("已经调整到了-4度 ！！！\n");
				temp_normal_time = normal_time;
				intention_num = 8;
			}
		}
		break;

		/*********************** 中线识别球 ******************************/
	case 8:
		ROS_INFO_STREAM("case 8888888888888888888888888888888888888888888888888888888888888888888888888888\n");
		if ((normal_time - temp_normal_time) > 5)
		{
			//TODO:发送接收数据
			/*
			if(!Vision.create_communication_RAM_2())
			{
				ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
				if(!Vision.open_file_mapping_2())
				{
					ROS_INFO_STREAM("fail to open_file_mapping_2\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to open_file_mapping_2\n");
					if(!Vision.create_communication_RAM_4())
					{
						ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
						if(!Vision.open_file_mapping_4())
						{
							ROS_INFO_STREAM("fail to open_file_mapping_4\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to open_file_mapping_4\n");
							TheObject.whatObject =(objectType)8;//找什么东西在这里改
							if(!Vision.send_find_whatobject(TheObject.whatObject))
							{
								ROS_INFO_STREAM("Fail to send command\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to send command\n");
								TheObject.whatObject = (objectType)0;//变量重新初始化，为下一次找球做准备
								if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
								{
									ROS_INFO_STREAM("Fail to receive final object result\n");
								}
								else
								{
									ROS_INFO_STREAM("Success to receive final object result\n");
									ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
									ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
								}
							}
						}
					}
				}
			}
			*/

			if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
			{
				findballflag = 0;
				aimballflag = 0;
				if (1 == place_num)
				{
					if (abs((dsp_status.YDist) - (-800)) < 50)
					{
						Stop();
						ROS_INFO_STREAM("已经到边界了，但啥也没找着 ！！！\n");
						temp_normal_time = normal_time;
						//intention_num=103;
					}
					else
					{
						ROS_INFO_STREAM("横向左移找球...\n");
						robotstraightleftlow(); //继续走
					}
				}
				if (2 == place_num)
				{
					if (abs((dsp_status.YDist) - 800) < 50)
					{
						Stop();
						ROS_INFO_STREAM("已经到边界了，但啥也没找着 ！！！\n");
						temp_normal_time = normal_time;
						//intention_num=103;
					}
					else
					{
						ROS_INFO_STREAM("横向右移找球...\n");
						robotstraightrightlow(); //继续走
					}
				}
			}
			else
			{
				findballflag = 1;
				ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
			}
			if ((findballflag == 1) && (aimballflag == 0))
			{
				ROS_INFO_STREAM(" cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
				if (ToAimBallByVision())
				{
					ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //对准球的过程中突然丢了
						ROS_INFO("丢了 丢了 丢了 ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
						findballflag = 0;
						aimballflag = 0;
						intention_num = 8;
					}
					else
					{
						aimballflag = 1;
						ROS_INFO("!!!中圈球:objectAngle aimedaimedaimed is %lf ; 中圈球:distance aimedaimedaimed is %ld!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					}
				}
			}
			if ((findballflag == 1) && (aimballflag == 1))
			{
				ROS_INFO_STREAM(" ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n");
				if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
				{
					ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //前往球的跟前过程中，又没看到球，重新去找
						aimballflag = 0;
						findballflag = 0;
						intention_num = 8;
						ROS_INFO_STREAM(" 又丢了 又丢了 又丢了 ！！！\n");
					}
					else
					{
						aimballflag = 0;
						findballflag = 0;
						intention_num = 9;
						ROS_INFO_STREAM("捡球 捡球 捡球 ！！！\n");
					}
				}
			}
		}
		break;

	case 9:
		ROS_INFO_STREAM("case 9999999999999999999999999999999999999999999999999999999999999999999999999999\n");
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令
		ROS_INFO_STREAM("case uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuppppppppppppppppppppppppppppppppppppp\n");
		temp_normal_time = normal_time;
		intention_num = 10; //抬机械臂命令已发送
		break;

	case 10: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 40)
		{
			ROS_INFO_STREAM("机械臂抬起，等待延时...\n");
			temp_normal_time = normal_time;
			intention_num = 11;
		}
		break;

	case 11:
		ROS_INFO_STREAM("case 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 \n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中发第一次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			ROS_INFO_STREAM("第一次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 12;
		}
		break;

	case 12:
		ROS_INFO_STREAM("case 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			temp_normal_time = normal_time;
			intention_num = 13;
			ROS_INFO_STREAM("发现已经捡到球了，架子上有球！！！\n");
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			temp_normal_time = normal_time;
			intention_num = 14;
			ROS_INFO_STREAM("判断没有球啊！等待抬臂稳定后在判断一遍！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 13:
		ROS_INFO_STREAM("case 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 \n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 16;
		}
		break;

	case 14:
		ROS_INFO_STREAM("case 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 \n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			//m_bSend = 1;				//TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("第二次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 15;
		}
		break;

	case 15:
		ROS_INFO_STREAM("case 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 \n");
		Control.SendData_state = 0; //单片机位置0
		ROS_INFO_STREAM("第二次，判断架子上有没有球！！！！\n");
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("发现有球啊！！！第一次误判！！！\n");
			temp_normal_time = normal_time;
			intention_num = 16;
		}
		else //判断两次都没球，可以再去找了
		{
			findballflag = 0;
			aimballflag = 0;
			intention_num = 8;
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 16:
		ROS_INFO_STREAM("case 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 \n");
		if (ToPostureByMilemeter_Pid2(ptA)) //捡了球之后直接漂移到A点
		{
			Stop();
			ROS_INFO_STREAM("已经到了A点 ！！！\n");
			temp_normal_time = normal_time;
			intention_num = 17;
		}
		break;

	case 17:
		ROS_INFO_STREAM("case 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 \n");
		if (ToAngleByMPU6050(ptD2))
		{
			if (stop())
			{
				ROS_INFO_STREAM("已经到达D2点！！！ \n");
				temp_normal_time = normal_time;
				intention_num = 171;
			}
		}
		break;

	case 171:
		ROS_INFO_STREAM("case 171 171 171 171 171 171 171 171 171 171 171 171 171 171 171 \n");
		if (ToPostureByMilemeter_Pid3(ptD2))
		{
			Stop();
			if ((abs(dsp_status.YDist - ptD2.y) > 80))
			{
				ROS_INFO_STREAM("y 没有达到，需要向y再走点\n");
				temp_normal_time = normal_time;
				intention_num = 175;
			}
			else
			{
				ROS_INFO_STREAM("y 到了！！！ \n");
				temp_normal_time = normal_time;
				intention_num = 172;
			}
		}
		break;

		/*case 173:
ROS_INFO_STREAM("case 173 112 112 112 112 112 112 112 112 112 112 112 112 112 112\n");
		if(ToAngleByMPU6050(ptD2))
		{
			Stop();
			temp_normal_time=normal_time;
			intention_num=175;
		}
		break;*/

	case 175:
		ROS_INFO_STREAM("case 175 175 175 175 175 175 175 175 175 175 175 175 175 175 175 175 \n");
		if (ToPostureByMilemeter_Pid6(ptD2))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 18;
		}
		break;

	case 172:
		ROS_INFO_STREAM("case 172 172 172 172 172 172 172 172 172 172 172 172 172 172 172 172 \n");
		if (ToAngleByMPU(-60))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 18;
		}
		break;

		/************************** 第二次找标定柱 **************************/
	case 18:
		ROS_INFO_STREAM("case 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 \n ");
		//TODO:接受发送数据
		/*
		if (!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if (!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if (!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if (!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
						TheObject.whatObject = (objectType)1; //找什么东西在这里改
						if (!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to send command\n");
							TheObject.whatObject = (objectType)0; //变量初始化，为下一次找球做准备
							if (Vision.receive_final_object_result(objectAngleDistance_Y) == false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n", objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n", objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}
		*/

		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				ToFindObjectByturnright();
			}
			if (2 == place_num)
			{
				ToFindObjectByturnleft();
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((00.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 18;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!标注杆:objectAngle aimedaimedaimed is %lf;标注杆:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					Stop();
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("!!!!!!!!标定柱:The distance has been to the min error;The distance has been to the min error!!!!!!!!\n");
					temp_normal_time = normal_time;
					intention_num = 19;
				}
			}
		}
		break;

	case 19: //发射
		ROS_INFO_STREAM("case 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 \n");
		if ((normal_time - temp_normal_time) > 22)
		{
			Stop();
			Control.SendData_state = 3; //发送发射指令，下一步要立即置位0
			ROS_INFO_STREAM("发送弹球指令 ！！！\n");
			temp_normal_time = normal_time;
			intention_num = 20;
		}
		break;

	case 20:
		ROS_INFO_STREAM("case 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 ");
		Control.SendData_state = 0; //单片机位置0
		Stop();
		break;

	default:
		normal_time = 0;
		break;
	}
}

void DecisionMaking::Normal_bat2() //**********投篮-回合2**********
{
	//一些决策相关的标志
	static int intention_num = 0;	 //回合过程的意图顺序号
	static bool findballflag = 0;	 //找到目标标志位,每找完一次目标后都要记得清零
	static bool aimballflag = 0;	 //对准目标标志位，每对准一次目标后都要记得清零
	static bool refound = 0;		 //重新开始中线找球标志
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptA0, ptA, ptC, ptD, ptM, ptCC, ptDD, ptD2;
	static Posture ptN = {0, 0}; // B1-N

	ptA0.x = 1000; //出发前，一个定点
	ptA0.y = 0;

	if (1 == place_num)
	{
		ptA.x = 3200; //中点，扫描起点
		ptA.y = 4600;

		ptD.x = 3200;
		ptD.y = 8100; //第一次投球点

		ptC.x = 4250;
		ptC.y = 10925; //投球目标点

		ptM.x = 3400;
		ptM.y = 3200; //中圈置球开始扫描点

		ptCC.x = 800;
		ptCC.y = 7500; //边界点（临界主要用到X）

		ptDD.x = 3200;
		ptDD.y = -1000; //中线区找球边界点（临界主要用到Y）

		ptD2.x = 3100;
		ptD2.y = 8100; //第二次投球点
	}

	if (2 == place_num)
	{
		ptA.x = 3500; //中点，扫描起点
		ptA.y = -5000;

		ptD.x = 3200;
		ptD.y = -8100; //第一次投球点

		ptC.x = 4250;
		ptC.y = -10925; //投球目标点

		ptM.x = 3600;
		ptM.y = -3200; //中圈置球开始扫描点(done)

		ptCC.x = 800;
		ptCC.y = -7500; //边界点（临界主要用到X）

		ptDD.x = 3200;
		ptDD.y = 1000; //中线区找球边界点（临界主要用到Y）

		ptD2.x = 3200;
		ptD2.y = -8100; //第二次投球点
	}

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000\n");
		if ((normal_time - temp_normal_time) > 200) //延时大约9s
		{
			temp_normal_time = normal_time;
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA0))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 222222222222222222222222222222222222222222222222222222222222222\n");
		if (ToAngleByMPU6050(ptA))
		{
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 201;
			}
		}
		break;

	case 201:
		ROS_INFO_STREAM("case 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 \n");
		if ((normal_time - temp_normal_time) > 5)
		{
			if (ToPostureByMilemeter_Pid3(ptA))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 202;
				}
			}
		}
		break;

	case 202: //使车子成66度倾斜，为横向移动找三分球做准备
		ROS_INFO_STREAM("case 202 202 202 202 202 202 202 202 202 202 202 202 202 202 202 202 202 202 202 \n");
		/*ROS_INFO_STREAM("正在旋转！！！\n");
		if(ToAngleByMPU(66))
		{
			ROS_INFO_STREAM("转到66度了！！！\n");
			if(stop())
			{
				ROS_INFO_STREAM("停稳！！！\n");
				temp_normal_time=normal_time;
				intention_num=4;
			}							
		}*/
		if ((normal_time - temp_normal_time) > 5)
		{
			if (1 == place_num)
			{
				ROS_INFO_STREAM("角度调整过程ing................\n");
				if (fabs(dsp_status.fAngle - 64.00) > 2)
				{
					ROS_INFO_STREAM("和60度相差大于2度................\n");
					if (dsp_status.fAngle > 66)
					{
						ROS_INFO_STREAM("角度为正，需要左旋................\n");
						ToFindObjectByturnleft();
					}
					if (dsp_status.fAngle < 62)
					{
						ROS_INFO_STREAM("角度为负，需要右旋................\n");
						ToFindObjectByturnright();
					}
				}
				else
				{
					if (stop())
					{
						ROS_INFO_STREAM("角度为64度，不要动了！！！................\n");
						temp_normal_time = normal_time;
						intention_num = 5;
					}
				}
			}
			if (2 == place_num)
			{
				ROS_INFO_STREAM("角度调整过程ing................\n");
				if (fabs(dsp_status.fAngle - (-66.00)) > 2)
				{
					ROS_INFO_STREAM("和-66度相差大于2度................\n");
					if (dsp_status.fAngle > -64)
					{
						ROS_INFO_STREAM("角度为正，需要左旋................\n");
						ToFindObjectByturnleft();
					}
					if (dsp_status.fAngle < -68)
					{
						ROS_INFO_STREAM("角度为负，需要右旋................\n");
						ToFindObjectByturnright();
					}
				}
				else
				{
					if (stop())
					{
						ROS_INFO_STREAM("角度为-64度，不要动了！！！................\n");
						temp_normal_time = normal_time;
						intention_num = 5;
					}
				}
			}
		}
		break;

		/************************** 三分线找球 *************************/
	case 5:
		ROS_INFO_STREAM("case  5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 \n");
		if ((normal_time - temp_normal_time) > 5)
		{
			//TODO:发送接收数据
			/*
			if(!Vision.create_communication_RAM_2())
			{
				ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
				if(!Vision.open_file_mapping_2())
				{
					ROS_INFO_STREAM("fail to open_file_mapping_2\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to open_file_mapping_2\n");
					if(!Vision.create_communication_RAM_4())
					{
						ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
						if(!Vision.open_file_mapping_4())
						{
							ROS_INFO_STREAM("fail to open_file_mapping_4\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to open_file_mapping_4\n");
							TheObject.whatObject =(objectType)8;//找什么东西在这里改
							if(!Vision.send_find_whatobject(TheObject.whatObject))
							{
								ROS_INFO_STREAM("Fail to send command\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to send command\n");
								TheObject.whatObject = (objectType)0;//变量重新初始化，为下一次找球做准备
								if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
								{
									ROS_INFO_STREAM("Fail to receive final object result\n");
								}
								else
								{
									ROS_INFO_STREAM("Success to receive final object result\n");
									ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
									ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
								}
							}
						}
					}
				}
			}
			*/

			if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
			{
				findballflag = 0;
				aimballflag = 0;
				if (1 == place_num)
				{
					if (abs((dsp_status.XDist) - (ptCC.x)) < 50)
					{
						Stop();
						ROS_INFO_STREAM("已经到边界了，但啥也没找着，算了，去找中线区的球！！！\n");
						temp_normal_time = normal_time;
						intention_num = 18;
					}
					else
					{
						ROS_INFO_STREAM("以66度方向行进...\n");
						robotstraightright(); //继续右走
					}
				}
				if (2 == place_num)
				{
					if (abs((dsp_status.XDist) - (ptCC.x)) < 50)
					{
						Stop();
						ROS_INFO_STREAM("已经到边界了，但啥也没找着，算了，去找中线区的球！！！\n");
						temp_normal_time = normal_time;
						intention_num = 18; //去找中线区的球！！
					}
					else
					{
						ROS_INFO_STREAM("以-66度方向行进...\n");
						robotstraightleft(); //继续左走
					}
				}
			}
			else
			{
				findballflag = 1;
				ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
			}
			if ((findballflag == 1) && (aimballflag == 0))
			{
				ROS_INFO_STREAM(" cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
				if (ToAimBallByVision())
				{
					ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //对准球的过程中突然丢了
						findballflag = 0;
						aimballflag = 0;
						ROS_INFO("丢了 丢了 丢了 ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 5;
						}
					}
					else
					{
						aimballflag = 1;
						ROS_INFO("!!!!!!!!三分线球:objectAngle aimedaimedaimed is %lf;三分线球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					}
				}
			}
			if ((findballflag == 1) && (aimballflag == 1))
			{
				ROS_INFO_STREAM(" ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n");
				if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
				{
					ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //前往球的跟前过程中，又没看到球，重新去找
						aimballflag = 0;
						findballflag = 0;
						ROS_INFO_STREAM(" 又丢了 又丢了 又丢了 ！！！\n");
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 5;
						}
					}
					else
					{
						aimballflag = 0;
						findballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 6;
						ROS_INFO_STREAM("捡球 捡球 捡球 ！！！\n");
					}
				}
			}
		}
		break;

	case 6:
		ROS_INFO_STREAM("case 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 \n");
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令
		ROS_INFO_STREAM("case uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuppppppppppppppppppppppppppppppppppppp\n");
		temp_normal_time = normal_time;
		intention_num = 7; //抬机械臂命令已发送
		break;

	case 7: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 40)
		{
			ROS_INFO_STREAM("机械臂抬起，等待延时...\n");
			temp_normal_time = normal_time;
			intention_num = 8;
		}
		break;

	case 8:
		ROS_INFO_STREAM("case 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 \n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中发第一次判断命令
		{
			//m_bSend=1;    //TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			ROS_INFO_STREAM("第一次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 9;
		}
		break;

	case 9:
		ROS_INFO_STREAM("case 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			temp_normal_time = normal_time;
			intention_num = 10;
			ROS_INFO_STREAM("发现已经捡到球了，架子上有球！！！\n");
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			temp_normal_time = normal_time;
			intention_num = 100;
			ROS_INFO_STREAM("判断没有球啊！等待抬臂稳定后在判断一遍！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 10:
		ROS_INFO_STREAM("case 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 \n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 11;
		}
		break;

	case 100:
		ROS_INFO_STREAM("100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 \n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("第二次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 101;
		}
		break;

	case 101:
		ROS_INFO_STREAM("case 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 \n");
		Control.SendData_state = 0; //单片机位置0
		ROS_INFO_STREAM("第二次，判断架子上有没有球！！！！\n");
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("发现有球啊！！！第一次误判！！！\n");
			temp_normal_time = normal_time;
			intention_num = 11;
		}
		else //判断两次都没球，可以再去找了
		{
			findballflag = 0;
			aimballflag = 0;
			intention_num = 5;
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 11: //准备走到投球点
		ROS_INFO_STREAM("case 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 \n");
		ROS_INFO_STREAM("正在对准投球定位点！！！！\n");
		if (ToAngleByMPU6050(ptD))
		{
			ROS_INFO_STREAM("已经对准了投球定位点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停到了投球定位点，再旋转调整！！！！\n");
				ptN.x = dsp_status.XDist; //把当前坐标赋给点N
				ptN.y = dsp_status.YDist;
				ROS_INFO("点N的坐标为：%ld , %ld\n", ptN.x, ptN.y);

				temp_normal_time = normal_time;
				intention_num = 12;
			}
		}
		break;

	case 12:
		ROS_INFO_STREAM("case 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 \n");
		temp_normal_time = normal_time;
		if (ToPostureByMilemeter_Pid6(ptD)) //到达D点
		{
			Stop();
			ROS_INFO_STREAM("到 DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD\n");
			intention_num = 13;
		}
		break;

	case 13: //坐标对准投球目标点
		ROS_INFO_STREAM("case 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 \n");
		ROS_INFO_STREAM("正在坐标对准投球目标点！！！！\n");
		if (ToAngleByMPU6050(ptC))
		{
			ROS_INFO_STREAM("已经坐标对准了投球目标点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("坐标对准之后已经停下，视觉对准！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 14;
			}
		}
		break;

		/***************************** 第一次找标定柱 ******************************/
	case 14:
		ROS_INFO_STREAM("case 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 \n ");
		//TODO:发送接收数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
						TheObject.whatObject = (objectType)1;	//找什么东西在这里改
						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to send command\n");
							TheObject.whatObject = (objectType)0;//变量初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");	
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}
		*/

		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				if (normal_time - temp_normal_time < 100)
					ToFindObjectByturnright();
				else if (stop())
					ToFindObjectByturnleft();
			}
			if (2 == place_num)
			{
				if (normal_time - temp_normal_time < 100)
					ToFindObjectByturnleft();
				else if (stop())
					ToFindObjectByturnright();
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((00.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 14;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!标注杆:objectAngle aimedaimedaimed is %lf;标注杆:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 40)) && (aimballflag == 1))
				{
					stop();
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("!!!!!!!!标定柱:The distance has been to the min error;The distance has been to the min error!!!!!!!!\n");
					temp_normal_time = normal_time;
					intention_num = 140;
				}
			}
		}
		break;

	/*case 140:
		if(ToAngleByMPU(dsp_status.fAngle-3))
		{
			ROS_INFO_STREAM("已经对准了N点！！！！\n");
			if(stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，下一步要往前走！！！！\n");
				temp_normal_time=normal_time;
				intention_num=15;
			}							
		}		
		break;*/
	case 140:
		ROS_INFO_STREAM("case 140 140 140 140 140 140 140 140 140 140 140 140 140 140 140 140 140 \n");
		if (normal_time - temp_normal_time < 30)
		{
			ROS_INFO_STREAM("back!!!\n");
			robotbacklow();
		}
		else
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 15;
		}
		break;

	case 15: //准备发射排球
		ROS_INFO_STREAM("case 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 \n");
		if ((normal_time - temp_normal_time) > 22)
		{
			Control.SendData_state = 3; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("发射 发射 发射 ！！！！\n");

			temp_normal_time = normal_time;
			intention_num = 16;
		}
		break;

	case 16: //准备返回N点
		ROS_INFO_STREAM("case 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16  \n");
		ROS_INFO_STREAM("正在对准N点！！！！\n");
		Control.SendData_state = 0;
		if (ToAngleByMPU6050(ptN))
		{
			ROS_INFO_STREAM("已经对准了N点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，下一步要往前走！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 17;
			}
		}
		break;

	case 17:
		ROS_INFO_STREAM("case 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17  \n");

		if (ToPostureByMilemeter_Pid3(ptN)) //到达N点
		{
			Stop();
			ROS_INFO_STREAM("到 NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN\n");
			temp_normal_time = normal_time;
			intention_num = 18;
		}
		break;

	case 18: //准备到达M点开始扫描中线置球
		ROS_INFO_STREAM("case 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18  \n");
		ROS_INFO_STREAM("正在对准M点！！！！\n");
		if (ToAngleByMPU6050(ptM))
		{
			ROS_INFO_STREAM("已经对准了M点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，下一步要往前走！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 180;
			}
		}
		break;

	case 180:
		ROS_INFO_STREAM("case 180 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18  \n");
		if (ToPostureByMilemeter_Pid2(ptM)) //到达M点
		{
			Stop();
			ROS_INFO_STREAM("diyi到 MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM\n");
			temp_normal_time = normal_time;
			intention_num = 19;
		}
		break;

	case 19:
		ROS_INFO_STREAM("case 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 \n");

		if (ToPostureByMilemeter_Pid5(ptM)) //到达M点
		{
			Stop();
			ROS_INFO_STREAM("到 MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM\n");
			temp_normal_time = normal_time;
			intention_num = 190;
		}
		break;

	case 190: //使车子成0度，为横向移动找中线球做准备
		ROS_INFO_STREAM("case 190 190 190 190 190 190 190 190 190 190 190 190 190 190 190 190 190 190 \n");
		if (1 == place_num)
		{
			if (ToAngleByMPU(-2))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 20;
				}
			}
		}
		if (2 == place_num)
		{
			if (ToAngleByMPU(-1))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 20;
				}
			}
		}
		break;

		/***********************************  中线识球开始  ***********************************/
	case 20:
		ROS_INFO_STREAM("case 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 \n");
		//temp_normal_time=normal_time;     //为延时作准备
		//TODO:发送接收数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");

		//视觉修改版找球测试start
						TheObject.whatObject =(objectType)8;//找什么东西在这里改
						if(TheObject.whatObject==(objectType)1)
							ROS_INFO_STREAM("send command to find calibration\n");
						if(TheObject.whatObject==(objectType)2)
							ROS_INFO_STREAM("send command to find reddish_brown_basketball\n");
						if(TheObject.whatObject==(objectType)3)
							ROS_INFO_STREAM("send command to find basketball\n");
						if(TheObject.whatObject==(objectType)4)
							ROS_INFO_STREAM("send command to find blue_gray_basketball\n");
						if(TheObject.whatObject==(objectType)5)
							ROS_INFO_STREAM("send command to find calibration_plus\n");
						if(TheObject.whatObject==(objectType)6)
							ROS_INFO_STREAM("send command to find orange_red_volleyball\n");
						if(TheObject.whatObject==(objectType)7)
							ROS_INFO_STREAM("send command to find red_blue_volleyball\n");
						if(TheObject.whatObject==(objectType)8)
							ROS_INFO_STREAM("send command to find volleyball\n");

						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM(" Success to send command Success to send command Success to send command Success to send command\n");
							TheObject.whatObject = (objectType)0;//变量重新初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}
		*/

		if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
		{
			findballflag = 0;
			aimballflag = 0;
			//if(dsp_status.XDist>
			if (1 == place_num)
			{
				if ((dsp_status.XDist - ptA.x) > 80)
					robotbacktoolow();
				else if ((dsp_status.XDist - ptA.x) < (-80))
					robotforwardtoolow();
				else
				{
					if ((dsp_status.YDist < (-1000)) && (0 == refound)) //找了好久，一个都没找到，就回去了
					{
						ROS_INFO_STREAM("中圈第一遍走完了，也没找到球！！！\n");
						refound = 1;
					}
					else if ((dsp_status.YDist >= (-1000)) && (0 == refound))
					{
						robotstraightleft(); //开环横向低速右移
						ROS_INFO_STREAM(" 我在zuo移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 \n");
					}
					else if ((1 == refound) && ((dsp_status.YDist < 3500)))
					{
						robotstraightright(); //开环横向低速左移
						ROS_INFO_STREAM(" 我在you移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移  \n");
					}
					else
					{
						ROS_INFO_STREAM("中圈走完了，也没找到球！！！那就回家吧！！！\n");
						Stop();
						temp_normal_time = normal_time;
						intention_num = 2;
					}
				}
				//if (abs((dsp_status.YDist) - (ptDD.y))<50)
				//{
				//	Stop();
				//	ROS_INFO_STREAM("已经到边界了，但啥也没找着，停车，结束比赛！！！\n");
				//	temp_normal_time=normal_time;
				//	intention_num=33;         //停车，结束比赛 停车，结束比赛停车!!!!!!!!!!!!!!
				//}else
				//{
				//	ROS_INFO_STREAM("左移找球...\n");
				//	robotstraightleft();		//左移找球
				//}
			}
			if (2 == place_num)
			{
				if ((dsp_status.XDist - ptA.x) > 80)
					robotbacktoolow();
				else if ((dsp_status.XDist - ptA.x) < (-80))
					robotforwardtoolow();
				else
				{
					if ((dsp_status.YDist > 1000) && (0 == refound)) //找了好久，一个都没找到，就回去了
					{
						ROS_INFO_STREAM("中圈第一遍走完了，也没找到球！！！\n");
						refound = 1;
					}
					else if ((dsp_status.YDist <= 1000) && (0 == refound))
					{
						robotstraightright(); //开环横向低速右移
						ROS_INFO_STREAM(" 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 \n");
					}
					else if ((1 == refound) && ((dsp_status.YDist > (-3500))))
					{
						robotstraightleft(); //开环横向低速左移
						ROS_INFO_STREAM(" 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移 我在左移  \n");
					}
					else
					{
						ROS_INFO_STREAM("中圈走完了，也没找到球！！！那就回家吧！！！\n");
						Stop();
						temp_normal_time = normal_time;
						intention_num = 2;
					}
				}
				//if (abs((dsp_status.YDist) - (ptDD.y))<50)
				//{
				//	Stop();
				//	ROS_INFO_STREAM("已经到边界了，但啥也没找着，停车，结束比赛！！！\n");
				//	temp_normal_time=normal_time;
				//	intention_num=33;         //停车，结束比赛 停车，结束比赛停车!!!!!!!!!!!!!!
				//}else
				//{
				//	ROS_INFO_STREAM("右移找球...\n");
				//	robotstraightright();		//右移找球
				//}
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM(" findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					ROS_INFO("丢了 丢了 丢了 : %lf \n", objectAngleDistance_Y.first);
					if ((normal_time - temp_normal_time) > 2)
					{
						temp_normal_time = normal_time; //为延时作准备
						intention_num = 20;
					}
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!中圈球:objectAngle aimedaimedaimed is %lf;中圈球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1))
		{
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("又丢了 又丢了 又丢了 \n");
					if ((normal_time - temp_normal_time) > 2)
					{
						temp_normal_time = normal_time; //为延时作准备
						intention_num = 20;
					}
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 21;
					ROS_INFO_STREAM("捡球 捡球 捡球 \n");
				}
			}
		}
		break;

	case 21:
		ROS_INFO_STREAM("case 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 \n");
		//立即捡球，不要延时
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		ROS_INFO_STREAM("发送抬臂指令！！！");
		temp_normal_time = normal_time;
		intention_num = 22; //抬机械臂命令已发送
		break;

	case 22: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 40)
		{
			temp_normal_time = normal_time;
			intention_num = 23;
		}
		break;

	case 23:
		ROS_INFO_STREAM("case 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 \n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中就发第一次判断命令
		{
			//m_bSend=1;    //TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			ROS_INFO_STREAM("第一次，发送捡球判断！！！");
			temp_normal_time = normal_time;
			intention_num = 24;
		}
		break;

	case 24:
		ROS_INFO_STREAM("case 24 24 24 24 24 24 24 24 24 24 24 24 24 \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			ROS_INFO_STREAM("检测到有球！！！");
			temp_normal_time = normal_time;
			intention_num = 25;
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			ROS_INFO_STREAM("没检测到有球，在判断一次！！！");
			temp_normal_time = normal_time;
			intention_num = 250;
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 25:
		ROS_INFO_STREAM("case 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 \n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 252;
		}
		break;

	case 250:
		ROS_INFO_STREAM("case 250 250 250 250 250 250 250 250 250 250 250 250  \n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			//m_bSend=1;    //TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("第二次，发送捡球判断！！！");
			temp_normal_time = normal_time;
			intention_num = 251;
		}
		break;

	case 251:
		ROS_INFO_STREAM("case 251 251 251 251 251 251 251 251 251 251 251 251 251 251  \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("第一次为误判，明明有球！！！");
			temp_normal_time = normal_time;
			intention_num = 252;
		}
		else //判断两次都没球，可以再去找了
		{
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！");
			findballflag = 0;
			aimballflag = 0;
			temp_normal_time = normal_time;
			intention_num = 20;
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 252:
		ROS_INFO_STREAM("case 252 251 251 251 251 251 251 251 251 251 251 251 251 251  \n");
		if (normal_time - temp_normal_time < 22)
		{
			ROS_INFO_STREAM("后退中\n");
			robotbacklow();
		}
		else
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 253;
		}
		break;

	case 253:
		ROS_INFO_STREAM("case 253 251 251 251 251 251 251 251 251 251 251 251 251 251  \n");
		if (ToPostureByMilemeter_Pid2(ptM))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 26;
		}
		break;

	case 26: //准备返回B1点
		ROS_INFO_STREAM("case 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 \n");
		ROS_INFO_STREAM("正在对准N点！！！！\n");
		Control.SendData_state = 0;
		if (ToAngleByMPU6050(ptN))
		{
			ROS_INFO_STREAM("已经对准了N点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，下一步要往前走！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 27;
			}
		}
		break;

	case 27:
		ROS_INFO_STREAM("case 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 \n");

		if (ToPostureByMilemeter_Pid3(ptN)) //到达N点
		{
			Stop();
			ROS_INFO_STREAM("到 NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN\n");
			temp_normal_time = normal_time;
			intention_num = 28;
		}
		break;

	case 28: //准备到达投球点D
		ROS_INFO_STREAM("case 28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  \n");
		ROS_INFO_STREAM("正在对准D点！！！！\n");
		Control.SendData_state = 0;
		if (ToAngleByMPU6050(ptD2))
		{
			ROS_INFO_STREAM("已经对准了D2点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，下一步要往前走！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 29;
			}
		}
		break;

	case 29:
		ROS_INFO_STREAM("case 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 \n");

		if (ToPostureByMilemeter_Pid3(ptD2)) //到达D点
		{
			Stop();
			ROS_INFO_STREAM("到 DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD\n");
			temp_normal_time = normal_time;
			intention_num = 290;
		}
		break;

	case 290:
		ROS_INFO_STREAM("case 290 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 \n");
		if (ToPostureByMilemeter_Pid2(ptD2))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 291;
		}
		break;

	case 291:
		ROS_INFO_STREAM("case 291 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 \n");
		if (ToPostureByMilemeter_Pid6(ptD2))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 30;
		}
		break;

	case 30: //坐标对准投球目标点
		ROS_INFO_STREAM("case 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 \n");
		ROS_INFO_STREAM("正在坐标对准投球目标点！！！！\n");
		if (ToAngleByMPU6050(ptC))
		{
			ROS_INFO_STREAM("已经坐标对准了投球目标点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("坐标对准之后已经停下，视觉对准！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 31;
			}
		}
		break;

		/********************************* 第二次识别标定柱 *********************************/
	case 31:
		ROS_INFO_STREAM("case 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 \n ");
		//TODO:发送接收数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
						TheObject.whatObject = (objectType)1;	//找什么东西在这里改
						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to send command\n");
							TheObject.whatObject = (objectType)0;//变量初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");	
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}	
		}*/
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				if (normal_time - temp_normal_time < 100)
					ToFindObjectByturnright();
				else if (stop())
					ToFindObjectByturnleft();
			}
			if (2 == place_num)
			{
				if (normal_time - temp_normal_time < 100)
					ToFindObjectByturnleft();
				else if (stop())
					ToFindObjectByturnright();
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((00.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					intention_num = 31;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!标注杆:objectAngle aimedaimedaimed is %lf;标注杆:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 40)) && (aimballflag == 1))
				{
					stop();
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("!!!!!!!!标定柱:The distance has been to the min error;The distance has been to the min error!!!!!!!!\n");
					temp_normal_time = normal_time;
					intention_num = 32;
				}
			}
		}
		break;

	case 32: //准备发射排球
		ROS_INFO_STREAM("case 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32  \n");
		if ((normal_time - temp_normal_time) > 22)
		{
			Control.SendData_state = 3; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("发射 发射 发射 ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 33;
		}
		break;

	case 33: //停车，比赛结束！！！！！！！！！！！
		ROS_INFO_STREAM("case 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33  \n");
		Control.SendData_state = 0;
		Stop(); //停车
		break;

	default:
		ROS_INFO_STREAM("case： default ！！！ default ！！！ default ！！！ \n");
		normal_time = 0;
		break;
	}
}

void DecisionMaking::Normal_bat2_new()
{									 //一些决策相关的标志
	static int intention_num = 0;	 //回合过程的意图顺序号
	static bool findballflag = 0;	 //找到目标标志位,每找完一次目标后都要记得清零
	static bool aimballflag = 0;	 //对准目标标志位，每对准一次目标后都要记得清零
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptA0, ptA, ptC, ptD, ptM, ptCC, ptDD, ptD2;
	static Posture ptN = {0, 0}; // B1-N

	ptA0.x = 1000; //出发前，一个定点
	ptA0.y = 0;

	if (1 == place_num)
	{
		ptA.x = 4050; //中点，扫描起点
		ptA.y = 4600;

		ptD.x = 4600;
		ptD.y = 7500; //投球点,正中

		ptC.x = 4600;
		ptC.y = 10925; //投球目标点

		ptM.x = 3200;
		ptM.y = 3200; //中圈置球开始扫描点

		ptCC.x = 800;
		ptCC.y = 7500; //边界点（临界主要用到X）

		ptDD.x = 3200;
		ptDD.y = -500; //中线区找球边界点（临界主要用到Y）

		ptD2.x = 4900;
		ptD2.y = 7500;
	}

	if (2 == place_num)
	{
		ptA.x = 4050; //中点，扫描起点
		ptA.y = -4600;

		ptD.x = 3400;
		ptD.y = -8100; //投球点B2-D

		ptC.x = 4250;
		ptC.y = -10925; //投球目标点

		ptM.x = 3400;
		ptM.y = -3200; //中圈置球开始扫描点(done)

		ptCC.x = 1000;
		ptCC.y = -7500; //边界点（临界主要用到X）

		ptDD.x = 3200;
		ptDD.y = 500; //中线区找球边界点（临界主要用到Y）

		ptD2.x = 3300;
		ptD2.y = -8100;
	}

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000\n");
		if ((normal_time - temp_normal_time) > 200) //延时大约9s
		{
			temp_normal_time = normal_time;
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA0))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 222222222222222222222222222222222222222222222222222222222222222\n");
		if (ToAngleByMPU6050(ptA))
		{
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 201;
			}
		}
		break;

	case 201:
		ROS_INFO_STREAM("case 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 \n");
		if (ToPostureByMilemeter_Pid3(ptA))
		{
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 202;
			}
		}
		break;

	case 202: //使车子成66度倾斜，为横向移动找三分球做准备
		ROS_INFO_STREAM("case 202 202 202 202 202 202 202 202 202 202 202 202 202 202 202 202 202 202 202 \n");
		/*ROS_INFO_STREAM("正在旋转！！！\n");
		if(ToAngleByMPU(66))
		{
			ROS_INFO_STREAM("转到66度了！！！\n");
			if(stop())
			{
				ROS_INFO_STREAM("停稳！！！\n");
				temp_normal_time=normal_time;
				intention_num=4;
			}							
		}*/
		if (1 == place_num)
		{
			ROS_INFO_STREAM("角度调整过程ing................\n");
			if (fabs(dsp_status.fAngle - 60.00) > 2)
			{
				ROS_INFO_STREAM("和60度相差大于2度................\n");
				if (dsp_status.fAngle > 62)
				{
					ROS_INFO_STREAM("角度为正，需要左旋................\n");
					ToFindObjectByturnleft();
				}
				if (dsp_status.fAngle < 58)
				{
					ROS_INFO_STREAM("角度为负，需要右旋................\n");
					ToFindObjectByturnright();
				}
			}
			else
			{
				if (stop())
				{
					ROS_INFO_STREAM("角度为60度，不要动了！！！................\n");
					intention_num = 5;
				}
			}
		}
		if (2 == place_num)
		{
			ROS_INFO_STREAM("角度调整过程ing................\n");
			if (fabs(dsp_status.fAngle - (-64.00)) > 2)
			{
				ROS_INFO_STREAM("和-66度相差大于2度................\n");
				if (dsp_status.fAngle > -62)
				{
					ROS_INFO_STREAM("角度为正，需要左旋................\n");
					ToFindObjectByturnleft();
				}
				if (dsp_status.fAngle < -66)
				{
					ROS_INFO_STREAM("角度为负，需要右旋................\n");
					ToFindObjectByturnright();
				}
			}
			else
			{
				if (stop())
				{
					ROS_INFO_STREAM("角度为-64度，不要动了！！！................\n");
					intention_num = 5;
				}
			}
		}
		break;

		/************************** 三分线找球 *************************/
	case 5:
		ROS_INFO_STREAM("case  5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 \n");
		if ((normal_time - temp_normal_time) > 5)
		{
			//TODO:接收发送数据
			/*
			if(!Vision.create_communication_RAM_2())
			{
				ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
				if(!Vision.open_file_mapping_2())
				{
					ROS_INFO_STREAM("fail to open_file_mapping_2\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to open_file_mapping_2\n");
					if(!Vision.create_communication_RAM_4())
					{
						ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
						if(!Vision.open_file_mapping_4())
						{
							ROS_INFO_STREAM("fail to open_file_mapping_4\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to open_file_mapping_4\n");
							TheObject.whatObject =(objectType)8;//找什么东西在这里改
							if(!Vision.send_find_whatobject(TheObject.whatObject))
							{
								ROS_INFO_STREAM("Fail to send command\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to send command\n");
								TheObject.whatObject = (objectType)0;//变量重新初始化，为下一次找球做准备
								if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
								{
									ROS_INFO_STREAM("Fail to receive final object result\n");
								}
								else
								{
									ROS_INFO_STREAM("Success to receive final object result\n");
									ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
									ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
								}
							}
						}
					}
				}
			}
			*/

			if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
			{
				findballflag = 0;
				aimballflag = 0;
				if (1 == place_num)
				{
					if (abs((dsp_status.XDist) - (ptCC.x)) < 50)
					{
						Stop();
						ROS_INFO_STREAM("已经到边界了，但啥也没找着，算了，去找中线区的球！！！\n");
						temp_normal_time = normal_time;
						intention_num = 18;
					}
					else
					{
						ROS_INFO_STREAM("以66度方向行进...\n");
						robotstraightrightlow(); //继续右走
					}
				}
				if (2 == place_num)
				{
					if (abs((dsp_status.XDist) - (ptCC.x)) < 50)
					{
						Stop();
						ROS_INFO_STREAM("已经到边界了，但啥也没找着，算了，去找中线区的球！！！\n");
						temp_normal_time = normal_time;
						intention_num = 18; //去找中线区的球！！
					}
					else
					{
						ROS_INFO_STREAM("以-66度方向行进...\n");
						robotstraightleftlow(); //继续左走
					}
				}
			}
			else
			{
				findballflag = 1;
				ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
			}
			if ((findballflag == 1) && (aimballflag == 0))
			{
				ROS_INFO_STREAM(" cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
				if (ToAimBallByVision())
				{
					ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //对准球的过程中突然丢了
						findballflag = 0;
						aimballflag = 0;
						ROS_INFO("丢了 丢了 丢了 ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 5;
						}
					}
					else
					{
						aimballflag = 1;
						ROS_INFO("!!!!!!!!三分线球:objectAngle aimedaimedaimed is %lf;三分线球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					}
				}
			}
			if ((findballflag == 1) && (aimballflag == 1))
			{
				ROS_INFO_STREAM(" ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n");
				if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
				{
					ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //前往球的跟前过程中，又没看到球，重新去找
						aimballflag = 0;
						findballflag = 0;
						ROS_INFO_STREAM(" 又丢了 又丢了 又丢了 ！！！\n");
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 5;
						}
					}
					else
					{
						aimballflag = 0;
						findballflag = 0;
						intention_num = 6;
						ROS_INFO_STREAM("捡球 捡球 捡球 ！！！\n");
					}
				}
			}
		}
		break;

	case 6:
		ROS_INFO_STREAM("case 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 \n");
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令
		ROS_INFO_STREAM("case uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuppppppppppppppppppppppppppppppppppppp\n");
		temp_normal_time = normal_time;
		intention_num = 7; //抬机械臂命令已发送
		break;

	case 7: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 7 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 40)
		{
			ROS_INFO_STREAM("机械臂抬起，等待延时...\n");
			temp_normal_time = normal_time;
			intention_num = 8;
		}
		break;

	case 8:
		ROS_INFO_STREAM("case 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 \n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中发第一次判断命令
		{
			//m_bSend=1;    //TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			ROS_INFO_STREAM("第一次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 9;
		}
		break;

	case 9:
		ROS_INFO_STREAM("case 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 9 \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			temp_normal_time = normal_time;
			intention_num = 10;
			ROS_INFO_STREAM("发现已经捡到球了，架子上有球！！！\n");
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			temp_normal_time = normal_time;
			intention_num = 100;
			ROS_INFO_STREAM("判断没有球啊！等待抬臂稳定后在判断一遍！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 10:
		ROS_INFO_STREAM("case 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 \n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 11;
		}
		break;

	case 100:
		ROS_INFO_STREAM("100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 \n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			//m_bSend=1;    //TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("第二次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 101;
		}
		break;

	case 101:
		ROS_INFO_STREAM("case 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 101 \n");
		Control.SendData_state = 0; //单片机位置0
		ROS_INFO_STREAM("第二次，判断架子上有没有球！！！！\n");
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("发现有球啊！！！第一次误判！！！\n");
			temp_normal_time = normal_time;
			intention_num = 11;
		}
		else //判断两次都没球，可以再去找了
		{
			findballflag = 0;
			aimballflag = 0;
			intention_num = 5;
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 11: //准备走到投球点
		ROS_INFO_STREAM("case 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 \n");
		ROS_INFO_STREAM("正在对准投球定位点！！！！\n");
		if (ToAngleByMPU6050(ptD))
		{
			ROS_INFO_STREAM("已经对准了投球定位点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停到了投球定位点，再旋转调整！！！！\n");
				ptN.x = dsp_status.XDist; //把当前坐标赋给点N
				ptN.y = dsp_status.YDist;
				ROS_INFO("点N的坐标为：%ld , %ld\n", ptN.x, ptN.y);

				temp_normal_time = normal_time;
				intention_num = 12;
			}
		}
		break;

	case 12:
		ROS_INFO_STREAM("case 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 \n");
		temp_normal_time = normal_time;
		if (ToPostureByMilemeter_Pid3(ptD)) //到达D点
		{
			Stop();
			ROS_INFO_STREAM("到 DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD\n");
			intention_num = 13;
		}
		break;

	case 13: //坐标对准投球目标点
		ROS_INFO_STREAM("case 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 \n");
		ROS_INFO_STREAM("正在坐标对准投球目标点！！！！\n");
		if (ToAngleByMPU6050(ptC))
		{
			ROS_INFO_STREAM("已经坐标对准了投球目标点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("坐标对准之后已经停下，视觉对准！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 14;
			}
		}
		break;

		/***************************** 第一次找标定柱 ******************************/
	case 14:
		ROS_INFO_STREAM("case 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 \n ");
		//TODO:发送数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
						TheObject.whatObject = (objectType)1;	//找什么东西在这里改
						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to send command\n");
							TheObject.whatObject = (objectType)0;//变量初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");	
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}
		*/
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				if (normal_time - temp_normal_time < 100)
				{
					ToFindObjectByturnright();
				}
				else
				{
					if (stop())
						ToFindObjectByturnleft();
				}
			}
			if (2 == place_num)
			{
				if (normal_time - temp_normal_time < 100)
				{
					ToFindObjectByturnleft();
				}
				else
				{
					if (stop())
						ToFindObjectByturnright();
				}
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((00.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 14;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!标注杆:objectAngle aimedaimedaimed is %lf;标注杆:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					stop();
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("!!!!!!!!标定柱:The distance has been to the min error;The distance has been to the min error!!!!!!!!\n");
					temp_normal_time = normal_time;
					intention_num = 15;
				}
			}
		}
		break;

		/*case 140:
		if(ToAngleByMPU(dsp_status.fAngle-3))
		{
			ROS_INFO_STREAM("已经对准了N点！！！！\n");
			if(stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，下一步要往前走！！！！\n");
				temp_normal_time=normal_time;
				intention_num=15;
			}							
		}		
		break;*/

	case 15: //准备发射排球
		ROS_INFO_STREAM("case 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 \n");
		if ((normal_time - temp_normal_time) > 22)
		{
			Control.SendData_state = 3; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("发射 发射 发射 ！！！！\n");

			temp_normal_time = normal_time;
			intention_num = 16;
		}
		break;

	case 16: //准备返回N点
		ROS_INFO_STREAM("case 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16  \n");
		ROS_INFO_STREAM("正在对准N点！！！！\n");
		Control.SendData_state = 0;
		if (ToAngleByMPU6050(ptN))
		{
			ROS_INFO_STREAM("已经对准了N点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，下一步要往前走！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 17;
			}
		}
		break;

	case 17:
		ROS_INFO_STREAM("case 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17  \n");
		temp_normal_time = normal_time;
		if (ToPostureByMilemeter_Pid3(ptN)) //到达N点
		{
			Stop();
			ROS_INFO_STREAM("到 NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN\n");
			intention_num = 18;
		}
		break;

	case 18: //准备到达M点开始扫描中线置球
		ROS_INFO_STREAM("case 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18  \n");
		ROS_INFO_STREAM("正在对准M点！！！！\n");
		if (ToAngleByMPU6050(ptM))
		{
			ROS_INFO_STREAM("已经对准了M点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，下一步要往前走！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 19;
			}
		}
		break;

	case 19:
		ROS_INFO_STREAM("case 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 \n");
		temp_normal_time = normal_time;
		if (ToPostureByMilemeter_Pid5(ptM)) //到达M点
		{
			Stop();
			ROS_INFO_STREAM("到 MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM\n");
			intention_num = 190;
		}
		break;

	case 190: //使车子成0度，为横向移动找中线球做准备
		ROS_INFO_STREAM("case 190 190 190 190 190 190 190 190 190 190 190 190 190 190 190 190 190 190 \n");
		if (1 == place_num)
		{
			if (ToAngleByMPU(0))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 20;
				}
			}
		}
		if (2 == place_num)
		{
			if (ToAngleByMPU(-4))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 20;
				}
			}
		}
		break;

		/***********************************  中线识球开始  ***********************************/
	case 20:
		ROS_INFO_STREAM("case 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 \n");
		temp_normal_time = normal_time; //为延时作准备
		//TODO:发送接受数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");

						//视觉修改版找球测试start
						TheObject.whatObject =(objectType)8;//找什么东西在这里改
						if(TheObject.whatObject==(objectType)1)
							ROS_INFO_STREAM("send command to find calibration\n");
						if(TheObject.whatObject==(objectType)2)
							ROS_INFO_STREAM("send command to find reddish_brown_basketball\n");
						if(TheObject.whatObject==(objectType)3)
							ROS_INFO_STREAM("send command to find basketball\n");
						if(TheObject.whatObject==(objectType)4)
							ROS_INFO_STREAM("send command to find blue_gray_basketball\n");
						if(TheObject.whatObject==(objectType)5)
							ROS_INFO_STREAM("send command to find calibration_plus\n");
						if(TheObject.whatObject==(objectType)6)
							ROS_INFO_STREAM("send command to find orange_red_volleyball\n");
						if(TheObject.whatObject==(objectType)7)
							ROS_INFO_STREAM("send command to find red_blue_volleyball\n");
						if(TheObject.whatObject==(objectType)8)
							ROS_INFO_STREAM("send command to find volleyball\n");

						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM(" Success to send command Success to send command Success to send command Success to send command\n");
							TheObject.whatObject = (objectType)0;//变量重新初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}*/

		if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				if (abs((dsp_status.YDist) - (ptDD.y)) < 50)
				{
					Stop();
					ROS_INFO_STREAM("已经到边界了，但啥也没找着，停车，结束比赛！！！\n");
					temp_normal_time = normal_time;
					intention_num = 33; //停车，结束比赛 停车，结束比赛停车!!!!!!!!!!!!!!
				}
				else
				{
					ROS_INFO_STREAM("左移找球...\n");
					robotstraightleftlow(); //左移找球
				}
			}
			if (2 == place_num)
			{
				if (abs((dsp_status.YDist) - (ptDD.y)) < 50)
				{
					Stop();
					ROS_INFO_STREAM("已经到边界了，但啥也没找着，停车，结束比赛！！！\n");
					temp_normal_time = normal_time;
					intention_num = 33; //停车，结束比赛 停车，结束比赛停车!!!!!!!!!!!!!!
				}
				else
				{
					ROS_INFO_STREAM("右移找球...\n");
					robotstraightrightlow(); //右移找球
				}
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM(" findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					ROS_INFO("丢了 丢了 丢了 : %lf \n", objectAngleDistance_Y.first);
					if ((normal_time - temp_normal_time) > 2)
					{
						temp_normal_time = normal_time; //为延时作准备
						intention_num = 20;
					}
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!中圈球:objectAngle aimedaimedaimed is %lf;中圈球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1))
		{
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("又丢了 又丢了 又丢了 \n");
					if ((normal_time - temp_normal_time) > 2)
					{
						temp_normal_time = normal_time; //为延时作准备
						intention_num = 20;
					}
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 21;
					ROS_INFO_STREAM("捡球 捡球 捡球 \n");
				}
			}
		}
		break;

	case 21:
		ROS_INFO_STREAM("case 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 \n");
		//立即捡球，不要延时
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		ROS_INFO_STREAM("发送抬臂指令！！！");
		temp_normal_time = normal_time;
		intention_num = 22; //抬机械臂命令已发送
		break;

	case 22: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 40)
		{
			temp_normal_time = normal_time;
			intention_num = 23;
		}
		break;

	case 23:
		ROS_INFO_STREAM("case 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 \n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中就发第一次判断命令
		{
			//m_bSend=1;    //TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			ROS_INFO_STREAM("第一次，发送捡球判断！！！");
			temp_normal_time = normal_time;
			intention_num = 24;
		}
		break;

	case 24:
		ROS_INFO_STREAM("case 24 24 24 24 24 24 24 24 24 24 24 24 24 \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			ROS_INFO_STREAM("检测到有球！！！");
			temp_normal_time = normal_time;
			intention_num = 25;
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			ROS_INFO_STREAM("没检测到有球，在判断一次！！！");
			temp_normal_time = normal_time;
			intention_num = 250;
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 25:
		ROS_INFO_STREAM("case 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 \n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 252;
		}
		break;

	case 250:
		ROS_INFO_STREAM("case 250 250 250 250 250 250 250 250 250 250 250 250  \n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			//m_bSend=1;    //TODO:允许给单片机发送命令
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("第二次，发送捡球判断！！！");
			temp_normal_time = normal_time;
			intention_num = 251;
		}
		break;

	case 251:
		ROS_INFO_STREAM("case 251 251 251 251 251 251 251 251 251 251 251 251 251 251  \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("第一次为误判，明明有球！！！");
			temp_normal_time = normal_time;
			intention_num = 252;
		}
		else //判断两次都没球，可以再去找了
		{
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！");
			findballflag = 0;
			aimballflag = 0;
			intention_num = 20;
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 252:
		ROS_INFO_STREAM("case 252 251 251 251 251 251 251 251 251 251 251 251 251 251  \n");
		if (ToPostureByMilemeter_Pid2(ptM))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 26;
		}
		break;

	case 26: //准备返回B1点
		ROS_INFO_STREAM("case 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 \n");
		ROS_INFO_STREAM("正在对准N点！！！！\n");
		Control.SendData_state = 0;
		if (ToAngleByMPU6050(ptN))
		{
			ROS_INFO_STREAM("已经对准了N点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，下一步要往前走！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 27;
			}
		}
		break;

	case 27:
		ROS_INFO_STREAM("case 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 \n");
		temp_normal_time = normal_time;
		if (ToPostureByMilemeter_Pid3(ptN)) //到达N点
		{
			Stop();
			ROS_INFO_STREAM("到 NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN\n");
			intention_num = 28;
		}
		break;

	case 28: //准备到达投球点D
		ROS_INFO_STREAM("case 28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  28  \n");
		ROS_INFO_STREAM("正在对准D点！！！！\n");
		Control.SendData_state = 0;
		if (ToAngleByMPU6050(ptD2))
		{
			ROS_INFO_STREAM("已经对准了D2点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下，下一步要往前走！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 29;
			}
		}
		break;

	case 29:
		ROS_INFO_STREAM("case 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 \n");
		temp_normal_time = normal_time;
		if (ToPostureByMilemeter_Pid3(ptD2)) //到达D点
		{
			Stop();
			ROS_INFO_STREAM("到 DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD\n");
			intention_num = 30;
		}
		break;

	case 30: //坐标对准投球目标点
		ROS_INFO_STREAM("case 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 \n");
		ROS_INFO_STREAM("正在坐标对准投球目标点！！！！\n");
		if (ToAngleByMPU6050(ptC))
		{
			ROS_INFO_STREAM("已经坐标对准了投球目标点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("坐标对准之后已经停下，视觉对准！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 31;
			}
		}
		break;

		/********************************* 第二次识别标定柱 *********************************/
	case 31:
		ROS_INFO_STREAM("case 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 31 \n ");
		//TODO:发送接收数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
						TheObject.whatObject = (objectType)1;	//找什么东西在这里改
						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to send command\n");
							TheObject.whatObject = (objectType)0;//变量初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");	
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}
		*/

		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				if (normal_time - temp_normal_time < 100)
				{
					ToFindObjectByturnright();
				}
				else
				{
					if (stop())
						ToFindObjectByturnleft();
				}
			}
			if (2 == place_num)
			{
				if (normal_time - temp_normal_time < 100)
				{
					ToFindObjectByturnleft();
				}
				else
				{
					if (stop())
						ToFindObjectByturnright();
				}
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((00.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 31;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!标注杆:objectAngle aimedaimedaimed is %lf;标注杆:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					stop();
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("!!!!!!!!标定柱:The distance has been to the min error;The distance has been to the min error!!!!!!!!\n");
					temp_normal_time = normal_time;
					intention_num = 32;
				}
			}
		}
		break;

	case 32: //准备发射排球
		ROS_INFO_STREAM("case 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32  \n");
		if ((normal_time - temp_normal_time) > 22)
		{
			//m_bSend=1; TODO:
			Control.SendData_state = 3; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("发射 发射 发射 ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 33;
		}
		break;

	case 33: //停车，比赛结束！！！！！！！！！！！
		ROS_INFO_STREAM("case 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33  \n");
		Control.SendData_state = 0;
		Stop(); //停车
		break;

	default:
		normal_time = 0;
		break;
	}
}

void DecisionMaking::Normal_bat3() //**********投篮-回合3**********
{
	//一些决策相关的标志
	static int intention_num = 0; //回合过程的意图顺序号
	static bool findballflag = 0; //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
	static bool aimballflag = 0;
	static bool Isthreescoreunnormal = 0; //捡三分球是否出现异常
	static bool border3 = 0;

	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;

	Posture ptA0, ptA, ptC, ptD, ptE, ptCC, ptD1;
	static Posture ptN = {0, 0};

	ptA0.x = 1000;
	ptA0.y = 0;

	if (1 == place_num)
	{
		ptA.x = 3500; //中点，扫描起点
		ptA.y = 4700;

		ptC.x = 4250; //投篮标定柱位置
		ptC.y = 10925;

		ptD.x = 3200; //投篮正中定位点（激光距离篮筐2400）（钢板400计，激光超出30计）
		ptD.y = 8100;

		ptE.x = 1890; //投篮45偏角位置（预留出车身宽度和激光距离，分别400和30计）（2630处）
		ptE.y = 9065;

		ptCC.x = 1100; //投篮标定柱位置
		ptCC.y = 7500;

		ptD1.x = 3200; //投篮正中定位点（激光距离篮筐2400）（钢板400计，激光超出30计）
		ptD1.y = 8100;

		//ptE.x=2053;			//投篮45偏角位置（2400处）
		//ptE.y=9228;
	}
	if (2 == place_num)
	{
		ptA.x = 3600; //中点，扫描起点
		ptA.y = -4800;

		ptC.x = 4250; //投篮标定柱位置
		ptC.y = -10925;

		ptD.x = 3200; //投篮正中定位点（激光距离篮筐2400）（钢板400计，激光超出30计）
		ptD.y = -8100;

		ptE.x = 1890; //投篮45偏角位置（预留出车身宽度和激光距离，分别400和30计）（2630处）
		ptE.y = -9065;

		ptCC.x = 1100; //投篮标定柱位置
		ptCC.y = -7500;

		ptD1.x = 3200; //投篮正中定位点（激光距离篮筐2400）（钢板400计，激光超出30计）
		ptD1.y = -8100;
	}

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 000000000000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;
	case 120:
		if ((normal_time - temp_normal_time) > 200) //延时9s
		{
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 111111111111111111111111111111111111111111111111111111111111111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA0))
		{
			Stop();
			ROS_INFO_STREAM("已经到了A0点了 ！！！\n");
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 222222222222222222222222222222222222222222222222222222222222222\n");
		if (ToAngleByMPU6050(ptA))
		{
			if (stop())
			{
				ROS_INFO_STREAM("已经对准A点了 ！！！\n");
				temp_normal_time = normal_time;
				intention_num = 201;
			}
		}
		break;

	case 201:
		ROS_INFO_STREAM("case 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 \n");
		if (ToPostureByMilemeter_Pid3(ptA))
		{
			if (stop())
			{
				ROS_INFO_STREAM("已经第一次到了A点了 ！！！\n");
				temp_normal_time = normal_time;
				intention_num = 202;
			}
		}
		break;

	case 202:
		ROS_INFO_STREAM("case 202 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 201 \n");
		if (ToPostureByMilemeter_Pid6(ptA))
		{
			if (stop())
			{
				ROS_INFO_STREAM("已经再次到了A点了 ！！！\n");
				temp_normal_time = normal_time;
				intention_num = 3;
			}
		}

		break;
		/********************* 运动过程中避车 ************************/

		/********************* 运动避车结束 ************************/

	case 3: //使车子成66度倾斜，为横向移动找三分球做准备
		ROS_INFO_STREAM("case 3333333333333333333333333333333333333333333333333333333333333333333333333\n");
		if (1 == place_num)
		{
			ROS_INFO_STREAM("角度调整过程ing................\n");
			if (fabs(dsp_status.fAngle - 64.00) > 2)
			{
				ROS_INFO_STREAM("和66度相差大于2度................\n");
				if (dsp_status.fAngle > 66)
				{
					ROS_INFO_STREAM("角度为正，需要左旋................\n");
					ToFindObjectByturnleft();
				}
				if (dsp_status.fAngle < 62)
				{
					ROS_INFO_STREAM("角度为负，需要右旋................\n");
					ToFindObjectByturnright();
				}
			}
			else
			{
				if (stop())
				{
					ROS_INFO_STREAM("角度为60度，不要动了！！！................\n");
					temp_normal_time = normal_time;
					intention_num = 4;
				}
			}
		}
		if (2 == place_num)
		{
			ROS_INFO_STREAM("角度调整过程ing................\n");
			if (fabs(dsp_status.fAngle - (-63.00)) > 2)
			{
				ROS_INFO_STREAM("和-63度相差大于2度................\n");
				if (dsp_status.fAngle > -61)
				{
					ROS_INFO_STREAM("角度为正，需要左旋................\n");
					ToFindObjectByturnleft();
				}
				if (dsp_status.fAngle < -65)
				{
					ROS_INFO_STREAM("角度为负，需要右旋................\n");
					ToFindObjectByturnright();
				}
			}
			else
			{
				if (stop())
				{
					ROS_INFO_STREAM("角度为-66度，不要动了！！！................\n");
					temp_normal_time = normal_time;
					intention_num = 4;
				}
			}
		}
		break;

		/***************************** 第一次识别球 *******************************/
	case 4:
		ROS_INFO_STREAM("case 44444444444444444444444444444444444444444444444444444444444444444444444444444\n");
		if ((normal_time - temp_normal_time) > 11)
		{
			//TODO:发送接收数据
			/*
			if(!Vision.create_communication_RAM_2())
			{
				ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
				if(!Vision.open_file_mapping_2())
				{
					ROS_INFO_STREAM("fail to open_file_mapping_2\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to open_file_mapping_2\n");
					if(!Vision.create_communication_RAM_4())
					{
						ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
						if(!Vision.open_file_mapping_4())
						{
							ROS_INFO_STREAM("fail to open_file_mapping_4\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to open_file_mapping_4\n");
							TheObject.whatObject =(objectType)8;//找什么东西在这里改
							if(!Vision.send_find_whatobject(TheObject.whatObject))
							{
								ROS_INFO_STREAM("Fail to send command\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to send command\n");
								TheObject.whatObject = (objectType)0;//变量重新初始化，为下一次找球做准备
								if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
								{
									ROS_INFO_STREAM("Fail to receive final object result\n");
								}
								else
								{
									ROS_INFO_STREAM("Success to receive final object result\n");
									ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
									ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
								}
							}
						}
					}
				}
			}
			*/

			if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || ((objectAngleDistance_Y.second) > 1000))
			{
				findballflag = 0;
				aimballflag = 0;
				if (1 == place_num)
				{
					if ((dsp_status.XDist) - (ptCC.x) > 50)
					{
						ROS_INFO_STREAM("以66度方向行进...\n");
						robotstraightright(); //继续右走
					}
					else
					{
						ROS_INFO_STREAM("到边界了，没找到！！！................\n");
						Stop();
						if (ToAngleByMPU(90))
						{
							ROS_INFO_STREAM("边界右转结束 ！！！................\n");
							if (normal_time - temp_normal_time < 120)
							{
								ROS_INFO_STREAM("边界前进 ！！！................\n");
								robotforwardtoolow();
							}
							else
							{
								ROS_INFO_STREAM("边界前进，停止 ！！！................\n");
								Stop();
								temp_normal_time = normal_time;
								intention_num = 15;
							}
						}
					}
				}
				if (2 == place_num)
				{
					if ((dsp_status.XDist) - (ptCC.x) > 50)
					{
						ROS_INFO_STREAM("以66度方向行进...\n");
						robotstraightleft(); //继续右走
						temp_normal_time = normal_time;
					}
					else
					{
						ROS_INFO_STREAM("到边界了，没找到！！！................\n");
						Stop();
						if (ToAngleByMPU(-90))
						{
							ROS_INFO_STREAM("边界右转结束 ！！！................\n");
							if (normal_time - temp_normal_time < 120)
							{
								ROS_INFO_STREAM("边界前进 ！！！................\n");
								robotforwardtoolow();
							}
							else
							{
								ROS_INFO_STREAM("边界前进，停止 ！！！................\n");
								Stop();
								temp_normal_time = normal_time;
								intention_num = 15;
							}
						}
					}
				}
			}
			else
			{
				findballflag = 1;
				Isthreescoreunnormal = 0;
				ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
			}
			if ((findballflag == 1) && (aimballflag == 0))
			{
				ROS_INFO_STREAM(" cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
				if (ToAimBallByVision())
				{
					ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //对准球的过程中突然丢了
						findballflag = 0;
						aimballflag = 0;
						ROS_INFO("丢了 丢了 丢了 ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 4;
						}
					}
					else
					{
						aimballflag = 1;
						ROS_INFO("!!!!!!!!三分线球:objectAngle aimedaimedaimed is %lf;三分线球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					}
				}
			}
			if ((findballflag == 1) && (aimballflag == 1))
			{
				ROS_INFO_STREAM(" ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n");
				if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
				{
					ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //前往球的跟前过程中，又没看到球，重新去找
						aimballflag = 0;
						findballflag = 0;
						ROS_INFO_STREAM(" 又丢了 又丢了 又丢了 ！！！\n");
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 4;
						}
					}
					else
					{
						aimballflag = 0;
						findballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 5;
						ROS_INFO_STREAM("捡球 捡球 捡球 ！！！\n");
					}
				}
			}
		}
		break;

	case 5:
		ROS_INFO_STREAM("case 55555555555555555555555555555555555555555555555555555555555555555555555555555\n");
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令
		ROS_INFO_STREAM("case uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuppppppppppppppppppppppppppppppppppppp\n");
		temp_normal_time = normal_time;
		intention_num = 6; //抬机械臂命令已发送
		break;

	case 6: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 66666666666666666666666666666666666666666666666666666666666666666666666666666\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 40)
		{
			ROS_INFO_STREAM("机械臂抬起，等待延时...\n");
			temp_normal_time = normal_time;
			intention_num = 7;
		}
		break;

	case 7:
		ROS_INFO_STREAM("case 77777777777777777777777777777777777777777777777777777777777777777777777777777\n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中发第一次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			ROS_INFO_STREAM("第一次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 8;
		}
		break;

	case 8:
		ROS_INFO_STREAM("case 88888888888888888888888888888888888888888888888888888888888888888888888888888\n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			temp_normal_time = normal_time;
			intention_num = 80;
			ROS_INFO_STREAM("发现已经捡到球了，架子上有球！！！\n");
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			temp_normal_time = normal_time;
			intention_num = 81;
			ROS_INFO_STREAM("判断没有球啊！等待抬臂稳定后在判断一遍！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 80:
		ROS_INFO_STREAM("case 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80\n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 9;
		}
		break;

	case 81:
		ROS_INFO_STREAM("case 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81\n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("第二次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 82;
		}
		break;

	case 82:
		ROS_INFO_STREAM("case 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82\n");
		Control.SendData_state = 0; //单片机位置0
		ROS_INFO_STREAM("第二次，判断架子上没有球！！！！\n");
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("发现有球啊！！！第一次误判！！！\n");
			temp_normal_time = normal_time;
			intention_num = 9;
		}
		else //判断两次都没球，可以再去找了
		{
			findballflag = 0;
			aimballflag = 0;
			temp_normal_time = normal_time;
			intention_num = 4;
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 9: //转向目的点，去找标定柱
		ROS_INFO_STREAM("case 99999999999999999999999999999999999999999999999999999999999999999999999\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (ToAngleByMPU6050(ptC))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 90;
				}
			}
		}
		break;

	case 90:
		ROS_INFO_STREAM("case 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 \n");
		ROS_INFO_STREAM("正在对准传球目标点,准备避障行进！！！！\n");
		if (ToPostureByAvoidance(ptC))
		{
			ROS_INFO_STREAM("已经对准了投篮正中目标点，开始避障行进 ！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("ToPostureByAvoidance(ptM) ToPostureByAvoidance(ptM) ToPostureByAvoidance(ptM)\n");
				temp_normal_time = normal_time;
				intention_num = 91;
			}
		}
		if ((normal_time - temp_normal_time) > 180)
		{
			Stop();
			ROS_INFO_STREAM("到了100秒 ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 10;
		}
		break;

	case 91:
		ROS_INFO_STREAM("case 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 \n");
		if ((normal_time - temp_normal_time) > 2) //以开环速度向前1s
		{
			if (1 == place_num)
			{
				if (ToAngleByMPU(90))
				{
					Stop();
					temp_normal_time = normal_time;
					intention_num = 92;
				}
			}
			if (2 == place_num)
			{
				if (ToAngleByMPU(-90))
				{
					Stop();
					temp_normal_time = normal_time;
					intention_num = 92;
				}
			}
		}
		break;

	case 92:
		ROS_INFO_STREAM("case 92 92 92 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 91 \n");
		if ((normal_time - temp_normal_time) < 34) //以开环速度向前1s
		{
			robotforward();
		}
		else
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 93;
		}
		break;

	case 93: //避障后再次对准目标点
		ROS_INFO_STREAM("case 93 93 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 \n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (ToPostureByMilemeter_Pid6(ptD))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 10;
				}
			}
		}
		break;

	case 10:
		ROS_INFO_STREAM("case 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 10 \n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (ToAngleByMPU6050(ptD)) //对准投篮定位点
			{
				ROS_INFO_STREAM("已经对准了D ！！！\n");
				if (stop())
				{
					ROS_INFO_STREAM("停下 ！！！\n");
					temp_normal_time = normal_time;
					intention_num = 11;
				}
			}
		}
		break;

	case 11:
		ROS_INFO_STREAM("case 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 11 \n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (ToPostureByMilemeter_Pid2(ptD))
			{
				Stop();
				ROS_INFO_STREAM("已经到了D点 ！！！n");
				temp_normal_time = normal_time;
				intention_num = 12;
			}
		}
		break;

	case 12:
		ROS_INFO_STREAM("case 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 12 \n");
		if (ToAngleByMPU6050(ptC)) //对准投篮标定柱
		{
			ROS_INFO_STREAM("已经对准了C ！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("停下 ！！！\n");
				temp_normal_time = normal_time;
				intention_num = 13;
			}
		}
		break;

		/********************************* 第一次识别标定柱 *********************************/
	case 13:
		ROS_INFO_STREAM("case 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 13 \n ");
		//TODO:发送接收数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
						TheObject.whatObject = (objectType)1;	//找什么东西在这里改
						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to send command\n");
							TheObject.whatObject = (objectType)0;//变量初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");	
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}
		*/

		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				if ((normal_time - temp_normal_time) < 100)
				{
					ToFindObjectByturnright();
				}
				else if ((normal_time - temp_normal_time) < 122)
				{
					Stop();
				}
				else
					ToFindObjectByturnleft();
			}
			if (2 == place_num)
			{
				if ((normal_time - temp_normal_time) < 100)
				{
					ToFindObjectByturnleft();
				}
				else if ((normal_time - temp_normal_time) < 122)
				{
					Stop();
				}
				else
					ToFindObjectByturnright();
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag \n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((00.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 13;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!标注杆:objectAngle aimedaimedaimed is %lf;标注杆:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 40)) && (aimballflag == 1))
				{
					stop();
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("!!!!!!!!标定柱:The distance has been to the min error;The distance has been to the min error!!!!!!!!\n");
					temp_normal_time = normal_time;
					intention_num = 14;
				}
			}
		}
		break;

	case 14: //发射
		ROS_INFO_STREAM("case 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 14 \n");
		if (normal_time - temp_normal_time > 15)
		{
			Stop();
			Control.SendData_state = 3; //发送发射指令，下一步要立即置位0
			ROS_INFO_STREAM("发射 ！！！\n");
			temp_normal_time = normal_time;
			intention_num = 15;
		}
		break;

	case 15:
		ROS_INFO_STREAM("case 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 \n");
		Control.SendData_state = 0;
		if (normal_time - temp_normal_time > 11)
		{
			if (1 == place_num)
			{
				if (ToAngleByMPU(-94))
				{
					Stop();
					temp_normal_time = normal_time;
					intention_num = 150;
					/*robotstraightleftlow();
					ROS_INFO_STREAM("横向左移找球..\n");*/
				}
			}
			if (2 == place_num)
			{
				if (ToAngleByMPU(90))
				{
					Stop();
					temp_normal_time = normal_time;
					intention_num = 150;
					/*robotstraightrightlow();
					ROS_INFO_STREAM("横向右移找球..\n");*/
				}
			}
		}
		break;

	case 150:
		ROS_INFO_STREAM("case 150 150 150 150 150 150 150 150 150 150 150 150 150 150 150 150 150 150 150 150 \n");
		if (normal_time - temp_normal_time > 5)
		{
			if (1 == place_num)
			{
				if ((dsp_status.YDist - (8400)) < -50)
				{
					ROS_INFO_STREAM("后退！！！\n");
					robotbacktoolow();
				}
				else if ((dsp_status.YDist - (8400)) > 50)
				{
					robotforwardtoolow();
				}
				else
				{
					Stop();
					temp_normal_time = normal_time;
					intention_num = 16;
				}
			}
			if (2 == place_num)
			{
				if ((dsp_status.YDist - (-8400)) > 50)
				{
					ROS_INFO_STREAM("后退！！！\n");
					robotbacktoolow();
				}
				else if (((dsp_status.YDist - (-8400)) < -50))
				{
					robotforwardtoolow();
				}
				else
				{
					Stop();
					temp_normal_time = normal_time;
					intention_num = 16;
				}
				/*if(ToBallByVision_new())
				{
					Stop();
					temp_normal_time=normal_time;
					intention_num=16;
				}*/
			}
		}
		break;

		/************************************ 第二次找球 ***********************************/
	case 16:
		ROS_INFO_STREAM("case 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 16 \n");
		if (normal_time - temp_normal_time > 5)
		{
			//TODO:发送接收数据
			/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");

		//视觉修改版找球测试start
						TheObject.whatObject =(objectType)8;//找什么东西在这里改
						if(TheObject.whatObject==(objectType)1)
							ROS_INFO_STREAM("send command to find calibration\n");
						if(TheObject.whatObject==(objectType)2)
							ROS_INFO_STREAM("send command to find reddish_brown_basketball\n");
						if(TheObject.whatObject==(objectType)3)
							ROS_INFO_STREAM("send command to find basketball\n");
						if(TheObject.whatObject==(objectType)4)
							ROS_INFO_STREAM("send command to find blue_gray_basketball\n");
						if(TheObject.whatObject==(objectType)5)
							ROS_INFO_STREAM("send command to find calibration_plus\n");
						if(TheObject.whatObject==(objectType)6)
							ROS_INFO_STREAM("send command to find orange_red_volleyball\n");
						if(TheObject.whatObject==(objectType)7)
							ROS_INFO_STREAM("send command to find red_blue_volleyball\n");
						if(TheObject.whatObject==(objectType)8)
							ROS_INFO_STREAM("send command to find volleyball\n");

						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM(" Success to send command Success to send command Success to send command Success to send command\n");
							TheObject.whatObject = (objectType)0;//变量重新初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}
		*/

			if (((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) || (objectAngleDistance_Y.second > 1000))
			{
				aimballflag = 0;
				findballflag = 0;
				if (1 == place_num)
				{
					if ((abs(dsp_status.XDist - 1000) < 50) && (0 == border3))
					{
						Stop();
						border3 = 1;
					}
					else if (1 == border3)
					{
						if (abs(dsp_status.XDist - 4200) < 100)
						{
							Stop();
							ROS_INFO_STREAM("回合结束了，投篮3第二次啥也没找着！！！\n");
						}
						else
						{
							robotstraightright();
							ROS_INFO_STREAM("横向右移找球..\n");
						}
					}
					else
					{
						robotstraightleft();
						ROS_INFO_STREAM("横向左移找球..\n");
					}
				}

				if (2 == place_num)
				{
					if ((abs(dsp_status.XDist - 1000) < 50) && (0 == border3))
					{
						Stop();
						border3 = 1;
					}
					else if (1 == border3)
					{
						if (abs(dsp_status.XDist - 4200) < 100)
						{
							Stop();
							ROS_INFO_STREAM("回合结束了，投篮3第二次啥也没找着！！！\n");
						}
						else
						{
							robotstraightleft();
							ROS_INFO_STREAM("横向左移找球..\n");
						}
					}
					else
					{
						robotstraightright();
						ROS_INFO_STREAM("横向右移找球..\n");
					}
				}
			}
			else
			{
				findballflag = 1;
				ROS_INFO_STREAM(" findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
			}
			if ((findballflag == 1) && (aimballflag == 0))
			{
				if (ToAimBallByVision())
				{
					ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //对准球的过程中突然丢了
						findballflag = 0;
						aimballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 16;
						ROS_INFO("丢了 丢了 丢了 : %lf \n", objectAngleDistance_Y.first);
					}
					else
					{
						aimballflag = 1;
						ROS_INFO("!!!!!!!!中圈球:objectAngle aimedaimedaimed is %lf;中圈球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					}
				}
			}
			if ((findballflag == 1) && (aimballflag == 1))
			{
				if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
				{
					ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //前往球的跟前过程中，又没看到球，重新去找
						aimballflag = 0;
						findballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 16;
						ROS_INFO_STREAM("又丢了 又丢了 又丢了 \n");
					}
					else
					{
						aimballflag = 0;
						findballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 17;
						ROS_INFO_STREAM("捡球 捡球 捡球 \n");
					}
				}
			}
		}
		break;

	case 17:
		ROS_INFO_STREAM("case 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 17 \n");
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令
		ROS_INFO_STREAM("case uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuppppppppppppppppppppppppppppppppppppp\n");
		temp_normal_time = normal_time;
		intention_num = 18; //抬机械臂命令已发送
		break;

	case 18: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 18 \n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 40)
		{
			ROS_INFO_STREAM("机械臂抬起，等待延时...\n");
			temp_normal_time = normal_time;
			intention_num = 19;
		}
		break;

	case 19:
		ROS_INFO_STREAM("case 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 19 \n");
		if ((normal_time - temp_normal_time) > 2) //抬臂过程中发第一次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球,下一步立即置位0
			ROS_INFO_STREAM("第一次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 20;
		}
		break;

	case 20:
		ROS_INFO_STREAM("case 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 \n");
		Control.SendData_state = 0;			 //单片机位置0
		if ('y' == dsp_status.RecData_state) //抬臂过程中就检测到有球，需要给一定延时
		{
			temp_normal_time = normal_time;
			intention_num = 21;
			ROS_INFO_STREAM("发现已经捡到球了，架子上有球！！！\n");
		}
		else //抬臂过程中没看到，准备持球机构稳定了再判断一次
		{
			temp_normal_time = normal_time;
			intention_num = 22;
			ROS_INFO_STREAM("判断没有球啊！等待抬臂稳定后在判断一遍！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 21:
		ROS_INFO_STREAM("case 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 21 \n");
		if ((normal_time - temp_normal_time) > 40) //1.4s时间让持球和球机构稳定
		{
			temp_normal_time = normal_time;
			intention_num = 24;
		}
		break;

	case 22:
		ROS_INFO_STREAM("case 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 22 \n");
		if ((normal_time - temp_normal_time) > 40) //抬臂稳定后发送第二次判断命令
		{
			Control.SendData_state = 5; //向单片机询问是否捡到球
			ROS_INFO_STREAM("第二次，标志位给5，看看有没有捡到球上来！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 23;
		}
		break;

	case 23:
		ROS_INFO_STREAM("case 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 23 \n");
		Control.SendData_state = 0; //单片机位置0
		ROS_INFO_STREAM("第二次，判断架子上有没有球！！！！\n");
		if ('y' == dsp_status.RecData_state) //直接准备对准目标发射
		{
			ROS_INFO_STREAM("发现有球啊！！！第一次误判！！！\n");
			temp_normal_time = normal_time;
			intention_num = 24;
		}
		else //判断两次都没球，可以再去找了
		{
			findballflag = 0;
			aimballflag = 0;
			temp_normal_time = normal_time;
			intention_num = 16;
			ROS_INFO_STREAM("判断两次都没球，可以再去找了！！！！\n");
		}
		ROS_INFO("dsp_status.RecData_state dsp_status.RecData_state dsp_status.RecData_state is %c\n", dsp_status.RecData_state);
		break;

	case 24:
		ROS_INFO_STREAM("case 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 24 \n");
		ROS_INFO_STREAM("正在对准投篮定位点！！！！\n");
		if (ToAngleByMPU6050(ptD1))
		{
			ROS_INFO_STREAM("已经对准了投篮定位点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停下！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 25;
			}
		}
		break;

	case 25:
		ROS_INFO_STREAM("case 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 25 \n");
		if (ToPostureByMilemeter_Pid3(ptD1))
		{
			Stop();
			ROS_INFO_STREAM("已经到了D点 ！！！n");
			temp_normal_time = normal_time;
			intention_num = 26;
		}
		break;

	case 26:
		ROS_INFO_STREAM("case 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 26 \n");
		if (ToAngleByMPU6050(ptC)) //对准投篮标定柱
		{
			ROS_INFO_STREAM("已经对准了C ！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("停下 ！！！\n");
				temp_normal_time = normal_time;
				intention_num = 27;
			}
		}
		break;

		/***************************** 第二次找标定柱 *******************************/
	case 27:
		ROS_INFO_STREAM("case 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 27 \n ");
		//TODO:发送接收数据
		/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
						TheObject.whatObject = (objectType)1;	//找什么东西在这里改
						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM("Success to send command\n");
							TheObject.whatObject = (objectType)0;//变量初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");	
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}
		*/

		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				if ((normal_time - temp_normal_time) < 100)
					ToFindObjectByturnright();
				else if (stop())
					ToFindObjectByturnleft();
			}
			if (2 == place_num)
			{
				if ((normal_time - temp_normal_time) < 100)
					ToFindObjectByturnleft();
				else if (stop())
					ToFindObjectByturnright();
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((00.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 27;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!标注杆:objectAngle aimedaimedaimed is %lf;标注杆:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90) < 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 40)) && (aimballflag == 1))
				{
					stop();
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("!!!!!!!!标定柱:The distance has been to the min error;The distance has been to the min error!!!!!!!!\n");
					temp_normal_time = normal_time;
					intention_num = 28;
				}
			}
		}
		break;

	case 28:
		ROS_INFO_STREAM("case 28 28 28 28 28 28 28 28 28 28 28 28 28 28 28 28 28 28 28 28 28 28 \n");
		if ((normal_time - temp_normal_time) > 15)
		{
			Control.SendData_state = 3; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("发射 发射 发射 ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 29;
		}
		break;

	case 29:
		ROS_INFO_STREAM("case 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 \n");
		Control.SendData_state = 0;
		Stop();
		break;

	default:
		ROS_INFO_STREAM("case： default ！！！ default ！！！ default ！！！ \n");
		break;
	}
}

void DecisionMaking::Normal_bat4()
{
	//一些决策相关的标志
	static int intention_num = 4; //回合过程的意图顺序号
	static bool findballflag = 0; //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
	static bool aimballflag = 0;

	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	static int vision_scan_time = 0; //视觉扫描次数统计，达到设定值时，车子要回归原位
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptOO, ptA, ptB, ptxy;

	ptOO.x = 1000;
	ptOO.y = 0; //回合结束前，用于回位定点

	ptxy.x = 0;
	ptxy.y = 0;

	ptA.x = 2200;
	ptA.y = 0; //车子带着篮球进入传球边界线内的点，车子在此处往目标点M发射球

	ptB.x = 6200;
	ptB.y = 0;

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 1;
		break;

	case 120:
		if ((normal_time - temp_normal_time) > 200) //延时大约9s
		{
			temp_normal_time = normal_time;
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111111111111\n");

		//	Control.SendData_state=0;
		//if(ToPostureByMilemeter(ptA)) //走到点A (ToPostureByMilemeter)ToPostureByMilemeter_Pid
		//{
		//	ROS_INFO_STREAM("我到 A 点了 我到 A 点了 我到 A 点了 ！！！X is : %ld，Y is : %ld, angle is : %lf\n",dsp_status.XDist, dsp_status.YDist,dsp_status.fAngle);
		//	Stop();
		//	temp_normal_time=normal_time;
		//	intention_num=4;
		//}
		//break;
		Control.SendData_state = 0;
		if (ToPostureByMilemeter(ptA)) //走到点A (ToPostureByMilemeter)ToPostureByMilemeter_Pid
		{
			ROS_INFO("我到 A 点了 我到 A 点了 我到 A 点了 ！！！X is : %ld，Y is : %ld, angle is : %lf\n", dsp_status.XDist, dsp_status.YDist, dsp_status.fAngle);
			Stop();
			temp_normal_time = normal_time;
			intention_num = 4;
		}
		break;

		//void CDecisionMaking::Normal_pass0()  //激光摄像头测试
	case 4:
	{
		static int intention_num = 5; //回合过程的意图顺序号
		static bool findballflag = 0; //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
		static bool aimballflag = 0;
		static bool armupflag = 0;
		static bool stopflag = 0;
		static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
		static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
		normal_time++;

		switch (intention_num)
		{
		case 5:
			ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000000000000000000000000000000000000\n");
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 22;
			break;

		case 120:
			if ((normal_time - temp_normal_time) > 45) //延时2S
			{
				/*if(ToBallByVision_new())
			{*/
				temp_normal_time = normal_time; //为延时作准备
				intention_num = 2;
				//}
			}
			break;

		case 22:
			ROS_INFO_STREAM("case 2222222222222222222222222222222222222222222222222222222222222222222222222222222\n");
			//temp_normal_time=normal_time;     //为延时作准备
			//TODO:发送接收数据
			/*
		if(!Vision.create_communication_RAM_2())
		{
			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
		}
		else
		{
			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
			if(!Vision.open_file_mapping_2())
			{
				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
			}
			else
			{
				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
				if(!Vision.create_communication_RAM_4())
				{
					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
				}
				else
				{
					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
					if(!Vision.open_file_mapping_4())
					{
						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
					}
					else
					{
						ROS_INFO_STREAM("Success to open_file_mapping_4\n");

	
						TheObject.whatObject =(objectType)4;//找什么东西在这里改
						if(TheObject.whatObject==(objectType)1)
							ROS_INFO_STREAM("send command to find calibration\n");
						if(TheObject.whatObject==(objectType)2)
							ROS_INFO_STREAM("send command to find reddish_brown_basketball\n");
						if(TheObject.whatObject==(objectType)3)
							ROS_INFO_STREAM("send command to find basketball\n");
						if(TheObject.whatObject==(objectType)4)
							ROS_INFO_STREAM("send command to find blue_gray_basketball\n");
						if(TheObject.whatObject==(objectType)5)
							ROS_INFO_STREAM("send command to find calibration_plus\n");
						if(TheObject.whatObject==(objectType)6)
							ROS_INFO_STREAM("send command to find orange_red_volleyball\n");
						if(TheObject.whatObject==(objectType)7)
							ROS_INFO_STREAM("send command to find red_blue_volleyball\n");
						if(TheObject.whatObject==(objectType)8)
							ROS_INFO_STREAM("send command to find volleyball\n");

						if(!Vision.send_find_whatobject(TheObject.whatObject))
						{
							ROS_INFO_STREAM("Fail to send command\n");
						}
						else
						{
							ROS_INFO_STREAM(" Success to send command Success to send command Success to send command Success to send command\n");
							TheObject.whatObject = (objectType)0;//变量重新初始化，为下一次找球做准备
							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
							{
								ROS_INFO_STREAM("Fail to receive final object result\n");
							}
							else
							{
								ROS_INFO_STREAM("Success to receive final object result\n");
								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
							}
						}
					}
				}
			}
		}
		*/

			if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
			{
				//ToFindObjectByturnright(); //左旋找球测试（需要测试旋转找球时请将该函数释放）
				//ROS_INFO_STREAM(" 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转\n");
				robotstraightright(); //右移找球测试
				ROS_INFO_STREAM(" 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 \n");
			}
			else
			{
				findballflag = 1;
				ROS_INFO_STREAM(" findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
			}
			if ((findballflag == 1) && (aimballflag == 0))
			{

				if (ToAimBallByVision())
				{
					ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //对准球的过程中突然丢了
						findballflag = 0;
						intention_num = 2;
						ROS_INFO("丢了 丢了 丢了 : %lf \n", objectAngleDistance_Y.first);
					}
					else
					{
						aimballflag = 1;
						ROS_INFO("!!!!!!!!中圈球:objectAngle aimedaimedaimed is %lf;中圈球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					}
				}
			}
			if ((findballflag == 1) && (aimballflag == 1))
			{
				if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
				{
					ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //前往球的跟前过程中，又没看到球，重新去找
						aimballflag = 0;
						findballflag = 0;
						intention_num = 2;
						ROS_INFO_STREAM("又丢了 又丢了 又丢了 \n");
					}
					else
					{
						aimballflag = 0;
						findballflag = 0;
						intention_num = 5;
						ROS_INFO_STREAM("捡球 捡球 捡球 \n");
					}
				}
				temp_normal_time = normal_time;
				intention_num = 55;
			}
			break;

		case 55:
			ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000000000000000000000000000000000000\n");
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 121;
			break;

		case 121:
			if ((normal_time - temp_normal_time) > 100) //延时2S
			{
				/*if(ToBallByVision_new())
			{*/
				temp_normal_time = normal_time; //为延时作准备
				intention_num = 66;
				//}
			}
			break;

		case 66:
			ROS_INFO_STREAM("case 55555555555555555555555555555555555555555555555555555555555555555555555555555\n");
			//立即捡球，不要延时
			Stop();
			//m_bSend=1;    //TODO:允许给单片机发送命令
			Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
			temp_normal_time = normal_time;
			intention_num = 77; //抬机械臂命令已发送
			break;
		case 77: //等待机械臂抬起,放臂完成
			ROS_INFO_STREAM("case 66666666666666666666666666666666666666666666666666666666666666666666666666666\n");
			Control.SendData_state = 0; //单片机位置0
			if ((normal_time - temp_normal_time) > 80)
			{
				temp_normal_time = normal_time;
				intention_num = 88;
			}
			break;
		case 88:
		{
			if ((normal_time - temp_normal_time) > 15)
			{
				Stop();
				Control.SendData_state = 2; //发送发射指令，下一步立即置位0
				ROS_INFO_STREAM("发射 发射 发射 ！！！！\n");
				temp_normal_time = normal_time;
				intention_num = 777;
			}

			break;

		case 777:
			ROS_INFO_STREAM("case 1213214 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 \n");
			Control.SendData_state = 0;
			Stop();
			break;
			break;
		}
		break;
		}
	}
	}
	//		 /*Control.V1 = 250;
	//		Control.V2 = -250;
	//		Control.V3 = -250;
	//		Control.V4 = 250;*/
	//		 /*temp_normal_time=normal_time;
	//			intention_num=99;*/
	//		 }
	//		break;
	//case 99:
	// {
	//  if((dsp_status.YDist)<-1500)		//找了好久，一个都没找到，就回去了
	//		{
	//Control.V1 = 250;
	//Control.V2 = -250;
	//Control.V3 = -250;
	//Control.V4 = 250;
	//		}
	// }break;
}

void DecisionMaking::Normal_bat5()
{
	static int intention_num = 1; //回合过程的意图顺序号
	static bool findballflag = 0; //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
	static bool aimballflag = 0;

	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	static int vision_scan_time = 0; //视觉扫描次数统计，达到设定值时，车子要回归原位
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptOO, ptA, ptB, ptM, ptM2, ptN, ptxy;

	ptOO.x = 1000;
	ptOO.y = 0; //回合结束前，用于回位定点

	ptA.x = 2450;
	ptA.y = 0;

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;

	case 120:
		if ((normal_time - temp_normal_time) > 200) //延时大约9s
		{
			temp_normal_time = normal_time;
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111111111111\n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid1(ptA)) //走到点A
		{
			ROS_INFO("我到 A 点了 我到 A 点了 我到 A 点了 ！！！X is : %ld，Y is : %ld, angle is : %lf\n", dsp_status.XDist, dsp_status.YDist, dsp_status.fAngle);
			Stop();
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;
	}
	//  //一些决策相关的标志
	//	static int intention_num=0;       //回合过程的意图顺序号
	//    static bool findballflag=0;    //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
	//	static bool aimballflag=0;
	//
	//	static int normal_time=0;       //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	//	static int temp_normal_time=0;     //辅助延时，记下需要延时时刻的time,延时完成要清零
	//	static int vision_scan_time=0;     //视觉扫描次数统计，达到设定值时，车子要回归原位
	//	normal_time++;//DSP 45ms发送一次数据
	//
	//	Posture ptOO, ptA, ptB, ptxy;
	//
	//	ptOO.x=1000;
	//	ptOO.y=0;					//回合结束前，用于回位定点
	//
	//	ptxy.x=0;
	//	ptxy.y=0;
	//
	//	ptA.x=5200;
	//	ptA.y=0;					//车子带着篮球进入传球边界线内的点，车子在此处往目标点M发射球
	//
	//	ptB.x=2200;
	//	ptB.y=0;
	//
	//
	//
	//
	//switch(intention_num)
	//{
	//	 case 0:
	//ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000000000000000000000000000000000000\n");
	//		temp_normal_time=normal_time;     //为延时作准备
	//		intention_num=1;
	//		break;
	//
	//	 case 120:
	//		if((normal_time-temp_normal_time)>200)            //延时大约9s
	//		{
	//			temp_normal_time=normal_time;
	//			intention_num=1;
	//		}
	//		break;
	//
	//
	//	 case 1:
	//ROS_INFO_STREAM("case 11111111111111111111111111111111111111111111111111111111111111111111111111111\n");
	//
	//			Control.SendData_state=0;
	//		if(ToPostureByMilemeter(ptA)) //走到点A (ToPostureByMilemeter)ToPostureByMilemeter_Pid
	//		{
	//			ROS_INFO_STREAM("我到 A 点了 我到 A 点了 我到 A 点了 ！！！X is : %ld，Y is : %ld, angle is : %lf\n",dsp_status.XDist, dsp_status.YDist,dsp_status.fAngle);
	//			Stop();
	//			temp_normal_time=normal_time;
	//			intention_num=4;
	//		}
	//		break;
	//
	//		//void CDecisionMaking::Normal_pass0()  //激光摄像头测试
	//	 case 4:
	//{
	//	static int intention_num=5;       //回合过程的意图顺序号
	//	static bool findballflag=0;    //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
	//	static bool aimballflag=0;
	//	static bool armupflag=0;
	//	static bool stopflag=0;
	//	static int normal_time=0;       //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	//	static int temp_normal_time=0;     //辅助延时，记下需要延时时刻的time,延时完成要清零
	//	normal_time++;
	//
	//switch(intention_num)
	//{
	//	case 5:
	//	ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000000000000000000000000000000000000\n");
	//		temp_normal_time=normal_time;     //为延时作准备
	//		intention_num=22;
	//		break;
	//
	//	case 120:
	//		if((normal_time-temp_normal_time)>45)            //延时2S
	//		{
	//			/*if(ToBallByVision_new())
	//			{*/
	//				temp_normal_time=normal_time;     //为延时作准备
	//				intention_num=2;
	//			//}
	//		}
	//		break;
	//
	//	case 22:
	//	ROS_INFO_STREAM("case 2222222222222222222222222222222222222222222222222222222222222222222222222222222\n");
	//		//temp_normal_time=normal_time;     //为延时作准备
	//		if(!Vision.create_communication_RAM_2())
	//		{
	//			ROS_INFO_STREAM("fail to create_communication_RAM_2\n");
	//		}
	//		else
	//		{
	//			ROS_INFO_STREAM("Success to create_communication_RAM_2\n");
	//			if(!Vision.open_file_mapping_2())
	//			{
	//				ROS_INFO_STREAM("fail to open_file_mapping_2\n");
	//			}
	//			else
	//			{
	//				ROS_INFO_STREAM("Success to open_file_mapping_2\n");
	//				if(!Vision.create_communication_RAM_4())
	//				{
	//					ROS_INFO_STREAM("fail to create_communication_RAM_4\n");
	//				}
	//				else
	//				{
	//					ROS_INFO_STREAM("Success to create_communication_RAM_4\n");
	//					if(!Vision.open_file_mapping_4())
	//					{
	//						ROS_INFO_STREAM("fail to open_file_mapping_4\n");
	//					}
	//					else
	//					{
	//						ROS_INFO_STREAM("Success to open_file_mapping_4\n");
	//
	//		/*************************视觉修改版找球测试start****************************/
	//						TheObject.whatObject =(objectType)4;//找什么东西在这里改
	//						if(TheObject.whatObject==(objectType)1)
	//							ROS_INFO_STREAM("send command to find calibration\n");
	//						if(TheObject.whatObject==(objectType)2)
	//							ROS_INFO_STREAM("send command to find reddish_brown_basketball\n");
	//						if(TheObject.whatObject==(objectType)3)
	//							ROS_INFO_STREAM("send command to find basketball\n");
	//						if(TheObject.whatObject==(objectType)4)
	//							ROS_INFO_STREAM("send command to find blue_gray_basketball\n");
	//						if(TheObject.whatObject==(objectType)5)
	//							ROS_INFO_STREAM("send command to find calibration_plus\n");
	//						if(TheObject.whatObject==(objectType)6)
	//							ROS_INFO_STREAM("send command to find orange_red_volleyball\n");
	//						if(TheObject.whatObject==(objectType)7)
	//							ROS_INFO_STREAM("send command to find red_blue_volleyball\n");
	//						if(TheObject.whatObject==(objectType)8)
	//							ROS_INFO_STREAM("send command to find volleyball\n");
	//
	//						if(!Vision.send_find_whatobject(TheObject.whatObject))
	//						{
	//							ROS_INFO_STREAM("Fail to send command\n");
	//						}
	//						else
	//						{
	//							ROS_INFO_STREAM(" Success to send command Success to send command Success to send command Success to send command\n");
	//							TheObject.whatObject = (objectType)0/*unkownObject*/;//变量重新初始化，为下一次找球做准备
	//							if(Vision.receive_final_object_result(objectAngleDistance_Y)==false)
	//							{
	//								ROS_INFO_STREAM("Fail to receive final object result\n");
	//							}
	//							else
	//							{
	//								ROS_INFO_STREAM("Success to receive final object result\n");
	//								ROS_INFO_STREAM("ball distance is:%ld\n",objectAngleDistance_Y.second);
	//								ROS_INFO_STREAM("ball Angle is:%lf\n",objectAngleDistance_Y.first);
	//							}
	//						}
	//					}
	//				}
	//			}
	//		}
	//		if((objectAngleDistance_Y.first==0.0)&&(objectAngleDistance_Y.second==0))
	//		{
	//			//ToFindObjectByturnright(); //左旋找球测试（需要测试旋转找球时请将该函数释放）
	//			//ROS_INFO_STREAM(" 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转 我在左转\n");
	//			robotstraightright();	//右移找球测试
	//			ROS_INFO_STREAM(" 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 我在右移 \n");
	//
	//		}
	//		else
	//		{
	//			findballflag=1;
	//			ROS_INFO_STREAM(" findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
	//		}
	//		if((findballflag==1)&&(aimballflag==0))
	//		{
	//
	//			if (ToAimBallByVision())
	//			{
	//				ROS_INFO_STREAM("ToAimBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n",objectAngleDistance_Y.first);
	//				if ((0.0==objectAngleDistance_Y.first)&&(0.0==objectAngleDistance_Y.second))
	//				{//对准球的过程中突然丢了
	//					findballflag=0;
	//					intention_num=2;
	//					ROS_INFO_STREAM("丢了 丢了 丢了 : %lf \n",objectAngleDistance_Y.first);
	//				}
	//				else
	//				{
	//					aimballflag=1;
	//					ROS_INFO_STREAM("!!!!!!!!中圈球:objectAngle aimedaimedaimed is %lf;中圈球:distance aimedaimedaimed is %ld!!!!!!!!\n",objectAngleDistance_Y.first,objectAngleDistance_Y.second);
	//				}
	//			}
	//
	//		}
	//		if((findballflag==1)&&(aimballflag==1))
	//		{
	//			if(ToBallByVision(objectAngleDistance_Y.second,objectAngleDistance_Y.first))
	//			{
	//				ROS_INFO_STREAM("ToBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n",objectAngleDistance_Y.first);
	//				if((0.0==objectAngleDistance_Y.first)&&(0.0==objectAngleDistance_Y.second))
	//				{//前往球的跟前过程中，又没看到球，重新去找
	//					aimballflag=0;
	//					findballflag=0;
	//					intention_num=2;
	//					ROS_INFO_STREAM("又丢了 又丢了 又丢了 \n");
	//				}
	//				else
	//				{
	//					aimballflag=0;
	//					findballflag=0;
	//					intention_num=5;
	//					ROS_INFO_STREAM("捡球 捡球 捡球 \n");
	//				}
	//			}
	//			temp_normal_time=normal_time;
	//		intention_num=55;
	//		}
	//	break;
	//
	//	case 55:
	//	ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000000000000000000000000000000000000\n");
	//		temp_normal_time=normal_time;     //为延时作准备
	//		intention_num=121;
	//		break;
	//
	//	case 121:
	//		if((normal_time-temp_normal_time)>100)            //延时2S
	//		{
	//			/*if(ToBallByVision_new())
	//			{*/
	//				temp_normal_time=normal_time;     //为延时作准备
	//				intention_num=66;
	//			//}
	//		}
	//		break;
	//
	//	 case 66:
	//ROS_INFO_STREAM("case 55555555555555555555555555555555555555555555555555555555555555555555555555555\n");
	//		//立即捡球，不要延时
	//		Stop();
	//		m_bSend=1;    //允许给单片机发送命令
	//		Control.SendData_state=1;//球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
	//		temp_normal_time=normal_time;
	//		intention_num=77;     //抬机械臂命令已发送
	//		break;
	//	 case 77:			//等待机械臂抬起,放臂完成
	//ROS_INFO_STREAM("case 66666666666666666666666666666666666666666666666666666666666666666666666666666\n");
	//		Control.SendData_state=0;//单片机位置0
	//		if((normal_time-temp_normal_time)>80)
	//		{
	//			temp_normal_time=normal_time;
	//			intention_num=88;
	//		}
	//		break;
	//	 case 88:
	//		 {
	//			if((normal_time-temp_normal_time)>15)
	//		{
	//			Stop();
	//			Control.SendData_state=2;//发送发射指令，下一步立即置位0
	//			ROS_INFO_STREAM("发射 发射 发射 ！！！！\n");
	//			temp_normal_time=normal_time;
	//			intention_num=777;
	//		}
	//
	//		break;
	//
	//		case 777:
	//			ROS_INFO_STREAM("case 1213214 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 29 \n");
	//		Control.SendData_state=0;
	//		Stop();
	//		temp_normal_time=normal_time;
	//			intention_num=711;
	//		}break;
	//	 case 711:
	//		 {
	//			 Control.SendData_state=0;
	//		if(ToPostureByMilemeter1(ptB)) //走到点O (ToPostureByMilemeter)ToPostureByMilemeter_Pid
	//		{
	//			ROS_INFO_STREAM("我到 0 点了 我到 0 点了 我到 0 点了 ！！！X is : %ld，Y is : %ld, angle is : %lf\n",dsp_status.XDist, dsp_status.YDist,dsp_status.fAngle);
	//			Stop();
	//			temp_normal_time=normal_time;
	//			intention_num=4;
	//		}
	//		break;
	//		 }
	//}
	//}
	//}
	////		 /*Control.V1 = 250;
	////		Control.V2 = -250;
	////		Control.V3 = -250;
	////		Control.V4 = 250;*/
	////		 /*temp_normal_time=normal_time;
	////			intention_num=99;*/
	////		 }
	////		break;
	//	 //case 99:
	//		// {
	//  //  if((dsp_status.YDist)<-1500)		//找了好久，一个都没找到，就回去了
	//		//		{
	//		//Control.V1 = 250;
	//		//Control.V2 = -250;
	//		//Control.V3 = -250;
	//		//Control.V4 = 250;
	//		//		}
	//		// }break;
	//
	//	}
}

void DecisionMaking::PidAngle_init(float kp, float ki, float kd) //PidAngle控制算法的变量初始化
{
	pid_angle.set_value = 0;
	pid_angle.actual_value = 0;
	pid_angle.err = 0;
	pid_angle.err_last = 0;
	pid_angle.input_value = 0;
	pid_angle.integral = 0;

	pid_angle.Kp = kp;
	pid_angle.Ki = ki;
	pid_angle.Kd = kd;
}

void DecisionMaking::PidDistX_init(float kp, float ki, float kd) //PidDist控制算法的变量初始化
{
	pid_distx.set_value = 0;
	pid_distx.actual_value = 0;
	pid_distx.err = 0;
	pid_distx.err_last = 0;
	pid_distx.input_value = 0;
	pid_distx.integral = 0;

	pid_distx.Kp = kp;
	pid_distx.Ki = ki;
	pid_distx.Kd = kd;
}

void DecisionMaking::PidDistY_init(float kp, float ki, float kd) //PidDist控制算法的变量初始化
{
	pid_disty.set_value = 0;
	pid_disty.actual_value = 0;
	pid_disty.err = 0;
	pid_disty.err_last = 0;
	pid_disty.input_value = 0;
	pid_disty.integral = 0;

	pid_disty.Kp = kp;
	pid_disty.Ki = ki;
	pid_disty.Kd = kd;
}

float DecisionMaking::Pidlow_angle(float angle) //慢速的角度调整，快速移动时用
{
	//这里kp之前默认为2
	pid_angle.err = angle;

	float k = 1;
	if (fabs(pid_angle.err) < 15)
		//k=0.3;
		k = 1.4;
	else
		//k=1;          //定义一个比例系数k，当偏差小于15时，使得比例系数扩大1.5倍（其实就是分段P控制了一下）
		k = 1;

	pid_angle.integral += pid_angle.err;
	pid_angle.input_value = k * pid_angle.Kp * pid_angle.err + pid_angle.Ki * pid_angle.integral + pid_angle.Kd * (pid_angle.err - pid_angle.err_last);

	//！！！设定车体旋转角速度的最高值，单位度/s
	//注意：不能限较高的最低速（特别是这个最低速度还不小时），因为当偏差很小时，由于限制了最低速度，会超调到反向偏差（每次采样时间和速度决定了每次调节的偏差值），如此不断震荡。
	if (fabs(pid_angle.input_value) > 30)
	{
		if (pid_angle.err > 0)
			pid_angle.input_value = 30;
		else
			pid_angle.input_value = -30;
	}

	pid_angle.err_last = pid_angle.err;

	return pid_angle.input_value;
}

float DecisionMaking::Pid_angle_new(float angle)
{
	ROS_INFO("pid调节前的角度为：%lf \n", angle);
	pid_angle.err = angle;
	float k = 0;
	if ((angle >= 0) && (angle <= 180))
	{
		if (angle <= 10)
			k = 0.85;
		else if (angle <= 25)
			k = 0.6;
		else if (angle <= 40)
			k = 0.65;
		else if ((angle > 40) && (angle <= 80))
			k = 0.7; //k=-(1-((1-0.4)/(140-40))*(angle-40));
		else if ((angle > 80) && (angle <= 110))
			k = 0.6;
		else
			k = 0.46;
	}
	else
	{
		if (angle >= -10)
			k = 0.85;
		else if (angle >= -25)
			k = 0.6; //k=0.75;
		else if (angle >= -40)
			k = 0.65; //k=0.8;
		else if ((angle < -40) && (angle >= -80))
			k = 0.7; //k=0.9;//k=1-((1-0.4)/(140-40))*(angle-40);
		else if ((angle < -80) && (angle >= -110))
			k = 0.6; //k=0.88;
		else
			k = 0.46;
	}
	pid_angle.integral += pid_angle.err;
	pid_angle.input_value = k * (pid_angle.Kp * pid_angle.err + pid_angle.Ki * pid_angle.integral + pid_angle.Kd * (pid_angle.err - pid_angle.err_last));

	pid_angle.err_last = pid_angle.err;
	//PidAngle.input_value = k * PidAngle.Kp * PidAngle.err;
	return pid_angle.input_value;
}

float DecisionMaking::Pid_distX(float dist) //相对X轴距离的PID控制实现
{
	pid_distx.err = dist;
	float k;
	if (pid_distx.err <= 1000)
	{
		k = 0.4;
	}
	else if ((pid_distx.err > 1000) && (pid_distx.err <= 4000))
	{
		k = 0.4 - ((0.3 - 0.05) / (4000 - 1000)) * (pid_distx.err - 1000);
	}
	else if ((pid_distx.err > 4000) && (pid_distx.err <= 5000))
	{
		k = 0.18 - ((0.18 - 0.08) / (5000 - 4000)) * (pid_distx.err - 4000);
	}
	else
	{
		k = 0.08;
	}
	pid_distx.integral += pid_distx.err;
	pid_distx.input_value = k * (pid_distx.Kp * pid_distx.err + pid_distx.Ki * pid_distx.integral + pid_distx.Kd * (pid_distx.err - pid_distx.err_last));
	if (pid_distx.input_value < 200)
	{
		pid_distx.input_value = 200;
	}
	if (pid_distx.input_value > 1300)
	{
		pid_distx.input_value = 1300;
	}
	pid_distx.err_last = pid_distx.err;
	return pid_distx.input_value;
}

float DecisionMaking::Pid_distY(float dist) //相对Y轴距离的PID控制实现
{
	pid_disty.err = dist;
	float k = 1;
	if (abs(pid_disty.err) < 15)
		//k=0.3;
		k = 1.2;
	else
		//k=1;          //定义一个比例系数k，当偏差小于15时，使得比例系数扩大1.5倍（其实就是分段P控制了一下）
		k = 0.8;
	pid_disty.integral += pid_disty.err;
	pid_disty.input_value = k * (pid_disty.Kp * pid_disty.err + pid_disty.Ki * pid_disty.integral + pid_disty.Kd * (pid_disty.err - pid_disty.err_last));

	pid_disty.err_last = pid_disty.err;
	return pid_disty.input_value;
}

float DecisionMaking::Pid_realize(float set, float actual) //常规PID实现
{
	pid_angle.set_value = set;
	pid_angle.err = pid_angle.set_value - actual;
	pid_angle.integral += pid_angle.err;
	pid_angle.input_value = pid_angle.Kp * pid_angle.err + pid_angle.Ki * pid_angle.integral + pid_angle.Kd * (pid_angle.err - pid_angle.err_last);
	pid_angle.err_last = pid_angle.err;
	return pid_angle.input_value;
}

void DecisionMaking::GetNspeed(short Vx, short Vy, double W, float jd) //轮速转换函数，换算得到轮子的给定转速,Vx,Vy为车体在以出发点为坐标原点的X/Y方向的实际速度，单位mm/s，W为车体自身旋转角速度，单位角度/s
{
	jd = -jd;																																  //逻辑取反，理解起来有点绕（比如车子要往30度方向走，按照只给Vx应该先让它顺时针转30度才会是往30度正前方走，但现在车头还是朝向出发时的正前方，相对于期望的转30度后的方位角就是-30度了！）
	Control.V1 = (short)(-sin((45.00 - jd) * pi / 180) * Vx - cos((45.00 - jd) * pi / 180) * Vy - 304 * W * pi / 180.0) / (127 * pi / 612.5); //(76.2*pi*2.0)/12.25*60
	Control.V2 = (short)(cos((45.00 - jd) * pi / 180) * Vx - sin((45.00 - jd) * pi / 180) * Vy - 304 * W * pi / 180.0) / (127 * pi / 612.5);
	Control.V3 = (short)(sin((45.00 - jd) * pi / 180) * Vx + cos((45.00 - jd) * pi / 180) * Vy - 304 * W * pi / 180.0) / (127 * pi / 612.5);
	Control.V4 = (short)(-cos((45.00 - jd) * pi / 180) * Vx + sin((45.00 - jd) * pi / 180) * Vy - 304 * W * pi / 180.0) / (127 * pi / 612.5);
}

double DecisionMaking::mth_PointsDist(Posture a, Posture b)
{
    return sqrt((float)((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)));
}

double DecisionMaking::mth_PointsDist(Posture a)
{
    return sqrt((float)(a.x*a.x+a.y*a.y));
}

double DecisionMaking::mth_ChangeAngle(double theta)
{
    while(theta>180)
        theta-=360;
    while(theta<=180)
        theta+=180;
    return theta;
}

bool DecisionMaking::ToAngleByPosture(Posture pt)
{
	Posture ept;
	double theta1, theta2, Er;
	float Vx, Vy, W, jd;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);	//转化为角度制
	theta1 = (double)dsp_status.fAngle; //当前机器人偏离X轴角度,角度制
	theta2 = atan2(ept.y, ept.x) * 180 / pi;
	//double atan2(double y,double x) 返回的是原点至点(x,y)的方位角，即与 x 轴的夹角。
	//也可以理解为复数 x+yi 的辐角。返回值的单位为弧度，取值范围为(-pi,pi)
	Er = DecisionMaking::mth_ChangeAngle(theta2 - theta1); //**2、机器人当前位姿偏离目标点的角度Er，角度制
	Er = DecisionMaking::mth_ChangeAngle(Er);

	ROS_INFO("dsp_status.fAngle theta1 is %f\n", theta1);
	ROS_INFO("atan2(ept.y,ept.x) theat2 is %f\n", theta2);
	ROS_INFO("ToAngleByPosture Er is:%f\n", Er);

	if ((Er > 0) && (Er <= 180))
	{
		if (Er > 15)
			ToFindObjectByturnright_high();
		else
			ToFindObjectByturnrightlow();
	}
	else
	{
		if (Er < -15)
			ToFindObjectByturnleft_high();
		else
			ToFindObjectByturnleftlow();
	}
	Control.flag_start_stop = 1; //允许电机转动
	if (fabs(Er) < 2.5)
	{
		stop();
		ROS_INFO_STREAM("ToAngleByMPU6050() stop11111111111111111111111111111111111111\n");
		return 1;
	}
	else
		return 0;
}

bool DecisionMaking::ToAngleByPosture2(Posture pt) //2019 new
{
	Posture ept;
	double theta1, theta2, Er;
	float Vx, Vy, W, jd;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);	//转化为角度制
	theta1 = (double)dsp_status.fAngle; //当前机器人偏离X轴角度,角度制
	theta2 = atan2(ept.y, ept.x) * 180 / pi;
	//double atan2(double y,double x) 返回的是原点至点(x,y)的方位角，即与 x 轴的夹角。
	//也可以理解为复数 x+yi 的辐角。返回值的单位为弧度，取值范围为(-pi,pi)
	Er = DecisionMaking::mth_ChangeAngle(theta2 - theta1); //**2、机器人当前位姿偏离目标点的角度Er，角度制
	Er = DecisionMaking::mth_ChangeAngle(Er);

	ROS_INFO("dsp_status.fAngle theta1 is %f\n", theta1);
	ROS_INFO("atan2(ept.y,ept.x) theat2 is %f\n", theta2);
	ROS_INFO("ToAngleByPosture Er is:%f\n", Er);

	if ((Er >= -180) && (Er <= 180))
		//{
		//if(Er>15)
		//	ToFindObjectByturnright_high();
		//	else
		//		ToFindObjectByturnrightlow();
		//}else
		//{
		//	if(Er<-15)
		//	ToFindObjectByturnleft_high();
		//	else
		//	ToFindObjectByturnleftlow();
		//}
		Vx = 950;
	Vy = 0;
	jd = (Er);
	W = 0;
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
	if ((abs((dsp_status.XDist) - (pt.x)) < 50) && (abs((dsp_status.YDist) - (pt.y)) < 50))
	{
		Stop();
		ROS_INFO_STREAM("已经到目标点！！！\n");
		return 1;
	}
	/*if(fabs(Er)<2.5)
	{
		stop();
	ROS_INFO_STREAM(" stop11111111111111111111111111111111111111\n");
		return 1;
	}*/
	else
		return 0;
}

bool DecisionMaking::ToAngleByMPU6050(Posture pt) //通过陀螺仪调整自身到一个期望姿态角度以对准某点
{
	Posture ept;
	double theta1, theta2, Er;
	float Vx, Vy, W, jd;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);	//转化为角度制
	theta1 = (double)dsp_status.fAngle; //当前机器人偏离X轴角度,角度制
	theta2 = atan2(ept.y, ept.x) * 180 / pi;
	//double atan2(double y,double x) 返回的是原点至点(x,y)的方位角，即与 x 轴的夹角。
	//也可以理解为复数 x+yi 的辐角。返回值的单位为弧度，取值范围为(-pi,pi)
	Er = DecisionMaking::mth_ChangeAngle(theta2 - theta1); //**2、机器人当前位姿偏离目标点的角度Er，角度制
	Er = DecisionMaking::mth_ChangeAngle(Er);

	ROS_INFO("dsp_status.fAngle theta1 is %f\n", theta1);
	ROS_INFO("atan2(ept.y,ept.x) theat2 is %f\n", theta2);
	ROS_INFO("ToAngleByMPU6050 Er is:%f\n", Er);

	///******************PID控制start******************/
	W = Pid_angle_new((float)Er); //pid比例系数要根据实际情况调节
	ROS_INFO("ToAngleByMPU6050 W is:%f\n", W);

	Vx = 0;
	Vy = 0;
	jd = 0;
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
								 ///*****************PID控制end*******************/

	//给函数添加完成返回值，视为完成的角度根据精度调整
	if (fabs(Er) < 2.5)
	{
		stop();
		ROS_INFO_STREAM("ToAngleByMPU6050() stop11111111111111111111111111111111111111\n");
		return 1;
	}
	else
		return 0;
}

bool DecisionMaking::ToAngleByMPU(double Angle) //通过陀螺仪调整自身到一个期望姿态角度
{
	double theta1, theta2, Er;
	float Vx, Vy, W, jd;

	theta1 = (double)dsp_status.fAngle; //当前机器人偏离X轴角度,角度制
	theta2 = Angle;

	Er = DecisionMaking::mth_ChangeAngle(theta2 - theta1); //**2、机器人当前位姿偏离目标点的角度Er，角度制
	Er = DecisionMaking::mth_ChangeAngle(Er);

	W = Pid_angle_new((float)Er); //pid比例系数要根据实际情况调节
	ROS_INFO("ToAngleByMPU W is :  %f\n", W);
	Vx = 0;
	Vy = 0;
	jd = 0;
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动

	if (fabs(Er) < 1)
	{
		stop();
		ROS_INFO_STREAM("stop11111111111111111111111111111111111111\n");
		return 1;
	}
	else
		return 0;
}

bool DecisionMaking::ToAngleForHomeByMPU6050() //用于走折线快速回位原点
{
	if (abs(dsp_status.fAngle) >= 3.0)
	{
		if (dsp_status.fAngle > 0)
		{
			ToFindObjectByturnleft();
		}
		else
		{
			ToFindObjectByturnright();
		}
		return 0;
	}
	else if (abs(dsp_status.YDist) >= 30)
	{
		if (dsp_status.YDist > 0)
		{
			robotstraightleft();
		}
		if (dsp_status.YDist < 0)
		{
			robotstraightright();
		}
		return 0;
	}
	else if (abs(dsp_status.XDist) >= 30)
	{
		robotback();
		return 0;
	}
	else
	{
		Stop();
		return 1;
	}
}

bool DecisionMaking::ToAimBallByVision() //这是停下的时候通过摄像头返回的角度偏差对准球
{
	double Er;
	short Vx, Vy, W;
	float jd;

	Er = 90.00 - objectAngleDistance_Y.first;
	ROS_INFO("ToAimBallByVision 里的角度 Er is : %lf \n", Er);
	if (fabs(Er) >= 2)
	{
		W = 0.2 * Pidlow_angle(Er); //此比例系数要根据实际情况调节，也可以通过形参传递过来
		if (abs(W) < aim_Wmin)
			if (W > 0)
				W = aim_Wmin;
			else
				W = -aim_Wmin;
	}
	else
		W = 0;
	ROS_INFO("ToAimBallByVision 里的角速度 W is: %d \n", W);
	Vx = 0;
	Vy = 0;
	jd = 0;

	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("ToAimBallByVision 里的旋转:%d\n", W);
	ROS_INFO("ToAimBallByVision 里的角度 Er is : %lf \n", Er);
	if (fabs(Er) < 2)
	{
		stop();
		return 1;
		ROS_INFO("ToAimBallByVision 对准了 对准了 对准了 :%d\n", W);
	}
	else if (90.00 == Er) //对准的过程中突然来了一个0度，要做处理，这里是在DecisionMaking里完成
	{
		ROS_INFO_STREAM("突然来个90度，需要处理呀！！！");
		return 1;
	}
	else
		return 0;
}

void DecisionMaking::ToFindObjectByturnleft()
{
	Control.V1 = 150; //新换的大轮给100的轮速可以识别，给150试一下
	Control.V2 = 150;
	Control.V3 = 150;
	Control.V4 = 150;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("ToFindObjectByturnleft V1 V2 V3 V4 is %d ,%d ,%d, %d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::ToFindObjectByturnleft_high()
{
	Control.V1 = 700; //新换的大轮给100的轮速可以识别，给150试一下
	Control.V2 = 700;
	Control.V3 = 700;
	Control.V4 = 700;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("ToFindObjectByturnleft V1 V2 V3 V4 is %d ,%d ,%d, %d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::ToFindObjectByturnleftlow()
{
	Control.V1 = 1; //新换的大轮给100的轮速可以识别，给150试一下
	Control.V2 = 1;
	Control.V3 = 1;
	Control.V4 = 1;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("ToFindObjectByturnleft V1 V2 V3 V4 is %d ,%d ,%d, %d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::ToFindObjectByturnright()
{
	Control.V1 = -150;
	Control.V2 = -150;
	Control.V3 = -150;
	Control.V4 = -150;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("ToFindObjectByturnright V1 V2 V3 V4 is %d , %d , %d, %d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::ToFindObjectByturnright_high()
{
	Control.V1 = -700; //新换的大轮给100的轮速可以识别，给150试一下
	Control.V2 = -700;
	Control.V3 = -700;
	Control.V4 = -700;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("ToFindObjectByturnleft V1 V2 V3 V4 is %d ,%d ,%d, %d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::ToFindObjectByturnrightlow()
{
	Control.V1 = -1;
	Control.V2 = -1;
	Control.V3 = -1;
	Control.V4 = -1;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("ToFindObjectByturnright V1 V2 V3 V4 is %d , %d , %d, %d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotback() //开环后退
{
	Control.V1 = 500;
	Control.V2 = -500;
	Control.V3 = -500;
	Control.V4 = 500;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotback V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotbacklow() //开环后退
{
	Control.V1 = 250;
	Control.V2 = -250;
	Control.V3 = -250;
	Control.V4 = 250;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotbacklow V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotbacktoolow() //开环后退
{
	Control.V1 = 100;
	Control.V2 = -100;
	Control.V3 = -100;
	Control.V4 = 100;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotbacklow V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotforward() //开环前进
{
	Control.V1 = -500;
	Control.V2 = 500;
	Control.V3 = 500;
	Control.V4 = -500;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotforward V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotforwardlow() //开环前进
{
	Control.V1 = -250;
	Control.V2 = 250;
	Control.V3 = 250;
	Control.V4 = -250;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotforwardlow V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotforwardtoolow() //开环前进
{
	Control.V1 = -100;
	Control.V2 = 100;
	Control.V3 = 100;
	Control.V4 = -100;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotforwardlow V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotstraightright() //开环横向右移
{
	Control.V1 = -350; //-300; //450(2017)
	Control.V2 = -350; //-300;
	Control.V3 = 350;  //300;
	Control.V4 = 350;  //300;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotstraightright V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotstraightleft() //开环横向左移
{
	Control.V1 = 350;  //300; //450(2017)
	Control.V2 = 350;  //300;
	Control.V3 = -350; //-300;
	Control.V4 = -350; //-300;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotstraughtleft V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotstraightrightlow() //开环横向右移(速度略低)
{
	Control.V1 = -220;
	Control.V2 = -220;
	Control.V3 = 220;
	Control.V4 = 220;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotstraightrightlow V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotstraightleftlow() //开环横向左移(速度略低)
{
	Control.V1 = 220;
	Control.V2 = 220;
	Control.V3 = -220;
	Control.V4 = -220;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotstraughtleftlow V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robot45back() //开环45度后退
{
	Control.V1 = 450;
	Control.V2 = 0;
	Control.V3 = -450;
	Control.V4 = 0;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robot45back V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robot45backlow() //开环45度后退
{
	Control.V1 = 150;
	Control.V2 = 0;
	Control.V3 = -150;
	Control.V4 = 0;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robot45backlow V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotfu45back() //开环-45度后退
{
	Control.V1 = 0;
	Control.V2 = -450;
	Control.V3 = 0;
	Control.V4 = 450;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotfu45back V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotfu45backlow() //开环-45度后退
{
	Control.V1 = 0;
	Control.V2 = -150;
	Control.V3 = 0;
	Control.V4 = 150;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotfu45backlow V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

bool DecisionMaking::ToPostureByMilemeter_Pid(Posture sourpt, Posture destpt)
{
	//处理当前位姿距目标点的距离Ed（mm）、角度偏差Er(角度) **** (start)
	double Ed, Er; //定义当前位置到目标点的角度偏差、距离
	short Vx, Vy, W;
	float jd, aimjd;
	Posture ept;

	ept.x = (destpt.x - dsp_status.XDist);
	ept.y = (destpt.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept); //**1、机器人当前位置点距目标点的距离Ed

	aimjd = atan2((double)(destpt.y - sourpt.y), (double)(destpt.x - sourpt.x)) * 180 / pi;
	aimjd = DecisionMaking::mth_ChangeAngle(aimjd);

	Er = atan2((double)ept.y, (double)ept.x) * 180 / pi; //转化为角度制
	Er = DecisionMaking::mth_ChangeAngle(Er);
	//处理当前位姿距目标点的距离、角度偏差end

	//***测试9
	ROS_INFO("ToPostureByMilemeter_Pid Ed is:%f\n", Ed);
	ROS_INFO("ToPostureByMilemeter_Pid aimjd is:%f\n", aimjd);
	ROS_INFO("ToPostureByMilemeter_Pid Er is:%f\n", Er);

	if (abs(aimjd - dsp_status.fAngle) <= 3)
		W = 0;
	else
		W = 0.6 * Pidlow_angle(Er);
	ROS_INFO("ToPostureByMilemeter_Pid W is %d\n", W);

	static bool startflag1 = false;
	static bool startflag2 = false;
	static int speedcount = 0;
	speedcount++;
	if (!startflag1 && !startflag2)
	{
		if (speedcount < 3)
			if (Ed > 0)
				Vx = 200;
			else
				Vx = -200;
		else
			startflag1 = true;
	}
	if (startflag1 && !startflag2)
	{
		if (speedcount < 6)
			if (Ed > 0)
				Vx = 450;
			else
				Vx = -450;
		else
			startflag2 = true;
	}
	Vy = 0;
	jd = 0;
	if (startflag1 && startflag2)
	{
		if (abs(Ed) > 2000) //注意对Ed分段P控制时，交接处的V不要越变太大。否则较大扭矩损伤电机也影响运动品质。					 //根据决策需要，修改分段距离
		{
			Vx = 700; //Vx=350;
			ROS_INFO("ToPostureByMilemeter_Pid when Ed>2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
		else if ((abs(Ed) <= 2000) && (abs(Ed) > 1000))
		{
			Vx = 600; //	Vx=300;
			ROS_INFO("ToPostureByMilemeter_Pid when 1000<Ed<2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
		else if ((abs(Ed) <= 1000) && (abs(Ed) > 600))
		{
			Vx = 500; //Vx=160;
			ROS_INFO("ToPostureByMilemeter_Pid when 600< Ed<1000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
		else if ((abs(Ed) <= 600) && (abs(Ed) > 400))
		{
			Vx = 400; //Vx=150;
			ROS_INFO("ToPostureByMilemeter_Pid when 400<Ed<600,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
		else if ((abs(Ed) <= 400) && (abs(Ed) > 200))
		{
			Vx = 300; //Vx=140
			ROS_INFO("ToPostureByMilemeter_Pid when 200<Ed<400,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
		if (abs(Ed) <= 200)
		{
			Vx = 200; //Vx=130
			ROS_INFO("ToPostureByMilemeter_Pid when Ed<200,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
	}
	GetNspeed(Vx, Vy, W, jd);
	//（这是主要原因2）这里一定要限定距离停下来（根据情况可能还要调大），因为当距离很小的，哪怕走的很准，X,Y值都误差不大，但此时的偏角是等于X，Y与目标点坐标的差值来算的，是会变的很大的
	if (abs(Ed) < 150 || (abs(ept.x) < 141 && abs(ept.y) < 141))
	{
		startflag1 = false;
		startflag2 = false;
		speedcount = 0;
		stop();
		ROS_INFO_STREAM("ToPostureByMilemeter_Pid stop because abs(Ed)<150||(abs(ept.x)<141&&abs(ept.y)<141) ToPostureByMilemeter_Pid stop\n");
		return 1;
	}
	else if ((abs(ept.x) < 120 || abs(ept.y) < 120))
	{
		startflag1 = false;
		startflag2 = false;
		speedcount = 0;
		stop();
		ROS_INFO_STREAM("ToPostureByMilemeter_Pid stopbecause abs(ept.x)<120||abs(ept.y)<120 ToPostureByMilemeter_Pid stop\n");
		return 1;
	}
	else
		return 0;
	Control.flag_start_stop = 1; //允许电机转动
								 /******************PID控制end******************/
}

bool DecisionMaking::ToPostureByMilemeter(Posture pt)
{
	//处理当前位姿距目标点的距离Ed（mm）、角度偏差Er(角度) **** (start)
	double Er, Ed; //定义当前位置到目标点的角度偏差、距离
	short Vx, Vy, W;
	float jd;
	Posture ept;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept); //机器人当前位置点距目标点的距离Ed

	double theta1, theta2;
	theta1 = (double)dsp_status.fAngle;						 //当前机器人偏离X轴角度,角度制
	theta2 = atan2((double)ept.y, (double)ept.x) * 180 / pi; //转化为角度制
	Er = DecisionMaking::mth_ChangeAngle(theta2 - theta1);					 //机器人当前位姿偏离目标点的角度Er，角度制
	Er = DecisionMaking::mth_ChangeAngle(Er);
	//处理当前位姿距目标点的距离、角度偏差end
	ROS_INFO("ToPostureByMilemeter_Pid Ed is:%f\n", Ed);
	ROS_INFO("ToPostureByMilemeter_Pid Er is:%f\n", Er);

	if (abs(Er) < 2)
		W = 0;
	else
		W = 0.3 * Pidlow_angle(Er);

	static bool startflag1 = false;
	static bool startflag2 = false;
	static int speedcount = 0;
	speedcount++;
	if (!startflag1 && !startflag2)
	{
		if (speedcount < 3)
			if (Ed > 0)
				Vx = 200;
			else
				Vx = -200;
		else
			startflag1 = true;
	}
	if (startflag1 && !startflag2)
	{
		if (speedcount < 6)
			if (Ed > 0)
				Vx = 400;
			else
				Vx = -400;
		else
			startflag2 = true;
	}
	Vy = 0;
	jd = 0;
	if (startflag1 && startflag2)
	{
		if (abs(Ed) > 2000)
		{
			Vx = 600;
			ROS_INFO("ToPostureByMilemeter when Ed>2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}

		else if ((abs(Ed) <= 2000) && (abs(Ed) > 1000))
		{
			Vx = 500;
			ROS_INFO("ToPostureByMilemeter when 1000<Ed<2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
		else if ((abs(Ed) <= 1000) && (abs(Ed) > 400))
		{
			Vx = 400;
			ROS_INFO("ToPostureByMilemeter when 400< Ed<1000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
		if (abs(Ed) <= 400)
		{
			W = 0;
			Vx = 300;
			ROS_INFO("ToPostureByMilemeter when Ed<400,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
	}
	GetNspeed(Vx, Vy, W, jd);
	//（这是主要原因2）这里一定要限定距离停下来（根据情况可能还要调大），因为当距离很小的，哪怕走的很准，X,Y值都误差不大，但此时的偏角
	if (fabs(Ed) < 150) //是等于X，Y与目标点坐标的差值来算的，是会变的很大的
	{
		startflag1 = false;
		startflag2 = false;
		speedcount = 0;
		ROS_INFO_STREAM("ToPostureByMilemeter() stop111111111111111111111111111111111111111111111\n");
		return 1;
	}
	else
		return 0;
	Control.flag_start_stop = 1; //允许电机转动
}

bool DecisionMaking::ToPostureByMilemeter_Pid1(Posture pt)
{
	//处理当前位姿距目标点的距离Ed（mm）、角度偏差Er(角度) **** (start)
	double Ed; //定义当前位置到目标点的距离
	float jd;
	short Vx, Vy, W;
	Posture ept;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept); //**1、机器人当前位置点距目标点的距离Ed

	//处理当前位姿距目标点的距离end

	//***测试9
	ROS_INFO("ToPostureByMilemeter_Pid1 Ed is:%f\n", Ed);

	Vy = 0;
	W = 0;
	jd = 0;
	static bool startflag1 = false;
	static bool startflag2 = false;
	static bool startflag3 = false;
	static int speedcount = 0;
	speedcount++;
	if (!startflag1 && !startflag2 && !startflag3)
	{
		if (speedcount < 3)
			if (Ed > 0)
				Vx = 300;
			else
				Vx = -300;
		else
			startflag1 = true;
	}
	if (startflag1 && !startflag2 && !startflag3)
	{
		if (speedcount < 6)
			if (Ed > 0)
				Vx = 400;
			else
				Vx = -400;
		else
			startflag2 = true;
	}
	if (startflag1 && startflag2 && !startflag3)
	{
		if (speedcount < 9)
			if (Ed > 0)
				Vx = 500;
			else
				Vx = -500;
		else
			startflag3 = true;
	}
	if (startflag1 && startflag2 && startflag3)
		/*速度真正的PID调节start*/
		Vx = Pid_distX(Ed);
	/*速度真正的PID调节end*/

	GetNspeed(Vx, Vy, W, jd);
	if (abs(Ed) < 80 || ((abs(ept.x) < 80)))
	{
		startflag1 = false;
		startflag2 = false;
		startflag3 = false;
		speedcount = 0;
		Stop();
		ROS_INFO_STREAM("ToPostureByMilemeter_Pid1() Stop 距离到了容错距离到了容错距离到了容错\n");
		return 1;
	}
	else
		return 0;
	Control.flag_start_stop = 1; //允许电机转动
								 /*****************PID控制end******************/
}

bool DecisionMaking::ToPostureByMilemeter_Pid2(Posture pt) //通过里程计全向平移到绝对点
{
	//处理当前位姿距目标点的距离Ed（mm）、角度偏差Er(角度) **** (start)
	double Ed, Er; //定义当前位置到目标点的角度偏差、距离
	short Vx, Vy, W;
	float jd;
	Posture ept;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept); //**1、机器人当前位置点距目标点的距离Ed
	Er = atan2(ept.y, ept.x) * 180 / pi;
	Er = DecisionMaking::mth_ChangeAngle(Er);
	jd = Er - dsp_status.fAngle;
	jd = DecisionMaking::mth_ChangeAngle(jd);
	//处理当前位姿距目标点的距离、角度偏差end

	//***测试9
	ROS_INFO("ToPostureByMilemeter_Pid2 Ed is:%f\n", Ed);
	ROS_INFO("ToPostureByMilemeter_Pid2 Er is:%f\n", Er);
	ROS_INFO("ToPostureByMilemeter_Pid2 jd is:%f\n", jd);
	Vy = 0;
	if (fabs(Er) > 3)
	{
		W = 0.2 * Pidlow_angle(Er);
	}
	else
		W = 0;

	static bool startflag1 = false;
	static bool startflag2 = false;
	static int speedcount = 0;
	speedcount++;
	if (!startflag1 && !startflag2)
	{
		if (speedcount < 3)
		{
			if (Ed > 0)
				Vx = 300;
			else
				Vx = -300;
		}
		else
			startflag1 = true;
	}
	if (startflag1 && !startflag2)
	{
		if (speedcount < 6)
		{
			if (Ed > 0)
				Vx = 400;
			else
				Vx = -400;
		}
		else
			startflag2 = true;
	}
	if (startflag1 && startflag2)
	{
		if (speedcount < 9)
		{
			if (Ed > 0)
				Vx = 500;
			else
				Vx = -500;
		}
		else
		{
			if (fabs(Ed) > 2000) //注意对Ed分段P控制时，交接处的V不要越变太大。否则较大扭矩损伤电机也影响运动品质。					 //根据决策需要，修改分段距离
			{
				Vx = 800; //Vx=350;
				ROS_INFO("ToPostureByMilemeter_Pid2 when Ed>2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 2000) && (fabs(Ed) > 1000))
			{
				Vx = 720; //Vx=300;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 1000<Ed<2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 1000) && (fabs(Ed) > 600))
			{
				Vx = 660; //Vx=160;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 600< Ed<1000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 600) && (fabs(Ed) > 400))
			{
				Vx = 560; //Vx=150;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 400<Ed<600,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 400) && (fabs(Ed) > 200))
			{
				Vx = 440; //Vx=140
				ROS_INFO("ToPostureByMilemeter_Pid2 when 200<Ed<400,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			if ((fabs(Ed) <= 200) && (fabs(Ed) >= 100))
			{
				W = 0;
				Vx = 300; //Vx=130
				ROS_INFO("ToPostureByMilemeter_Pid2 when 90<Ed<200,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
		}
	}
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
	if ((fabs(Ed) <= 80) || (abs(ept.x) <= 80) /*||(abs(ept.y)<=130)*/)
	{
		Vx = 0;
		W = 0;
		GetNspeed(Vx, Vy, W, jd);
		Control.flag_start_stop = 1; //允许电机转动
		ROS_INFO("ToPostureByMilemeter_Pid2() Stop;ToPostureByMilemeter_Pid2() Stop;ToPostureByMilemeter_Pid2 Stop()\n");
		return 1;
	}
	else
		return 0;
	/*****************PID控制end******************/
}

bool DecisionMaking::ToPostureByMilemeter_Pid6(Posture pt) //通过里程计全向平移到绝对点
{
	//处理当前位姿距目标点的距离Ed（mm）、角度偏差Er(角度) **** (start)
	double Ed, Er; //定义当前位置到目标点的角度偏差、距离
	short Vx, Vy, W;
	float jd;
	Posture ept;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept); //**1、机器人当前位置点距目标点的距离Ed
	Er = atan2(ept.y, ept.x) * 180 / pi;
	Er = DecisionMaking::mth_ChangeAngle(Er);
	jd = Er - dsp_status.fAngle;
	jd = DecisionMaking::mth_ChangeAngle(jd);
	//处理当前位姿距目标点的距离、角度偏差end

	//***测试9
	ROS_INFO("ToPostureByMilemeter_Pid2 Ed is:%f\n", Ed);
	ROS_INFO("ToPostureByMilemeter_Pid2 Er is:%f\n", Er);
	ROS_INFO("ToPostureByMilemeter_Pid2 jd is:%f\n", jd);
	Vy = 0;
	if (fabs(Er) > 3)
	{
		W = 0.2 * Pidlow_angle(Er);
	}
	else
		W = 0;

	static bool startflag1 = false;
	static bool startflag2 = false;
	static int speedcount = 0;
	speedcount++;
	if (!startflag1 && !startflag2)
	{
		if (speedcount < 3)
		{
			if (Ed > 0)
				Vx = 300;
			else
				Vx = -300;
		}
		else
			startflag1 = true;
	}
	if (startflag1 && !startflag2)
	{
		if (speedcount < 6)
		{
			if (Ed > 0)
				Vx = 400;
			else
				Vx = -400;
		}
		else
			startflag2 = true;
	}
	if (startflag1 && startflag2)
	{
		if (speedcount < 9)
		{
			if (Ed > 0)
				Vx = 500;
			else
				Vx = -500;
		}
		else
		{
			if (fabs(Ed) > 2000) //注意对Ed分段P控制时，交接处的V不要越变太大。否则较大扭矩损伤电机也影响运动品质。					 //根据决策需要，修改分段距离
			{
				Vx = 800; //Vx=350;
				ROS_INFO("ToPostureByMilemeter_Pid2 when Ed>2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 2000) && (fabs(Ed) > 1000))
			{
				Vx = 680; //Vx=300;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 1000<Ed<2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 1000) && (fabs(Ed) > 600))
			{
				Vx = 560; //Vx=160;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 600< Ed<1000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 600) && (fabs(Ed) > 400))
			{
				Vx = 440; //Vx=150;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 400<Ed<600,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 400) && (fabs(Ed) > 200))
			{
				Vx = 320; //Vx=140
				ROS_INFO("ToPostureByMilemeter_Pid2 when 200<Ed<400,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			if ((fabs(Ed) <= 200) && (fabs(Ed) >= 100))
			{
				W = 0;
				Vx = 200; //Vx=130
				ROS_INFO("ToPostureByMilemeter_Pid2 when 90<Ed<200,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
		}
	}
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
	if ((fabs(Ed) <= 100) || (abs(ept.y) <= 50) /*||(abs(ept.x)<=130)*/)
	{
		Vx = 0;
		W = 0;
		GetNspeed(Vx, Vy, W, jd);
		Control.flag_start_stop = 1; //允许电机转动
		ROS_INFO_STREAM("ToPostureByMilemeter_Pid2() Stop;ToPostureByMilemeter_Pid2() Stop;ToPostureByMilemeter_Pid2 Stop()\n");
		return 1;
	}
	else
		return 0;
	/*****************PID控制end******************/
}

bool DecisionMaking::ToPostureByMilemeter_Pid3(Posture pt)
{
	//处理当前位姿距目标点的距离Ed（mm）、角度偏差Er(角度) **** (start)
	double Ed; //定义当前位置到目标点的距离
	float jd;
	short Vx, Vy, W;
	Posture ept;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept); //**1、机器人当前位置点距目标点的距离Ed

	//处理当前位姿距目标点的距离end

	//***测试9
	ROS_INFO("ToPostureByMilemeter_Pid3 Ed is:%f\n", Ed);

	Vy = 0;
	W = 0;
	jd = 0;
	static bool startflag1 = false;
	static bool startflag2 = false;
	static bool startflag3 = false;
	static int speedcount = 0;
	speedcount++;
	if (!startflag1 && !startflag2 && !startflag3)
	{
		if (speedcount < 3)
			if (Ed > 0)
				Vx = 400;
			else
				Vx = -400;
		else
			startflag1 = true;
	}
	if (startflag1 && !startflag2 && !startflag3)
	{
		if (speedcount < 6)
			if (Ed > 0)
				Vx = 450;
			else
				Vx = -450;
		else
			startflag2 = true;
	}
	if (startflag1 && startflag2 && !startflag3)
	{
		if (speedcount < 9)
			if (Ed > 0)
				Vx = 500;
			else
				Vx = -500;
		else
			startflag3 = true;
	}
	if (startflag1 && startflag2 && startflag3)
		/*速度真正的PID调节start*/
		Vx = Pid_distX(Ed);
	/*速度真正的PID调节end*/

	GetNspeed(Vx, Vy, W, jd);
	if ((fabs(Ed) < 100) || (abs(ept.x) < 50) || (abs(ept.y) < 50))
	{
		startflag1 = false;
		startflag2 = false;
		startflag3 = false;
		speedcount = 0;
		Stop();
		ROS_INFO_STREAM("ToPostureByMilemeter_Pid3() Stop 距离到了容错距离到了容错距离到了容错\n");
		return 1;
	}
	else
		return 0;
	Control.flag_start_stop = 1; //允许电机转动
								 /*****************PID控制end******************/
}

bool DecisionMaking::ToPostureByMilemeter_Pid4(Posture pt)
{
	//处理当前位姿距目标点的距离Ed（mm）、角度偏差Er(角度) **** (start)
	double Ed; //定义当前位置到目标点的距离
	float jd;
	short Vx, Vy, W;
	Posture ept;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept); //**1、机器人当前位置点距目标点的距离Ed

	//处理当前位姿距目标点的距离end

	//***测试9
	ROS_INFO("ToPostureByMilemeter_Pid3 Ed is:%f\n", Ed);

	Vy = 0;
	W = 0;
	jd = 0;
	static bool startflag1 = false;
	static bool startflag2 = false;
	static bool startflag3 = false;
	static int speedcount = 0;
	speedcount++;
	if (!startflag1 && !startflag2 && !startflag3)
	{
		if (speedcount < 3)
			if (Ed > 0)
				Vx = 150;
			else
				Vx = -150;
		else
			startflag1 = true;
	}
	if (startflag1 && !startflag2 && !startflag3)
	{
		if (speedcount < 6)
			if (Ed > 0)
				Vx = 300;
			else
				Vx = -300;
		else
			startflag2 = true;
	}
	if (startflag1 && startflag2 && !startflag3)
	{
		if (speedcount < 9)
			if (Ed > 0)
				Vx = 500;
			else
				Vx = -500;
		else
			startflag3 = true;
	}
	if (startflag1 && startflag2 && startflag3)
		/*速度真正的PID调节start*/
		Vx = Pid_distX(Ed);
	/*速度真正的PID调节end*/

	GetNspeed(Vx, Vy, W, jd);
	if (abs(Ed) < 150)
	{
		startflag1 = false;
		startflag2 = false;
		startflag3 = false;
		speedcount = 0;
		Stop();
		ROS_INFO_STREAM("ToPostureByMilemeter_Pid1() Stop 距离到了容错距离到了容错距离到了容错\n");
		return 1;
	}
	/*if((abs(ept.x)<80)||(abs(ept.y)<80))				
	{
		if((abs(ept.x)<80))
		{
			if((abs(ept.y)>80))
			{
				robotforwardlow();
			}else{
				startflag1=false;
				startflag2=false;
				startflag3=false;
				speedcount=0;
				Stop();
				ROS_INFO_STREAM("ToPostureByMilemeter_Pid1() Stop 距离到了容错距离到了容错距离到了容错\n");
				return 1;
			}
		}
		if((abs(ept.y)<80))
		{
			if((abs(ept.x)>80))
			{
				robotforwardlow();
			}else{
				startflag1=false;
				startflag2=false;
				startflag3=false;
				speedcount=0;
				Stop();
				ROS_INFO_STREAM("ToPostureByMilemeter_Pid4() Stop 距离到了容错距离到了容错距离到了容错\n");
				return 1;
			}
		}
	}*/
	else
		return 0;

	Control.flag_start_stop = 1; //允许电机转动
								 /*****************PID控制end******************/
}

bool DecisionMaking::ToPostureByMilemeter_Pid5(Posture pt)
{
	//处理当前位姿距目标点的距离Ed（mm）、角度偏差Er(角度) **** (start)
	double Ed; //定义当前位置到目标点的距离
	float jd;
	short Vx, Vy, W;
	Posture ept;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept); //**1、机器人当前位置点距目标点的距离Ed

	//处理当前位姿距目标点的距离end

	//***测试9
	ROS_INFO("ToPostureByMilemeter_Pid3 Ed is:%f\n", Ed);

	Vy = 0;
	W = 0;
	jd = 0;
	static bool startflag1 = false;
	static bool startflag2 = false;
	static bool startflag3 = false;
	static int speedcount = 0;
	speedcount++;
	if (!startflag1 && !startflag2 && !startflag3)
	{
		if (speedcount < 3)
			if (Ed > 0)
				Vx = 200;
			else
				Vx = -200;
		else
			startflag1 = true;
	}
	if (startflag1 && !startflag2 && !startflag3)
	{
		if (speedcount < 6)
			if (Ed > 0)
				Vx = 350;
			else
				Vx = -350;
		else
			startflag2 = true;
	}
	if (startflag1 && startflag2 && !startflag3)
	{
		if (speedcount < 9)
			if (Ed > 0)
				Vx = 500;
			else
				Vx = -500;
		else
			startflag3 = true;
	}
	if (startflag1 && startflag2 && startflag3)
		/*速度真正的PID调节start*/
		Vx = Pid_distX(Ed);
	/*速度真正的PID调节end*/

	GetNspeed(Vx, Vy, W, jd);
	if (((fabs(Ed) < 100) && (fabs(Ed) > 0)) || (abs(ept.y) < 50))
	{
		startflag1 = false;
		startflag2 = false;
		startflag3 = false;
		speedcount = 0;
		Stop();
		ROS_INFO_STREAM("ToPostureByMilemeter_Pid5() Stop 距离到了容错距离到了容错距离到了容错\n");
		return 1;
	}
	else
		return 0;
	Control.flag_start_stop = 1; //允许电机转动
								 /*****************PID控制end******************/
}

bool DecisionMaking::ToPostureByAvoidance(Posture pt) //通过里程计前后左右平移到绝对点
{
	static bool avoidance_flag = false;
	//static int avoidance_count=0;
	double Edleft, Edright; //定义当前位置到目标点的角度偏差、距离
	double Vx, Vy, W;
	float jd;

	vector<long> data;
	vector<pair<double, long>> objectAngleDist;
	double ki = 0.03; //pid 比例系数
	//TODO:接受激光数据、
	/*
	for(;data.empty();)
	{
		Vision.receive_laser_data(data);
	}
	if(data.size())
	{
		ROS_INFO_STREAM("GET LASER DATA\n");
	}
	else
	{
		ROS_INFO_STREAM("DO NOT GET LASER DATA\n");
	}
	*/

	int returnValue = find_object(data, objectAngleDist);

	if (1 == returnValue)
	{
		avoidance_flag = true;
		ROS_INFO_STREAM("find ball right find ball right find ball right find ball right find ball right find ball right\n");
		ROS_INFO("ToPostureByAvoidance rightball_angle is:%f\n", objectAngleDist[0].first);
		ROS_INFO("ToPostureByAvoidance rightball_dist is:%d\n", objectAngleDist[0].second);
		if (objectAngleDist[0].second < 2000)
		{
			Edright = (objectAngleDist[0].second) * sin(abs(objectAngleDist[0].first) / 57.3);
			ROS_INFO("Edright is %f\n", Edright);
			if (Edright < 500)
				W = ki * Pid_distY(Edright - 500);
			else
				W = 0;
			ROS_INFO("W by Edright is %f\n", W);
			//Vx=400;Vy=0;jd=0;
			if (Edright > 1500) //离右边球比较远，就不去避了
				avoidance_flag = false;
		}
		else
		{
			avoidance_flag = false;
		}
		Vx = 400;
		Vy = 0;
		jd = 0;
	}
	if (-1 == returnValue)
	{
		avoidance_flag = true;
		ROS_INFO_STREAM("find ball left find ball left find ball left find ball left find ball left find ball left \n");
		ROS_INFO("ToPostureByAvoidance leftball_angle is:%f\n", objectAngleDist[0].first);
		ROS_INFO("ToPostureByAvoidance leftball_dist is:%d\n", objectAngleDist[0].second);
		if (objectAngleDist[0].second < 2000)
		{
			Edleft = (objectAngleDist[0].second) * sin(abs(objectAngleDist[0].first) / 57.3);
			ROS_INFO("Edleft is %f\n", Edleft);
			if (Edleft < 500)
				W = -ki * Pid_distY(Edleft - 500);
			else
				W = 0;
			ROS_INFO("W by Edleft is %f\n", W);
			//Vx=400;Vy=0;jd=0;
			if (Edleft > 1500) //离左边球比较远，就不去避了
				avoidance_flag = false;
		}
		else
		{
			avoidance_flag = false;
		}
		Vx = 400;
		Vy = 0;
		jd = 0;
	}
	if (2 == returnValue)
	{
		avoidance_flag = true;
		ROS_INFO_STREAM("find ball left and right find ball left and right find ball left and right find ball left and right \n");
		ROS_INFO("ToPostureByAvoidance rightball_angle is:%f\n", objectAngleDist[0].first);
		ROS_INFO("ToPostureByAvoidance rightball_dist is:%d\n", objectAngleDist[0].second);
		ROS_INFO("ToPostureByAvoidance leftball_angle is:%f\n", objectAngleDist[1].first);
		ROS_INFO("ToPostureByAvoidance leftball_dist is:%d\n", objectAngleDist[1].second);
		Edright = (objectAngleDist[0].second) * sin(abs(objectAngleDist[0].first) / 57.3);
		Edleft = (objectAngleDist[1].second) * sin(abs(objectAngleDist[1].first) / 57.3);
		ROS_INFO("Edright is %f\n", Edright);
		ROS_INFO("Edleft is %f\n", Edleft);
		if ((objectAngleDist[0].second > 2000) && (objectAngleDist[1].second > 2000))
			avoidance_flag = false;
		else if ((objectAngleDist[0].second > 2000) && (objectAngleDist[1].second < 2000))
		{
			if ((Edleft < 500) /*&&(Edright>=500)*/)
			{
				W = -ki * Pid_distY(Edleft - 500);
				ROS_INFO("W by Edleft is %f\n", W);
			}
			else
			{
				W = 0;
			}
		}
		else if ((objectAngleDist[0].second < 2000) && (objectAngleDist[1].second > 2000))
		{
			if ((Edright < 500) /*&&(Edleft>=500)*/)
			{
				W = ki * Pid_distY(Edright - 500);
				ROS_INFO("W by Edright is %f\n", W);
			}
			else
			{
				W = 0;
			}
		}
		else
		{
			if ((Edright >= 1500) && (Edleft >= 1500))
				avoidance_flag = false;
			if ((Edright >= 500) && (Edleft >= 500))
			{
				W = 0;
			}
			if ((Edright < 500) && (Edleft >= 500))
			{
				W = ki * Pid_distY(Edright - 500);
				ROS_INFO("W by Edright is %f\n", W);
			}
			if ((Edleft < 500) && (Edright >= 500))
			{
				W = -ki * Pid_distY(Edleft - 500);
				ROS_INFO("W by Edleft is %f\n", W);
			}
			if ((Edleft < 500) && (Edright < 500))
			{
				//优先避开最近的球,等到最近的球成功避开后，前方就只有另一个球了，接下来避开另一个球
				if (objectAngleDist[0].second <= objectAngleDist[1].second)
					W = ki * Pid_distY(Edright - 500);
				else
					W = -ki * Pid_distY(Edleft - 500);
			}
			//Vx=400;Vy=0;jd=0;
		}
		Vx = 400;
		Vy = 0;
		jd = 0;
	}
	if (0 == returnValue)
	{
		avoidance_flag = false;
	}
	GetNspeed(Vx, Vy, W, jd);
	if (avoidance_flag == false)
		return 1;
	else
		return 0;
}

bool DecisionMaking::ToBallByVision(float Ed, float Er)
//这个函数的比例系数很重要，系数大会有超调，系数小会出现克服不了惯性动不了（特别是停下来调整角度过）
{
	short Vx, Vy, W;
	float jd;
	Er = 90.00 - Er;

	ROS_INFO("ToBallByVision Ed is:%f\n", Ed);
	ROS_INFO("ToBallByVision Er is:%f\n", Er);

	if (fabs(Er) > 2)
		W = 0.2 * Pidlow_angle((float)Er);
	else
		W = 0;
	ROS_INFO("ToBallByVision() W is:%d\n", W);
	Vy = 0;
	jd = 0.0;

	static bool startflag7 = false;
	static bool startflag8 = false;
	static int Nspeedcount = 0;
	Nspeedcount++;
	if (!startflag7 && !startflag8)
	{
		if (Nspeedcount < 3)
			if (Ed > 0)
				Vx = 150;
			else
				Vx = -150;
		else
			startflag7 = true;
	}
	if (startflag7 && !startflag8)
	{
		if (Nspeedcount < 6)
			if (Ed > 0)
				Vx = 220;
			else
				Vx = -220;
		else
			startflag8 = true;
	}
	if (startflag7 && startflag8)
	{
		if (fabs(Ed) > 1800) //注意对Ed分段P控制时，交接处的V不要越变太大。否则较大扭矩损伤电机也影响运动品质。					 //根据决策需要，修改分段距离
		{
			Vx = 360;
			ROS_INFO("when Ed>1800,Vx is %d,Vy is %d\n", Vx, Vy);
		}
		else if ((fabs(Ed) <= 1800) && (fabs(Ed) > 1200)) //进入第一次识球范围，由于视觉扫描周期较长，且不稳定，速度降低些
		{
			Vx = 330;
			ROS_INFO("when 1200<Ed<1800,Vx is %d,Vy is %d\n", Vx, Vy);
		}
		else if ((fabs(Ed) <= 1200) && (fabs(Ed) > 700))
		{
			Vx = 300;
			ROS_INFO("when 700<Ed<1200,Vx is %d,Vy is %d\n", Vx, Vy);
		}
		else if ((fabs(Ed) <= 700) && (fabs(Ed) > 300)) //靠近机械臂持球距离330
		{
			Vx = 270; //210
			ROS_INFO("when 300<Ed<700,Vx is %d,Vy is %d\n", Vx, Vy);
		}
		else if ((fabs(Ed) <= 300) && (fabs(Ed) > 150)) //此时球已经进入机械臂（实际上此时，视觉已经不调整角度，直接读取激光正面距离直线走过去）
		{
			Vx = 240; //180
			ROS_INFO("when 150<Ed<300,Vx is %d,Vy is %d\n", Vx, Vy);
		}
		if ((fabs(Ed) <= 150) && (fabs(Ed) > 100)) //此时距离太小，速度太慢，将系数调大
		{
			Vx = 210; //150
			ROS_INFO("when Ed<150,Vx is %d,Vy is %d\n", Vx, Vy);
		}
		//if((fabs(Ed)<=100)&&(fabs(Ed)>50)) //此时距离太小，速度太慢，将系数调大
		//{
		//	Vx=120;//120
		//	ROS_INFO_STREAM("when Ed<100,Vx is %d,Vy is %d\n",Vx,Vy);
		//}
	}
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
	if ((0 < Ed) && (Ed <= 100)) //（这是主要原因2）这里一定要限定距离停下来（根据情况可能还要调大），因为当距离很小的，哪怕走的很准，X,Y值都误差不大，但此时的偏角						//是等于X，Y与目标点坐标的差值来算的，是会变的很大的
	{
		startflag7 = false;
		startflag8 = false;
		Nspeedcount = 0;
		Stop();
		ROS_INFO_STREAM("ToBallByVision Stop ToBallByVision Stop ToBallByVision Stop ToBallByVision Stop ToBallByVision Stop\n");
		return 1;
	}
	else if (0 == Ed) //走近的过程中突然来了一个0距离，要做处理，这里是在DecisionMaking里完成
	{
		ROS_INFO_STREAM("ToBallByVision 走近的过程中突然来了一个0距离 ！！！\n");
		Stop();
		return 1;
	}
	else
		return 0;
	/*****************PID控制end******************/
}

bool DecisionMaking::ToBallByVision_new()
//这个函数的比例系数很重要，系数大会有超调，系数小会出现克服不了惯性动不了（特别是停下来调整角度）
{
	/*  */
	static bool avoidance_flag = false;
	//static int avoidance_count=0;
	double Edleft, Edright, Edforward; //定义当前位置到目标点的角度偏差、距离
	//double Vx,Vy,W;
	//float jd;

	vector<long> data;
	vector<pair<double, long>> objectAngleDist;
	double ki = 0.03; //pid 比例系数
	//TODO:接受激光数据
	/*
	for(;data.empty();)
	{
		Vision.receive_laser_data(data);
	}
	if(data.size())
	{
		ROS_INFO_STREAM("GET LASER DATA\n");
	}
	else
	{
		ROS_INFO_STREAM("DO NOT GET LASER DATA\n");
	}
	*/
	int returnValue = find_object(data, objectAngleDist);

	if ((-1 == returnValue) || (1 == returnValue) || (2 == returnValue))
	{
		avoidance_flag = true;
		ROS_INFO("ToPostureByAvoidance rightball_angle is:%f\n", objectAngleDist[0].first);
		ROS_INFO("ToPostureByAvoidance rightball_dist is:%d\n", objectAngleDist[0].second);
		Edforward = (objectAngleDist[0].second) * cos(fabs(objectAngleDist[0].first) / 57.3);
		ROS_INFO("Edforward is %f\n", Edforward);
	}
	/*  */
	short Vx, Vy, W;
	float jd;
	float k;
	Edforward = Edforward - 700.00;

	ROS_INFO("ToBallByVision_new Edforward is:%f\n", Edforward);

	Vy = 0;
	W = 0;
	jd = 0.0;
	static bool startflag7 = false;
	static bool startflag8 = false;
	static int Nspeedcount = 0;
	Nspeedcount++;
	if (!startflag7 && !startflag8)
	{
		if (Nspeedcount < 3)
			if (Edforward > 0)
				Vx = 200;
			else
				Vx = -200;
		else
			startflag7 = true;
	}
	if (startflag7 && !startflag8)
	{
		if (Nspeedcount < 6)
			if (Edforward > 0)
				Vx = 240;
			else
				Vx = -240;
		else
			startflag8 = true;
	}
	if (startflag7 && startflag8)
	{
		if (abs(Edforward) > 3000) //注意对Ed分段P控制时，交接处的V不要越变太大。否则较大扭矩损伤电机也影响运动品质。					 //根据决策需要，修改分段距离
		{
			if (Edforward > 0)
				Vx = 300;
			else
				Vx = -300;
			//Vx=700;
			ROS_INFO("when Edforward>3000,Vx is %d,Vy is %d,W is %d\n", Vx, Vy, W);
		}
		else if ((fabs(Edforward) <= 3000) && (fabs(Edforward) > 1800)) //进入第一次识球范围，由于视觉扫描周期较长，且不稳定，速度降低些
		{
			if (Edforward > 0)
				Vx = 270;
			else
				Vx = -270;
			//Vx=600;
			ROS_INFO("when 1800<Edforward<3000,Vx is %d,Vy is %d,W is %d\n", Vx, Vy, W);
		}
		else if ((fabs(Edforward) <= 1800) && (fabs(Edforward) > 500))
		{
			if (Edforward > 0)
				Vx = 240;
			else
				Vx = -240;
			//Vx=500;
			ROS_INFO("when 500<Edforward<1800,Vx is %d,Vy is %d,W is %d\n", Vx, Vy, W);
		}
		else if ((fabs(Edforward) <= 500) && (fabs(Edforward) > 20))
		{
			k = 0.4;
			if (Edforward > 0)
				Vx = k * Edforward;
			else
				Vx = k * Edforward;
			ROS_INFO("when 20<Edforward<500,Vx is %d,Vy is %d,W is %d\n", Vx, Vy, W);
		}
	}
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
	if (fabs(Edforward) <= 50)	 //（这是主要原因2）这里一定要限定距离停下来（根据情况可能还要调大），因为当距离很小的，哪怕走的很准，X,Y值都误差不大，但此时的偏角						//是等于X，Y与目标点坐标的差值来算的，是会变的很大的
	{
		Vx = 0;
		GetNspeed(Vx, Vy, W, jd);
		Control.flag_start_stop = 1; //允许电机转动
		startflag7 = false;
		startflag8 = false;
		Nspeedcount = 0;
		ROS_INFO("ToBallByVision Stop ToBallByVision Stop ToBallByVision Stop ToBallByVision Stop ToBallByVision Stop\n");
		return 1;
	}
	else
		return 0;
	/*****************PID控制end******************/
}

bool DecisionMaking::ToPoleByVision(float Ed, float Er)
//这个函数的比例系数很重要，系数大会有超调，系数小会出现克服不了惯性动不了（特别是停下来调整角度）
{
	short Vx, Vy, W;
	float jd;
	float k;
	Ed = Ed - 2400.00;
	Er = 90.00 - Er;

	ROS_INFO("ToPoleByVision Ed is:%f\n", Ed);
	ROS_INFO("ToPoleByVision Er is:%f\n", Er);

	Vy = 0;
	jd = 0.0;
	static bool startflag7 = false;
	static bool startflag8 = false;
	static int Nspeedcount = 0;
	Nspeedcount++;
	if (!startflag7 && !startflag8)
	{
		W = 0;
		if (Nspeedcount < 3)
			if (Ed > 0)
				Vx = 200;
			else
				Vx = -200;
		else
			startflag7 = true;
	}
	if (startflag7 && !startflag8)
	{
		W = 0;
		if (Nspeedcount < 6)
			if (Ed > 0)
				Vx = 280;
			else
				Vx = -280;
		else
			startflag8 = true;
	}
	if (startflag7 && startflag8)
	{
		if (abs(Ed) > 3000) //注意对Ed分段P控制时，交接处的V不要越变太大。否则较大扭矩损伤电机也影响运动品质。					 //根据决策需要，修改分段距离
		{
			if (fabs(Er) > 3)
				W = 0.2 * Pid_angle_new((float)Er);
			else
			{
				W = 0;
			}
			if (Ed > 0)
				Vx = 360;
			else
				Vx = -360;
			//Vx=700;
			ROS_INFO("when Ed>3000,Vx is %d,Vy is %d,W is %d\n", Vx, Vy, W);
		}
		else if ((abs(Ed) <= 3000) && (abs(Ed) > 1800)) //进入第一次识球范围，由于视觉扫描周期较长，且不稳定，速度降低些
		{
			if (fabs(Er) > 3)
				W = 0.2 * Pid_angle_new((float)Er);
			else
			{
				W = 0;
			}
			if (Ed > 0)
				Vx = 240;
			else
				Vx = -240;
			//Vx=600;
			ROS_INFO("when 1800<Ed<3000,Vx is %d,Vy is %d,W is %d\n", Vx, Vy, W);
		}
		else if ((abs(Ed) <= 1800) && (abs(Ed) > 500))
		{
			if (fabs(Er) > 3)
				W = 0.2 * Pid_angle_new((float)Er);
			else
			{
				W = 0;
			}
			if (Ed > 0)
				Vx = 120;
			else
				Vx = -120;
			//Vx=500;
			ROS_INFO("when 500<Ed<1800,Vx is %d,Vy is %d,W is %d\n", Vx, Vy, W);
		}
		else if ((abs(Ed) <= 500) && (abs(Ed) > 20))
		{
			if (fabs(Er) > 3)
				W = 0.3 * Pid_angle_new((float)Er);
			else
			{
				W = 0;
			}
			k = 0.4;
			if (Ed > 0)
			{
				Vx = k * Ed;
			}
			else
			{
				Vx = k * Ed;
			}
			ROS_INFO("when 20<Ed<500,Vx is %d,Vy is %d,W is %d\n", Vx, Vy, W);
		}
	}
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1;		   //允许电机转动
	if ((fabs(Ed) <= 40) || (Ed == -2401)) //（这是主要原因2）这里一定要限定距离停下来（根据情况可能还要调大），因为当距离很小的，哪怕走的很准，X,Y值都误差不大，但此时的偏角						//是等于X，Y与目标点坐标的差值来算的，是会变的很大的
	{
		W = 0;
		Vx = 0;
		GetNspeed(Vx, Vy, W, jd);
		Control.flag_start_stop = 1; //允许电机转动
		startflag7 = false;
		startflag8 = false;
		Nspeedcount = 0;
		ROS_INFO_STREAM("ToPoleByVision Stop ToPoleByVision Stop ToPoleByVision Stop ToPoleByVision Stop ToPoleByVision Stop\n");
		return 1;
	}
	else
		return 0;
	/*****************PID控制end******************/
}

void DecisionMaking::Stop() //停车，无返回值，即不负责判断是否停下
{
	Control.V1 = 0;
	Control.V2 = 0;
	Control.V3 = 0;
	Control.V4 = 0;
	Control.flag_start_stop = 1;
	ROS_INFO_STREAM("Stop()开环停车，无返回值停车 开环停车，无返回值停车\n");
}

bool DecisionMaking::stop() //停车，有返回值
{
	short Vx, Vy, W;
	float jd;
	Vx = 0;
	Vy = 0;
	jd = 0;
	W = 0;
	//Vx=0.4*pid_distX(-dsp_status.Vx);
	//Vy=0.3*pid_distY(-dsp_status.Vy);
	//W=0.4*pidlow_angle(-DSPW.DW);
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1;
	if ((abs(dsp_status.Vx) < 40)) //使得原地转圈和较大合线速度下停车都适用
	{
		Control.V1 = 0;
		Control.V2 = 0;
		Control.V3 = 0;
		Control.V4 = 0;
		ROS_INFO_STREAM("stop()有返回值停车，有返回值停车，有返回值停车，有返回值停车\n");
		return 1;
	}
	else
		return 0;
}

bool DecisionMaking::stop_new() //停车，有返回值
{
	short Vx, Vy, W;
	float jd;
	Vx = 0;
	Vy = 0;
	jd = 0;
	W = 0;
	//Vx=0.4*pid_distX(-dsp_status.Vx);
	//Vy=0.3*pid_distY(-dsp_status.Vy);
	//W=0.4*pidlow_angle(-DSPW.DW);
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1;
	if ((abs(dsp_status.DW) < 5)) //使得原地转圈和较大合线速度下停车都适用
	{
		Control.V1 = 0;
		Control.V2 = 0;
		Control.V3 = 0;
		Control.V4 = 0;
		ROS_INFO_STREAM("stop()有返回值停车，有返回值停车，有返回值停车，有返回值停车\n");
		return 1;
	}
	else
		return 0;
}
