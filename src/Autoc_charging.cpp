#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string> 
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Int32.h>
#include <pthread.h>
#include <geometry_msgs/TwistStamped.h>
#include <mutex>

/********************************************************
日期：2021.5.7
版本：V 1.0
修改人：H B
功能描述：导航到达充电装附近后，给此节点发送数据，此节点接收到信息后，运行自动充电功能

********************************************************/
//	定义传感器数据与AGV状态结构
struct Charging_DATA_T
{
	float Ultrasonic_data;    
	int left_sensor;
	int right_sensor;
	int movebase_cmd;
	float  battery;
	float power;
	float current;
	int run_state;
}Charging_DATA;

typedef enum  charging_point
{
    not_found = 1,             
    left,                                 
    right,                               
    back,                                
    front,	                      
    stop,	                        
    rotate,
    back_left,	
    back_right,	        	                   
}charging_point_t;


static int Direction = 1;
static bool stop_run = false;

ros::Subscriber Ultrasonic_Sub; 
ros::Subscriber InfraRed_Sub; 	
ros::Subscriber MoveBase_Sub;
ros::Subscriber Charging_OK_Sub; 	
ros::Publisher mCmdvelPub_;

std_msgs::Float32MultiArray motor_control; 

uint8_t Charging_running = 0;
uint8_t speed_running = 0;
uint8_t Charging_tip_run = 0;
charging_point_t dir;
pthread_mutex_t mutex;

bool Charging_ok= false;
static int outime[2] ={0};  

//语音文件路径
const char *mplayer[] = {"mplayer /opt/ros/melodic/share/automatic_charging/voice/"," < /dev/null > /dev/null 2>power.log  &"};
const char *voise_tip[]={"power.wav","power_full.wav","power_low.mp3"};

/*
功能:  	电源的超声波（后超声波）数据获取
备注:	ros回调
*/
void Ultrasonic_data_Callback(const std_msgs::Float32MultiArray::ConstPtr& data)
{
	static int count = 0;
	Charging_DATA.battery = data->data[0];    //电池电压
	Charging_DATA.current = data->data[1];    //充电电流

    if(data->data[1] > 0.020)
	{ 
		count = 100;
		Charging_DATA.power = 55.0;
	}
	else 
	{
		count--;
		if(count < 0)
		{
			Charging_DATA.power = 0.0012;
			count = -1;
		}
				
	}
	Charging_DATA.Ultrasonic_data  = data->data[2];
}

// 红外数据读取回调函数
void InfraRed_data_Callback(const std_msgs::UInt32MultiArray::ConstPtr& msg)
{
	Charging_DATA.left_sensor  = msg->data[0];  			//左红外	
	Charging_DATA.right_sensor = msg->data[1]; 			 //右红外
}

// 手动控制自动回充回调函数
void MoveBase_Control_Callback(const std_msgs::Int32::ConstPtr& cmd)
{
	Charging_DATA.movebase_cmd = cmd->data;
}

/*
	初始位置正确 
	前进只需要调整左右方向
*/
void Direction_control(charging_point_t direction)
{
    geometry_msgs::Twist current_vel;
	
	switch(direction)
	{
		case left:
			        current_vel.linear.x = -0.06;
        			current_vel.linear.y = 0;
        			current_vel.linear.z = 0;
        			current_vel.angular.x = 0;
        			current_vel.angular.y = 0;
        			current_vel.angular.z = -0.10;
					break;
		case right:
			        current_vel.linear.x = -0.06;
        			current_vel.linear.y = 0;
        			current_vel.linear.z = 0;
        			current_vel.angular.x = 0;
        			current_vel.angular.y = 0;
        			current_vel.angular.z = 0.06;
					break;
		case back:
			        current_vel.linear.x = 0.1;
        			current_vel.linear.y = 0;
        			current_vel.linear.z = 0;
        			current_vel.angular.x = 0;
        			current_vel.angular.y = 0;
        			current_vel.angular.z = 0;
					break;
		case front:
			        current_vel.linear.x = -0.07;
        			current_vel.linear.y = 0;
        			current_vel.linear.z = 0;
        			current_vel.angular.x = 0;
        			current_vel.angular.y = 0;
        			current_vel.angular.z = 0;
					break;
		case stop:
			        current_vel.linear.x = 0;
        			current_vel.linear.y = 0;
        			current_vel.linear.z = 0;
        			current_vel.angular.x = 0;
        			current_vel.angular.y = 0;
        			current_vel.angular.z = 0;		
					break;
		case rotate:
			        current_vel.linear.x = 0.0;
        			current_vel.linear.y = 0;
        			current_vel.linear.z = 0;
        			current_vel.angular.x = 0;
        			current_vel.angular.y = 0;
        			current_vel.angular.z = 0.1;		
					break;
		case back_left:
			        current_vel.linear.x = 0.06;
        			current_vel.linear.y = 0;
        			current_vel.linear.z = 0;
        			current_vel.angular.x = 0;
        			current_vel.angular.y = 0;
        			current_vel.angular.z = 0.08;		
					break;
		case back_right:
			        current_vel.linear.x = 0.06;
        			current_vel.linear.y = 0;
        			current_vel.linear.z = 0;
        			current_vel.angular.x = 0;
        			current_vel.angular.y = 0;
        			current_vel.angular.z = -0.08;		
					break;
		default: 
					break;
	}
        mCmdvelPub_.publish(current_vel);
}

/*
功能:  	开启线程，循环判断AGV在充电桩的位置，调整AGV方位
备注:	1:为左边的信号，2:接收到充电庄的右边信号 3:为同时受到两个探头信号
*/
void* Charging_Getstate(void *parameter)
{
	pthread_detach(pthread_self()); 
	static int count = 0;	

	while(Charging_running)
	{	
		pthread_mutex_lock(&mutex);	//	互斥锁，保护共享资源
		//	未收到红外线信号
		if((Charging_DATA.left_sensor == 0) && (Charging_DATA.right_sensor == 0))		        				//未收到信号
            Direction = not_found;   
		//	左探头收到左边数据，但右探头收不到红外数据          
 		else if((Charging_DATA.left_sensor== 1) && (Charging_DATA.right_sensor== 0)) 		   				 
            Direction = right;	//	机器人右转调整标志
 //		else if((Charging_DATA.left_sensor== 2) && (Charging_DATA.right_sensor== 0)) 		    			
 //                       Direction = left; 
		//	左探头收到两边信号，右探头接收不到数据
 		else if((Charging_DATA.left_sensor== 3) && (Charging_DATA.right_sensor== 0)) 		    			
            Direction = right;	//	机器人右转调整标志
		//	有一个收到两边红外，但仍不是正对
 		else if((Charging_DATA.left_sensor== 3) && (Charging_DATA.right_sensor== 1)) 		    			
            Direction = right;  //	机器人右转调整标志
		//	左探头接收不到数据，右探头接收到左红外的数据 
 		else if((Charging_DATA.left_sensor== 0) && (Charging_DATA.right_sensor== 2)) 		    			
            Direction = left;	//	机器人左转调整标志
		//	左探头接收不到数据，右探头接收到右红外的数据
 		else if((Charging_DATA.left_sensor== 0) && (Charging_DATA.right_sensor== 3)) 		    			
            Direction = left;	//	机器人左转调整标志
		//	右探头接收到两边右红外的数据，但左探头收到右探头的红外数据
 		else if((Charging_DATA.left_sensor== 2) && (Charging_DATA.right_sensor== 3)) 		    			
            Direction = left;	//	机器人左转调整标志
		//	两个探头都接受到两个红外数据，说明车子正对充电桩
		else if((Charging_DATA.left_sensor== 3) && (Charging_DATA.right_sensor== 3)) 		 
			Direction = front;	 //车子在充电桩正中间
										                                                                                      
		pthread_mutex_unlock(&mutex);	//解锁
		
		ros::spinOnce();//循环等待回调函数
	}

	pthread_exit(NULL);

}

/*
功能:  	开启线程，速度控制
备注:	速度控制
*/
void *speed_contorl(void *parameter)
{

	pthread_detach(pthread_self()); 
	while(speed_running)
	{
		pthread_mutex_lock(&mutex);
		Direction_control(dir);	
		pthread_mutex_unlock(&mutex);
		usleep(1000*20);
	}

	pthread_exit(NULL);
}

/*
功能:  	电源的电压均值滤波，确定是否有电源接入
备注:	滤波
*/
int Charging_DATA_filter(int timer)
{
	int result = 0; 
	int temp = timer;
	while(temp--)
	{
		if (Charging_DATA.power > 50)
			result ++;
		
		if(result > (timer/2) )
			return 1;

		usleep(1000*100);
	}

	if(result < (timer/3))
		return 0;
	else 
		return 1;
}

/*
功能:  	开启线程，读取AGV充电状态，有电源接入即可语音提示
备注:	充电检测
*/
void* Charging_tip_thread(void *parameter)
{
	static bool STATE = false;	
	pthread_detach(pthread_self()); 
	outime[0] = 0;
	outime[1] = 0;		

	while(Charging_tip_run)
	{
		//pthread_mutex_lock(&mutex);
		//防撞条与超声波反馈
		//printf("data[1]:%f data[2]:%d\n",Charging_DATA.Ultrasonic_data,Charging_DATA.Power_Msg[0]);
		//printf("outime[0]: %d outime[1]: %d\n",outime[0],outime[1]);
		if((Charging_DATA.Ultrasonic_data < 85) && (Charging_DATA.movebase_cmd))
		{
			if(Charging_DATA.power < 50.0)	//	超声波反馈，可加入防撞条数据
			{
				outime[0]++;		//	超时处理
				usleep(1000*100);
				if(outime[0] < 30)
				   stop_run = true;
				else
				   stop_run = false;	
			}			
		}
		//printf("Ultrasonic_data:%f  Charging_DATA.power :%f \r\n",Charging_DATA.Ultrasonic_data ,Charging_DATA.power );
	    if((Charging_DATA.Ultrasonic_data  < 100.0) && (Charging_DATA.power >50.0))	 //	检测到电流接入
        {
			if(STATE)		 //	未充电状态   
			{
				//超声波距离 前进
				//充电完成  当前剩余电量，预计充电完成时间提示 			
				printf("开始充电\r\n");

				// 播放充电完成语音提示
				char power_tip[200] = {0};
				strcpy(power_tip,mplayer[0]);
				strcat(power_tip,voise_tip[0]);
				strcat(power_tip,mplayer[1]);
				system(power_tip);

				Charging_DATA.movebase_cmd = 0;
				outime[0] = 0;
				Charging_ok = true;
				stop_run = false;
				STATE = false;
			}		
			
        }
		else  if ((Charging_DATA.Ultrasonic_data > 150) && (Charging_DATA.power <10))	//未充电
		{
			STATE = true;
			Charging_ok = false;
		}
		//pthread_mutex_unlock(&mutex); 	
	}
	
}

/*自动充电主程序*/
void Charging_process_task(void)
{	 
	static int turn_dir = 0;	
	int index_dir = 0;
	static int nor_dir = 0;       
  	// ros::Rate loop_rate(100);

	while(Charging_DATA.movebase_cmd)
	{
		//if(Charging_DATA.InfraRed_data[0] || Charging_DATA.InfraRed_data[1])
		//printf("data[0] = %#x data[1] = %#x Ultrasonic_data = %f\n ",Charging_DATA.InfraRed_data[0],Charging_DATA.InfraRed_data[1],Charging_DATA.Ultrasonic_data );		
		usleep(1000*10);
		//printf("outime[0]: %d outime[1]: %d\n",outime[0],outime[1]);
		if(stop_run)
		{
			usleep(1000*10);
			dir = stop;
			//printf("STOP\n");				
			continue;		
		}

		//printf("Ultrasonic_data: %f\n",Charging_DATA.Ultrasonic_data);
		//printf("Charging_DATA.Power_Msg[0]: %d\n",Charging_DATA.Power_Msg[0]);
		//printf("outime[0]: %d outime[1]: %d\n",outime[0],outime[1]);		
		if((outime[0] >= 30) || (outime[1] > 500))	//返回重新尝试 或防撞条触发时
		{
			while(Charging_DATA.Ultrasonic_data < 650 && Charging_DATA.movebase_cmd)	//超时后退，可换为走里程计
			{	
				// 接收不到红外信号时
				if(Direction == not_found)      //没有检测到数据时
			    {
					turn_dir++;			
					if(turn_dir % 100 == 0)
					{	
						if(Charging_DATA.Ultrasonic_data  > 500)
						{
							// 转动
							dir = rotate;
							turn_dir = 0;
						}

					}

				}
				// AGV在充电桩左边时
				else if(Direction == left)
				{
					dir =  back_right;
					turn_dir = 0;
					usleep(1000*20);
					//printf("退右\n");
				}
				// AGV在充电桩右边时
				else if(Direction == right)
				{  
				    dir = back_left;     
					turn_dir = 0;
					//printf("退左\n");
					usleep(1000*20);
				}
				else                           //直行
				{
				    dir = back;
					//printf("直退\n");
					usleep(1000*50);
					turn_dir = 0;
				}

				//printf("Ultrasonic_data: %d\n",Charging_DATA.Ultrasonic_data);
				usleep(1000*10);
				ros::spinOnce();//循环等待回调函数
			}

			outime[0] = 0;
			outime[1] = 0;
			
		}	

		//pthread_mutex_lock(&mutex);
	    if(Direction == not_found)                      //没有检测到数据时
        {
			// printf("00000000\n");
			turn_dir++;			
			if(turn_dir % 100 == 0)
			{	
				if(Charging_DATA.Ultrasonic_data  > 500)
				{
					dir = rotate;
					turn_dir = 0;
				}

			}
	
        }
		else if(Direction == left)
        {
			dir = left ;
            index_dir = 1;
			//Direction_control(dir);
			//printf("左\n");
			turn_dir = 0;
			usleep(1000*25);
        }
		else if(Direction == right)
        {
			dir = right;
			index_dir = 2;
			//Direction_control(dir);
			//printf("右\n");
			turn_dir = 0;
			usleep(1000*10);
        }
        else              //	直行
        {
			dir = front;
			//Direction_control(dir); 
			index_dir = 3;
			//if(Direction == front)
			usleep(1000*60);
			//printf("直行\n");
			turn_dir = 0;	
			nor_dir = 1; 
        }
		//pthread_mutex_unlock(&mutex); 
agin:
//		printf("dir :%d index_dir:%d\n",Direction,index_dir);
		//usleep(1000*500);		
		//Charging_DATA.InfraRed_data[0] = 0x55;
		//Charging_DATA.InfraRed_data[1] = 0x55;
		
		//loop_rate.sleep();
		ros::spinOnce();//循环等待回调函数

	}
	 dir = stop;	
	 stop_run = false;
}

/*
功能:  	电源的电压均值滤波，判断是否充满电
备注:	滤波
*/
float power_filter(void)
{
	int i = 10;
	float power = 0.0;

	while(i--)
	{
		power += Charging_DATA.battery;
		sleep(1);
	}
	printf("\r-----------正在充电中，当前电量为:%.2fV-----------",power/10.0);
	fflush(stdout);
	return power/10.0;
}


/*1. 订阅超声波数据  
  2. 订阅红外接收数据
  3. 向电机话题发送数据
  4. 订阅导航控制话题
	-->执行自动充电功能
  5.充电成功状态

*/
int main(int argc,char **argv)
{
	int i = 0;
    int curr_low,power_low,power_full;
	pthread_t 	Charging_thread,speed,tip_thread;
	Charging_running = 1;
	speed_running = 1;
	Charging_tip_run = 1;

	ros::init(argc, argv, "Automatic_Charging_NODE");
	ros::NodeHandle n;		//创建句柄
	ros::NodeHandle nh("~");

	nh.param<int>("power_low", power_low, 45.6);	//	低电量阈值
	nh.param<int>("curr_low", curr_low, 45.5);		//	当前电量值
	nh.param<int>("power_full", power_full, 53.8);	//	充满电量值

	Ultrasonic_Sub = n.subscribe("/Power_And_Distance", 10, &Ultrasonic_data_Callback);	//	订阅后超声波值话题
	InfraRed_Sub = n.subscribe("/Ir_Sensor_Data", 10, &InfraRed_data_Callback);			//	订阅红外线值话题
	MoveBase_Sub = n.subscribe("/Automatic_Charging_CMD_Topic", 10, &MoveBase_Control_Callback);	//	订阅自动回充指令话题

	mCmdvelPub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);			//1 缓冲区

	// 创建AGV相对于充电桩方向线程
	pthread_create(&Charging_thread, NULL, Charging_Getstate, NULL);

	// 创建充电状态线程
	pthread_create(&tip_thread, NULL, Charging_tip_thread, NULL);		

	// 	AGV方向调整
	pthread_create(&speed, NULL, speed_contorl, NULL);

	Charging_DATA.movebase_cmd = 0;
	Charging_DATA.power = 0;
	Charging_DATA.current = 0;
	Charging_DATA.Ultrasonic_data = 300;
	ros::Rate loop_rate(500);

    if(curr_low < power_low)
	{
		// 播放语音提示音频
		char power_tip[200] = {0};
        strcpy(power_tip, mplayer[0]); 
        strcat(power_tip, voise_tip[2]);
        strcat(power_tip, mplayer[1]);
		system(power_tip);
		// 启动自动回充
		Charging_DATA.movebase_cmd = 1;
	}

	while(ros::ok())
	{
		Charging_process_task();							
        if(Charging_ok)
        {
			if(power_filter() > power_full)		//%96 以后充电过慢 不使用
			{
				//i++;
				//if(i>10)
				//{
				//printf("Charging_DATA.Power_Msg[1]:%d\n",Charging_DATA.Power_Msg[1]);
				//充电完成  当前剩余电量，预计充电完成时间提示 			
		        char power_tip[200] = {0};
				// 充电完成语音播报
		        strcpy(power_tip,mplayer[0]);
		        strcat(power_tip,voise_tip[1]);
		        strcat(power_tip,mplayer[1]);
		        system(power_tip);
				
				dir = back;
				sleep(2);
				dir = stop;
		        Charging_ok = false;
				printf("-----------充电完成！-----------\n");
				//}

            }
        }
		loop_rate.sleep();
		ros::spinOnce();//循环等待回调函数		
		//ros::spin();
	}

	Charging_running = 0;
	speed_running = 0;
	Charging_tip_run = 0;

	return 0;
}



