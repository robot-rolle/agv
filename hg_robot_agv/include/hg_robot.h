/*
 * @Author: robot_luoer@163.com robot_luoer@163.com
 * @Date: 2022-07-14 16:55:29
 * @LastEditors: robot_luoer@163.com robot_luoer@163.com
 * @LastEditTime: 2022-08-10 12:24:50
 * @FilePath: /hgfirst/src/hg_robot_agv/include/hg_robot.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __HG_ROBOT_H_
#define __HG_ROBOT_H_

#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h>
#include <stdlib.h>    
#include <unistd.h>      
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
using namespace std;
//Macro definition
//宏定义
#define limit(a,b,c)   (b>a?(b>c?c:b):a)   // b 输入  限制在 a c
#define SEND_DATA_CHECK   	  1          //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   	  0          //Receive data to check flag bits //接收数据校验标志位

#define FRAME_HEADER_A     	  0X13       //Frame head_a//帧头A
#define FRAME_HEADER_B		  0X14		 //Frame head_b

#define FRAME_TAIL_A		  0X66		 //Frame tail_a
#define FRAME_TAIL_B          0X0A       //Frame tail_b//帧尾

#define RECEIVE_DATA_LENGTH_A   31		 //第一个数据包长度
#define RECEIVE_DATA_LENGTH_B 	79		 //第二个数据包长度
#define RECEIVE_DATA_LENGTH_C   33		 //第三个数据包长度

#define WHEELLENGTH           	0.155f		//单个车轮直径
#define WHEELWIDTH              0.300f		//两个车轮距离 
#define WHEEL_LENGTH			0.180f		//车轮和万向轮 之间距离 
#define WHEEL_LENGTH_L			0.250f	    //车长 0.25 总长0.5



#define RECEIVE_DATA_SIZE_ESP 88        //The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define RECEIVE_DATA_SIZE_UWB 100		//  缓存区

#define RECEIVE_SIZE_ESP   10		//the length of receive接受数据长度 
#define SEND_DATA_SIZE    11         //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度    暂时没用
#define PI 				  3.1415926f //PI //圆周率

//Relative to the range set by the IMU gyroscope, the range is ±2000°, corresponding data range is ±32768
//The gyroscope raw data is converted in radian (rad) units, 1/16.38/57.30=0.00106576
//与IMU陀螺仪设置的量程有关，量程±2000°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位，1/16.38/57.30=0.00106576
// #define GYROSCOPE_PARAM   0.00106576 
#define GYROSCOPE_PARAM   (0.00026644f / 2) 
//Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is ±32768
//Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=208.98
//与IMU加速度计设置的量程有关，量程±16g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/16g=32768/156.8=208.98
// #define ACCEl_PARAM	  208.98f
#define ACCEl_PARAM	    1671.84f
extern sensor_msgs::Imu Gy95t; //External variables, IMU topic data //外部变量，IMU话题数据

//Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
//协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3,	 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0, 
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };
										      
const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9} ;
typedef struct
{
  float powerVoltage_Batter;         //电池电压
  float current_Batter;              //电池电流
  float current_5VOut0;              // 5Vout0电流
  float current_5VOut1;              // 5Vout1电流(接树莓派)
  float current_Chassis;             //底盘电流
  unsigned char powerSwitch_Master;  //总开关状态
  unsigned char powerSwitch_Chassis; //底盘开关状态
  unsigned char powerSwitch_5VOut1;  // 5VOut1(接树莓派)
  unsigned char powerSwitch_5VOut0;  // 5VOut0
} RobotData0;                        //电源板 24 字节

typedef struct
{
 
  float acc[3];          //加速度
  float gyro[3];         //角速度
  float mag[3];          //磁力计 
  float motorSpeeds[2];  //电机速度
  float ultrasonicB;     //底部 超声波数据(单位m)
  float xC;          //当前坐标x (单位:m)
  float yC;          //当前坐标y
  float speed;       //中心速度(单位:m/s)
  float speedRadian; //中心速度的弧度角,以起始位置为坐标系(单位:rad)
  float radian;      //当前的弧度角
  float radianSpeed; //弧度角速度  
} RobotData1;            //控制板 

typedef struct
{
  float ultrasonicL[3];        //L超声波数据(单位m)
  float ultrasonicR[3];        //R超声波数据(单位m)
  unsigned char crashStrip[2]; //防撞条
} RobotData2;                  //信号板 32 字节

typedef struct _ESP_DATA_
{
  RobotData0 data0;
  RobotData1 data1;
  RobotData2 data2;
}ESP_DATA;

//The structure in which the lower computer sends data to the ROS
//下位机向ROS发送数据的结构体
typedef struct _RECEIVE_DATA_     
{
	    uint8_t Flag_Stop;		
		uint8_t  rxesp[RECEIVE_DATA_SIZE_ESP];	//缓存所有数据
		uint8_t rxdata0[RECEIVE_DATA_LENGTH_A-7];//数据包1
		uint8_t rxdata1[RECEIVE_DATA_LENGTH_B-7];//数据包2
		uint8_t rxdata2[RECEIVE_DATA_LENGTH_C-7];//数据包3
		short Frame_Header;  //头
		float X_speed;   //速度
		float Y_speed;  	
		float Power_Voltage;	
		short Frame_Tail;
		int length;
}RECEIVE_DATA;
//The structure of the ROS to send data to the down machine
//ROS向下位机发送数据的结构体 SpeedLR:%f,%f,\n
typedef struct _SEND_DATA_  
{
	    uint8_t tx[SEND_DATA_SIZE];
		unsigned char *txesp[40];
		float X_speed;	       
		float Y_speed;           
		float Z_speed;         
		unsigned char Frame_Tail='\n'; 
}SEND_DATA;   //暂时没有使用

typedef struct _RECEIVE_DATA_UWB_
{
		 uint8_t rx;
	     char rxuwb[RECEIVE_DATA_SIZE_UWB];
		 uint8_t   FLAG_RECEIVE;
		 uint8_t   SUM_RECEIVE;
		 int       length;
		 float	   X_Position;
		 float     Y_Position;
		 float     Z_Position;
}RECEIVE_DATA_UWB;

//Data structure for speed and position
//速度、位置数据结构体
typedef struct __Vel_Pos_Data_
{
	float X;		//位置信息x
	float Y;		//位置信息y
	float Z;		//位置信息z
	float vl;		//左边速度
	float vr; 		//右边速度
	float vc;		//车中心速度 线速度
	float w;  		//角速度
	double t;		//时间积分量
	float Angle;	//当前角度
}Vel_Pos_Data;

class launch_agv
{
	public:
		launch_agv();
		~launch_agv();
		void Task();
		serial::Serial Esp32_Serial;
		serial::Serial Uwb_Serial;
	private:
		ros::NodeHandle n;
		//Time dependent, used for integration to find displacement (mileage) //时间相关，用于积分求位移(里程)
		ros::Time  _Now, _Last_Time;
		ros::Time  _Speed_Now, _Speed_Tlast, _Error_TimeL, _Error_TimeR;
		//Sampling time, used for integration to find displacement (mileage) //采样时间，用于积分求位移(里程)
		float Sampling_Time;
		//Initialize the topic subscriber //初始化话题订阅者
		ros::Subscriber Cmd_Vel_Sub;
		//initalize the timer1初始化定时器。 测试使用
		ros::Timer timer1;
		// speed topic receiver function
		void Cmd_Vel_Callback(const geometry_msgs::Twist  &twist_aux);
		void time_test(const ros::TimerEvent & event);
		//Initialize the topic publisher //初始化话题发布者
		ros::Publisher odom_publisher, imu_publisher,voltage_publisher,uwb_publisher;
		void Publish_Odom(); 		//Pub the speedometer topic //发布里程计话题
		void Publish_Imu(); 		//Pub the IMU sensor topic //发布IMU传感器话题
		void Publish_Voltage();		//Pub the power supply voltage topic //发布电源电压话题
		void Publish_Uwb(); 		//pub the uwb   topic
	//从串口(ttyUSB)读取运动底盘速度、IMU、电源电压数据
        //Read motion chassis speed, IMU, power supply voltage data from serial port (ttyUSB)
	int Get_Sensor_Esp();
	bool Get_Sensor_Uwb();
	//CRC16 check function //CRC校验函数
	unsigned short Check_Sum_CRC16(unsigned char *Data, unsigned int length,int mode); 

	//IMU data conversion read //IMU数据转化读取
	void Position_Check(); //
	void Position_Trans();


	string usart_port_name, usart_port_name_uwb,robot_frame_id, gyro_frame_id, odom_frame_id; //Define the related variables //定义相关变量
	int serial_baud_rate,timeout_esp;      //Serial communication baud rate //串口通信波特率		1
	int serial_baud_rate_uwb,timeout_uwb;			//												1



	RECEIVE_DATA_UWB Receive_Data_Uwb;//The serial port receives the data structure of the uwb//uwb 串口接受结构体		1
	RECEIVE_DATA   Receive_Data_Esp;  //The serial port receives the data structure //串口接收数据结构体			1

	SEND_DATA      Send_Data_Esp;       //The serial port sends the data structure //串口发送数据结构体      1  暂时没有用

	Vel_Pos_Data    Robot_Pos;    //The position of the robot //机器人的位置			1
	Vel_Pos_Data    Robot_Pos_Uwb;   // 					0 
   	Vel_Pos_Data    Robot_Vel;    //The speed of the robot //机器人的速度  //             1

	ESP_DATA   	Esp_Origin_Data;  //the origin data  from the esp data //机器人底盘原始数据  1

	uint8_t *databuffer0 =(uint8_t *)(&Esp_Origin_Data.data0);	// 数据地址1         1
	uint8_t *databuffer1 = (uint8_t *)(&Esp_Origin_Data.data1);	//数据地址2			 1
	uint8_t *databuffer2 = (uint8_t *)(&Esp_Origin_Data.data2);	//数据地址2			 1

    float Power_voltage;       //Power supply voltage //电源电压	 				1

};


#endif