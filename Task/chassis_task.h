/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "bsp_adc.h"


#define SQRT2 0.707106f
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//是否有电容 1：有    0：无
#define POWER_CAP 1

//切换溪地、安合电容
#define XIDI 0

#define CHASSIS_1_TURN_MOTOR_OFFSET_ECD 718
#define CHASSIS_2_TURN_MOTOR_OFFSET_ECD 6026
#define CHASSIS_3_TURN_MOTOR_OFFSET_ECD 1239
#define CHASSIS_4_TURN_MOTOR_OFFSET_ECD 7803

//底盘3508PID参数定义
//电机速度PID
#define CHASSIS_MOTOR_SPEED_KP 8.0f//8
#define CHASSIS_MOTOR_SPEED_KI 0.0f
#define CHASSIS_MOTOR_SPEED_KD 0.0f
#define CHASSIS_MOTOR_SPEED_MAX_OUT 14000.0f//8000
#define CHASSIS_MOTOR_SPEED_MAX_IOUT 0.0f

//底盘6020PID参数定义
//角度PID
#define CHASSIS_TURN_MOTOR_ANGLE_KP 800.0f//800
#define CHASSIS_TURN_MOTOR_ANGLE_KI 0.0f
#define CHASSIS_TURN_MOTOR_ANGLE_KD 0.0f
#define CHASSIS_TURN_MOTOR_ANGLE_MAX_OUT 300.0f
#define CHASSIS_TURN_MOTOR_ANGLE_MAX_IOUT 0.0f

//速度PID
#define CHASSIS_TURN_MOTOR_SPEED_KP 150.0f//150
#define CHASSIS_TURN_MOTOR_SPEED_KI 0.0f
#define CHASSIS_TURN_MOTOR_SPEED_KD 0.0f
#define CHASSIS_TURN_MOTOR_SPEED_MAX_OUT 16000.0f
#define CHASSIS_TURN_MOTOR_SPEED_MAX_IOUT 0.0f

//底盘跟随云台旋转PID
#define CHASSIS_ANGLE_KP 12.1f//12.1
#define CHASSIS_ANGLE_KI 0.0f
#define CHASSIS_ANGLE_KD 0.0f
#define CHASSIS_ANGLE_MAX_OUT 15.0f
#define CHASSIS_ANGLE_MAX_IOUT 0.0f//0.2

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

//底盘中心与轮子距离
#define MOTOR_DISTANCE_TO_CENTER 0.23f // 中心到轮子距离是20cm
//轮子半径
#define WHEEL_RADIUS 0.6f // 6也行，也就是1/R的值
#define WHEEL_PERIMETER (2*PI* WHEEL_RADIUS) //舵轮周长
#define M3508_RATIO 19 //3508电机减速比

//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f

// m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 40.0f

//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 7.0f//15//1.7
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 7.0f//15//1.7
//底盘小陀螺速度
#define CHASSIS_TOP_SPEED 10.0f
//底盘小陀螺时最大平移速度
#define TOP_MAX_CHASSIS_SPEED 8.0f

#define CHASSIS_WZ_SET_SCALE 0.1f

typedef enum
{
  CHASSIS_ZERO_FORCE = 0,   //底盘无力, 跟没上电那样
  CHASSIS_NO_MOVE,          //底盘保持不动
  CHASSIS_FOLLOW_GIMBAL,    //底盘跟随云台旋转
  CHASSIS_TOP,              //底盘小陀螺模式
  CHASSIS_NO_FOLLOW_GIMBAL, //底盘不跟随云台旋转
  CHASSIS_OPEN              //遥控器的值乘以比例成电流值 直接发送到can总线上
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  float relative_angle; //6020相对角度解算
  float relative_angle_set; //6020相对角度目标
  uint16_t offset_ecd;//中值计算
  float accel;          //加速度
  float speed;          //当前速度
  float speed_rpm_set;      //设定速度
  int16_t give_current; //发送电流
	float give_current_now;
} chassis_motor_t;

typedef struct
{
  float kp;
  float ki;
  float kd;

  float set;
  float get;
  float err;

  float max_out;
  float max_iout;

  float Pout;
  float Iout;
  float Dout;

  float out;
} chassis_turn_PID_t;

typedef struct
{
  const gimbal_motor_t *chassis_yaw_motor;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角.
  const gimbal_motor_t *chassis_pitch_motor; //底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
  const float *chassis_INS_angle;            //获取陀螺仪解算出的欧拉角指针
  chassis_mode_e chassis_mode;               //底盘控制状态机
  chassis_mode_e chassis_last_mode;
  chassis_motor_t motor_chassis[4];          //底盘3508电机数据
  chassis_motor_t motor_chassis_turn[4];          //底盘6020电机数据
  pid_type_def motor_speed_pid[4];           //底盘电机速度pid
  pid_type_def motor_turn_speed_pid[4];           //底盘电机速度pid
  pid_type_def motor_turn_angle_pid[4];           //底盘电机角度pid
  pid_type_def chassis_angle_pid;            //底盘跟随角度pid
  pid_type_def chassis_power_pid;   //功率限制pid

  first_order_filter_type_t chassis_cmd_slow_set_vx; //使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy; //使用一阶低通滤波减缓设定值
	

  float vx;                         //底盘速度 前进方向 前为正，单位 m/s
  float vy;                         //底盘速度 左右方向 左为正  单位 m/s
  float wz;                         //底盘旋转角速度，逆时针为正 单位 rad/s
  float vx_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
  float vy_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
  float wz_set;                     //底盘设定旋转角速度，逆时针为正 单位 rad/s
  float chassis_relative_angle;     //底盘与云台的相对角度，单位 rad
  float chassis_relative_angle_set; //设置相对云台控制角度
  float chassis_yaw_set;

  float vx_max_speed;  //前进方向最大速度 单位m/s
  float vx_min_speed;  //后退方向最大速度 单位m/s
  float vy_max_speed;  //左方向最大速度 单位m/s
  float vy_min_speed;  //右方向最大速度 单位m/s
  float top_max_speed; //小陀螺平移正方向最大速度
  float top_min_speed; //小陀螺平移反方向最大速度
  float chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
  float chassis_pitch; //陀螺仪和云台电机叠加的pitch角度
  float chassis_roll;  //陀螺仪和云台电机叠加的roll角度

  float chassis_current_double;//转速平方和，做功率拟合用
  float chassis_current_and_rpm;//电流转速乘积和，做功率拟合用


  float chassis_power;//底盘当前功率

  uint16_t chassis_power_buffer;//底盘剩余缓冲功率
  uint16_t chassis_power_limit;//底盘能量限制
	uint16_t chassis_output;//裁判系统底盘输入，输入24v为1，不输入为0

  float chassis_3508_power_set;//底盘功率设定值
	float chassis_6020_power_set;//底盘功率设定值

  float chassis_3508_rpm_double;//3508转速平方和
  float predict_send_power_3508;//功率预测值
	float predict_send_power_6020;//功率预测值

  float limit_k_3508;
	float limit_k_6020;

  float k_3508[4];
  float m_3508[4];
  float k_6020[4];
  float m_6020[4];
} chassis_move_t;

/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
extern void chassis_task(void const *pvParameters);

#endif
