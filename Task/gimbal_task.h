/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

/* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"

/* ******************************************************************
                        AGV_CHASIS
****************************************************************** */
//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 201
//��̨������Ƽ�� 1ms
#define GIMBAL_CONTROL_TIME 1

// yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL LEFT_SWITCH

//��ͷ��̨�ٶ�
#define TURN_SPEED 0.04f
//���԰�����δʹ��
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define RC_DEADBAND 50//10

//������
#define YAW_RC_SEN -0.000005f
#define PITCH_RC_SEN -0.000005f // 0.005
#define YAW_MOUSE_SEN 0.00007f
//0.00005
#define PITCH_MOUSE_SEN 0.00012f
#define PITCH_MOUSE_SEN_SMALL 0.00003f

//0.00015
#define YAW_ENCODE_SEN 0.01f
#define PITCH_ENCODE_SEN 0.015f

//�ж�ң�����������ʱ���Լ�ң�����������жϣ�������̨yaw����ֵ�Է�������Ư��
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

//����������ǰ�װ����
#define PITCH_TURN 1
#define PITCH_OUT_TURN 1
#define YAW_TURN 0
#define YAW_OUT_TURN 0
#define YAW_GYRO_TURN 1

// PITCH��̨�ǶȻ�PID����
#define PITCH_ABSOLUTE_ANGLE_KP 28.0f//22.0f//20.0f//12.0f
#define PITCH_ABSOLUTE_ANGLE_KI 0.03f//0.02f
#define PITCH_ABSOLUTE_ANGLE_KD 0.20f//0.16f//0.1f//0.03f//0.0f
#define PITCH_ABSOLUTE_ANGLE_MAX_OUT 20.0f
#define PITCH_ABSOLUTE_ANGLE_MAX_IOUT 0.2f
//PITCH��̨�ٶȻ�PID����
#define PITCH_GYRO_KP 8000.0f
#define PITCH_GYRO_KI 15.0f
#define PITCH_GYRO_KD 0.0f
#define PITCH_GYRO_MAX_OUT 25000.0f
#define PITCH_GYRO_MAX_IOUT 2000.0f

// YAW��̨�ǶȻ�PID����
#define YAW_ABSOLUTE_ANGLE_KP 12.0f//12
#define YAW_ABSOLUTE_ANGLE_KI 0.0f
#define YAW_ABSOLUTE_ANGLE_KD 0.02f//0.09
//YAW��̨С����״̬�ǶȻ�PID����
#define YAW_ABSOLUTE_ANGLE_TOP_KP 9.0f
#define YAW_ABSOLUTE_ANGLE_TOP_KI 0.0f
#define YAW_ABSOLUTE_ANGLE_TOP_KD 0.0f

#define YAW_ABSOLUTE_ANGLE_MAX_OUT 20.0f
#define YAW_ABSOLUTE_ANGLE_MAX_IOUT 0.6f

//YAW��̨�ٶȻ�PID����
#define YAW_GYRO_KP 15000.0f
#define YAW_GYRO_KI 0.0f
#define YAW_GYRO_KD 10.0f//100.0f
#define YAW_GYRO_MAX_OUT 25000.0f//18000
#define YAW_GYRO_MAX_IOUT 1000.0f

//��̨��λ����ֵ�����趨
#define GIMBAL_YAW_OFFSET_ECD 7546//7545//4572
#define GIMBAL_YAW_MAX_ECD 3.14f
#define GIMBAL_YAW_MIN_ECD -3.13f

#define GIMBAL_PITCH_OFFSET_ECD 3428//2187
#define GIMBAL_PITCH_MAX_ECD  0.31f//0.56f
#define GIMBAL_PITCH_MIN_ECD -0.48f//-0.64f

// PITCH��̨����ģʽPID�ǶȻ�����
#define PITCH_ABSOLUTE_ANGLE_AUTO_KP 18.0f
#define PITCH_ABSOLUTE_ANGLE_AUTO_KI 0.2f
#define PITCH_ABSOLUTE_ANGLE_AUTO_KD -1.0f
#define PITCH_ABSOLUTE_ANGLE_AUTO_MAX_OUT 5.0f
#define PITCH_ABSOLUTE_ANGLE_AUTO_MAX_IOUT 2.0f

// YAW��̨����ģʽPID�ǶȻ�����
#define YAW_ABSOLUTE_ANGLE_AUTO_KP 18.0f
#define YAW_ABSOLUTE_ANGLE_AUTO_KI 0.035f
#define YAW_ABSOLUTE_ANGLE_AUTO_KD 0.0f
#define YAW_ABSOLUTE_ANGLE_AUTO_MAX_OUT 10.0f 
#define YAW_ABSOLUTE_ANGLE_AUTO_MAX_IOUT 0.4f

// YAW��̨����ģʽPID�ٶȻ�����
#define YAW_GYRO_AUTO_KP 14000.0f
#define YAW_GYRO_AUTO_KI 0.0f
#define YAW_GYRO_AUTO_KD 0.0f
#define YAW_GYRO_AUTO_MAX_OUT 25000.0f
#define YAW_GYRO_AUTO_MAX_IOUT 0.0f

//�������ֵת���ɽǶ�ֵ
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

typedef enum
{
  GIMBAL_MOTOR_RAW = 0, //���ԭʼֵ����
  GIMBAL_MOTOR_GYRO,    //��������ǽǶȿ���
  GIMBAL_MOTOR_ENCONDE  //�������ֵ�Ƕȿ���
} gimbal_motor_mode_e;

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
} gimbal_PID_t;

typedef struct
{
  const motor_measure_t *gimbal_motor_measure;
  gimbal_PID_t gimbal_motor_absolute_angle_pid;
  gimbal_PID_t gimbal_motor_relative_angle_pid;
  pid_type_def gimbal_motor_gyro_pid;
  gimbal_motor_mode_e gimbal_motor_mode;
  gimbal_motor_mode_e last_gimbal_motor_mode;
  uint16_t offset_ecd;
  float max_relative_angle; // rad
  float min_relative_angle; // rad

  float relative_angle;     // rad
  float relative_angle_set; // rad
  float absolute_angle;     // rad
  float absolute_angle_set; // rad
  float motor_gyro;         // rad/s
  float motor_gyro_set;
  float motor_speed;
  float raw_cmd_current;
  float current_set;
  int16_t given_current;
} gimbal_motor_t;

typedef struct
{
  gimbal_motor_t gimbal_yaw_motor;
  gimbal_motor_t gimbal_pitch_motor;
} gimbal_control_t;

extern gimbal_control_t gimbal_control;

/**
 * @brief          ����yaw �������ָ��
 * @param[in]      none
 * @retval         yaw���ָ��
 */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
 * @brief          ����pitch �������ָ��
 * @param[in]      none
 * @retval         pitch
 */
extern const gimbal_motor_t *get_pitch_motor_point(void);

/**
 * @brief ��̨��̬���ݷ��͵���λ��
 *
 */
extern void gimbal_data_send(void);

extern void gimbal_task(void const *pvParameters);

#endif
