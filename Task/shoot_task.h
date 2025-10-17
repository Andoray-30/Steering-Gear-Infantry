/**
 * @file shoot_task.h
 * @author bobolin
 * @brief 
 * @version 0.1
 * @date 2024-03-21
 *
 * @copyright Copyright (c) 2022
 *
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __shoot_TASK_H__
#define __shoot_TASK_H__

/* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"

//����ʼǰ����һ��ʱ��
#define SHOOT_TASK_INIT_TIME 300
//���������� 5ms
#define SHOOT_CONTROL_TIME 5

//������俪��ͨ������
#define SHOOT_RC_MODE_CHANNEL RIGHT_SWITCH

//��곤���ж� 0.2 * 5 = 1ms
#define PRESS_LONG_TIME 100
//ң����������ؿ���һ��ʱ��� ���������ӵ� �����嵯 400 * 5 = 2000ms
#define RC_S_LONG_TIME 100

#define BULLET_SPEED_LIMIT 30.0f

//Ħ����PID
#define FRICTION_KP 20.0f
#define FRICTION_KP_L 20.0f
#define FRICTION_KI 0.0f
#define FRICTION_KD 1.2f

#define FRICTION_MAX_OUT 16000.0f
#define FRICTION_MAX_IOUT 1000.0f

//����PID
#define TRIGGER_KP 12.0f
#define TRIGGER_KI 0.0f
#define TRIGGER_KD 0.0f
#define TRIGGER_MAX_OUT 9500.0f
#define TRIGGER_MAX_IOUT 6000.0f

//�����ٶ��趨��ң��������ʱ��Ϊ�˷�����������һ����
#define TRIGGER_SPEED -2800.0f 

//���������ٶ�
#define CONTINUE_TRIGGER_SPEED -2800.0f 

//�������ƽ�Ϊ��ԣʱ��ת��
#define CONTINUE_TRIGGER_SPEED_H -4500.0f 

//Ħ�����ٶ��趨
#define FRICTION_SPEED_SET 6050.0f//6200.0f//6400.0f

#define FRICTION_ACCEL_MAX_OUT 8500 


//����ʱ�� �Լ���תʱ��
#define BLOCK_TRIGGER_SPEED 1.0f
#define BLOCK_TIME 25    // 5 * 400 = 2000ms
#define REVERSE_TIME 200 // 5 * 150 = 750ms
#define REVERSE_SPEED_LIMIT 13.0f

//����Ԥ��ֵ����ֹ������
#define SHOOT_HEAT_REMAIN_VALUE 40
//������Ϊ��ԣʱ
#define SHOOT_HEAT_REMAIN_VALUE_H 60


typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,  //��Ħ����
    SHOOT_BULLET, //�ӵ�
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,
} shoot_mode_e;

typedef struct
{
    const motor_measure_t *motor_measure;
    pid_type_def motor_speed_pid;
    float speed;
    float speed_set;
    float angle;
    float set_angle;
    int16_t given_current;
    int8_t ecd_count;
} trigger_motor_t;

typedef struct
{
    const motor_measure_t *motor_measure;
    pid_type_def motor_speed_pid;
    float accel;          //���ٶ� 
    float speed;          //��ǰ�ٶ� 
    float speed_set;      //�趨�ٶ� 
    int16_t given_current; //���͵��� 
} friction_motor_t;

typedef struct
{
    shoot_mode_e shoot_mode;

    trigger_motor_t trigger_motor;
    friction_motor_t right_fricition_motor;
    friction_motor_t left_fricition_motor;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;

    uint16_t heat_limit;
    uint16_t heat;

    float bullet_speed;
    float bullet_last_speed;

} shoot_control_t;

void fire_task(void const *pvParameters);

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
