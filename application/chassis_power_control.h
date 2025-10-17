/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.���̹��ʿ���
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             ֻ����80w���ʣ���Ҫͨ�����Ƶ�������趨ֵ,������ƹ�����40w������
  *             JUDGE_TOTAL_CURRENT_LIMIT��POWER_CURRENT_LIMIT��ֵ�����е�������ٶ�
  *             (����max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H
#include "chassis_task.h"
#include "main.h"

#define   MOTOR_3508_R 0.0989f//���賣��
#define   MOTOR_3508_K 0.001709f
#define   MOTOR_3508_P 8.186f//����ĺ;�Ϣ��ĳ���

#define   MOTOR_6020_R 0.06471f//���賣��
#define   MOTOR_6020_K 5.922f
#define   MOTOR_6020_P 13.55f//����ĺ;�Ϣ��ĳ���

#define   RM3508_CURRENT_RATIO 20.0f/16384.0f//����ӳ��
#define   RM6020_CURRENT_RATIO 3.0f/16384.0f//����ӳ��

#define MIN_CAP_VOLTAGE 18.0f//16.5f//��С�ŵ��ѹ
#define SUPER_MIN_CAP_VOLTAGE 14.0f//10.0f//��С�ŵ��ѹ


/**
 * @brief          ת�������ƹ���
 * @param[in]      chassis_power_control: ��������
 * @retval         none
 */
extern void chassis_power_control_turn(chassis_move_t *chassis_power_control);
/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
/**
  * @brief          ���ƹ��ʣ���Ҫ���Ƶ������
  * @param[in]      chassis_power_control: ��������
  * @retval         none
  */
extern void chassis_power_control(chassis_move_t *chassis_power_control);
extern void chassis_power_6020_control(chassis_move_t *chassis_power_control);

/*
 *calfromPID
 *��pid�����㷨�з���ת�ٺ͵���ӳ���ϵ����K��M
 */
extern void calfromPID(chassis_move_t *chassis_power_control);
extern void calfromPID_6020(chassis_move_t *chassis_power_control);


#endif
