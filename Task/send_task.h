/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       send_task.c/h
  * @brief      
  *             ��������ͨѶ����
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     5-13-2025     �ܱ���              
  *
  */
#ifndef __SEND_TASK_H
#define __SEND_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

void chassis_data_send(void);
extern void send_task(void const *pvParameters);

#endif
