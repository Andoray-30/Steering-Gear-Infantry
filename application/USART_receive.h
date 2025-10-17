/**
 * @file USART_receive.h
 * @author ���廪
 * @brief �����жϽ��պ���������������Ƭ���������豸�Ĵ���ͨ������
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef USART_RECEIVE_H
#define USART_RECEIVE_H

#include "struct_typedef.h"

#define USART1_RX_BUF_NUM 32u
#define USER_FRAME_LENGTH 16u//11u

#define USART_PI 3.1416f

typedef struct
{
	  uint8_t fire_flag;
    float yaw_add;
    float pitch_add;
} auto_shoot_t;

typedef struct
{
  uint8_t header;
  uint8_t fire;
	float yaw;
	float pitch;
  uint32_t checksum;
} auto_aim_data_t;

typedef __packed struct
{
  uint8_t header;
  uint8_t detect_color;
  bool_t reset_tracker;
  uint8_t reserved;
  //float roll;
  float pitch;
  float yaw;
  uint8_t end;
  uint8_t d1;
  bool_t d2;
  uint8_t d3;
  //float aim_x;
  //float aim_y;
  //float aim_z;
  //bool_t mood;
  //uint32_t checksum;
	//float shoot_speed;
} user_send_data_t;

extern auto_shoot_t auto_shoot;
extern user_send_data_t user_send_data;
extern auto_aim_data_t auto_aim_data;
extern uint8_t recieve_auto;

extern void user_data_pack_handle(void);

#endif
