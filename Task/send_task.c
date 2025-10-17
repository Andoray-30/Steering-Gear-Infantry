/**
 * @file send_task.c/h
 * @author 周宝琳
 * @brief 超级电容通讯任务线程
 * @version 0.1
 * @date 2025-05-13
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */
#include "chassis_task.h"
#include "chassis_power_control.h"
#include "main.h"
#include "detect_task.h"
#include "INS_task.h"
#include "CAN_receive.h"
#include "send_task.h"
#include "referee.h"
#include "chassis_task.h"
#include "arm_math.h"
#include "cmsis_os.h"


extern chassis_move_t chassis_move;
extern ext_game_robot_state_t robot_state;
 
uint16_t charge_power = 3000; 
void send_task(void const *pvParameters)
{
  vTaskDelay(CHASSIS_TASK_INIT_TIME);
	while(1)
	{
		chassis_data_send();
		vTaskDelay(5);
	}
}

//向超级电容发送功率限制数据
void chassis_data_send(void)
{
	#if XIDI
  static uint8_t send_time;
  //每5毫秒发送一次  500Hz
  if (send_time == 50)
  {    
		if(chassis_move.chassis_power_buffer > 20.0f && chassis_move.chassis_power_limit >= 60.0f)
		{
			super_cap_send_power((robot_state.chassis_power_limit+chassis_move.chassis_power_buffer-10) * 100);
		}
		else if(chassis_move.chassis_power_limit <= 60.0f)
		{
			super_cap_send_power(6000);
		}
		else
		{
			super_cap_send_power(robot_state.chassis_power_limit * 100);
		}
    send_time = 0;
  }
  send_time++;
	#else
	static uint8_t send_time=0;
	//开启电容
//	charge_power = (uint16_t)(robot_state.chassis_power_limit+chassis_move.chassis_power_buffer-10)*100;
	//防止20000J过快用完
	charge_power = (uint16_t)((robot_state.chassis_power_limit<60) ? ((robot_state.chassis_power_limit+chassis_move.chassis_power_buffer-10)*100) : robot_state.chassis_power_limit*100);


  if (send_time == 1)
  {
		//电池电压在20V左右时（快没电时），过高功率充电会导致输入过流，输入电流0-6A
		
		if(charge_power >= 9000)
		{
			PM01_power_set(9000,0x00);
		}
		else
		{
			PM01_power_set(charge_power,0x00);
		}
  }
  else if (send_time == 2)
  {
		PM01_OUT_READ();
	}
  else if (send_time == 3)
  {
		PM01_STATE_READ();
		send_time=0;
  }
  send_time++;
	#endif
}
