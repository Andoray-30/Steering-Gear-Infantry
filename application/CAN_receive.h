/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define CHASSIS_CAN_turn hcan2

#define ANGLE_T 8191
extern float PowerData[4];

/* CAN send and receive ID */
typedef enum
{
  CAN_CHASSIS_ALL_ID = 0x200,
  CAN_3508_M1_ID = 0x201,
  CAN_3508_M2_ID = 0x202,
  CAN_3508_M3_ID = 0x203,
  CAN_3508_M4_ID = 0x204,

  CAN_CHASSIS_TURN_1_ALL_ID = 0x1FE,
  CAN_CHASSIS_TURN_2_ALL_ID = 0x2FE,
  CAN_6020_M1_ID = 0x207,
  CAN_6020_M2_ID = 0x208,
  CAN_6020_M3_ID = 0x209,
  CAN_6020_M4_ID = 0x20A,

  CAN_GIMBAL_ALL_ID = 0x2FF,
  CAN_YAW_MOTOR_ID = 0x209,
  CAN_PIT_MOTOR_ID = 0x20A,
  CAN_TRIGGER_MOTOR_ID = 0x207,

  CAN_FRICTION_ALL_ID = 0x200,
  CAN_LEFT_FRICTION_ID = 0x201,
  CAN_RIGHT_FRICTION_ID = 0x202,
	
	POWER_ENABLE_ID = 0x600,
	POWER_P_SET_ID = 0x601,
	POWER_V_SET_ID = 0x602,
	POWER_ERROE_ID = 0x610,
	POWER_DATA_IN_ID = 0x611,
	POWER_DATA_OUT_ID = 0x612,
	POWER_STATE_ID = 0x613,


	
} can_msg_id_e;

// rm motor data
typedef struct
{
  int16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
  int32_t round;
  float angle;
} motor_measure_t;

typedef struct
{
	uint16_t ccr; 
	uint16_t state;
	uint16_t error;
	uint16_t p_set; 
	uint16_t v_set; 	
	uint16_t i_set; 	
	float  v_in; 
	float  i_in; 
	float  p_in; 	
	float  v_out;	
	float  i_out;	
	float  p_out;
	int16_t  temp;	
	
}Super_Power_t;//安合超电数据结构体
extern Super_Power_t Super_Power_Data;//安和超电数据


/**
 * @brief 摩擦轮控制量发送函数 can2
 *
 * @param left_friction
 * @param rigit_friction
 */
extern void CAN_cmd_friction(int16_t left_friction, int16_t rigit_friction);

/**
 * @brief 拨弹电机控制量发送函数 can1
 *
 * @param shoot
 */
extern void CAN_cmd_shoot(int16_t shoot);

/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
 * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
 * @retval         none
 */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch);

/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
 * @brief          发送电机控制电压(0x207,0x208)
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-25000,25000]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-25000,25000]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-25000,25000]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-25000,25000]
 * @retval         none
 */
extern void CAN_cmd_chassis_turn_1(int16_t motor1, int16_t motor2);

/**
 * @brief          发送电机控制电压(0x209,0x20A)
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-25000,25000]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-25000,25000]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-25000,25000]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-25000,25000]
 * @retval         none
 */
extern void CAN_cmd_chassis_turn_2(int16_t motor3, int16_t motor4);
/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
 * @brief          返回pitch 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

extern const motor_measure_t *get_left_friction_motor_measure_point(void);
extern const motor_measure_t *get_right_friction_motor_measure_point(void);

/**
 * @brief          返回拨弹电机 2006电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/**
 * @brief          返回底盘电机 6020电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_chassis_turn_motor_measure_point(uint8_t i);

/**
 * @brief     溪地发送超级电容设定功率
 * @param Power   3000~13000 对应30W~130W
 * 10Hz
 */
extern void super_cap_send_power(uint16_t Power);

/**
 * @brief     安合电容请求输入数据
 */
extern void PM01_IN_READ(void);
/**
 * @brief     安合电容请求输出数据
 */
extern void PM01_OUT_READ(void);
/**
 * @brief     安合电容请求状态数据
 */
extern void PM01_TEMP_READ(void);
/**
 * @brief     安合电容请求运行状态数据
 */
extern void PM01_STATE_READ(void);

/**
  * @brief          控制命令发送
  * @param[in]      new_cmd   0x00: 停机
															0x01: 运行，不打开输出负载开关（只给超级电容充电）
															0x02: 运行，打开输出负载开关（正常运行使用该指令）
	                  save_flg: 0x00: 不保存至EEPROM  0x01: 保存至EEPROM
  * @retval         none
  */
extern void PM01_cmd_send( uint16_t new_cmd, uint8_t save_flg );//默认为2
/**
  * @brief          设置功率
  * @param[in]      new_power：新的功率值
                    save_flg: 0x00: 不保存至EEPROM  0x01: 保存至EEPROM
  * @retval         none
  */
extern void PM01_power_set( uint16_t new_power, uint8_t save_flg );
	
/**
  * @brief          设置输出电压
  * @param[in]      new_volt：新的电压值
                    save_flg: 0x00: 不保存至EEPROM  0x01: 保存至EEPROM
  * @retval         none
  */
extern void PM01_voltage_set( uint16_t new_voltage, uint8_t save_flg );





#endif
