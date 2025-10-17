/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "detect_task.h"

#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
float _207_angle[2];

/**
 * @brief data of capacity
 * @param PowerData[0] input voltage
 * @param PowerData[1] cap voltage
 * @param PowerData[2] input current
 * @param PowerData[3] target power
 */
float PowerData[4];
Super_Power_t Super_Power_Data;//���ͳ�������


int16_t round_207;
float this_angle_207;
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }
static motor_measure_t motor_chassis[4];  //����3508���
static motor_measure_t motor_chassis_turn[4];  //����6020���
static motor_measure_t motor_gimbal[2];   //��̨���
static motor_measure_t motor_trigger;     //�������
static motor_measure_t motor_friction[2]; //Ħ���ֵ��
static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static CAN_TxHeaderTypeDef tx_message;
static uint8_t chassis_can_send_data[8];
static uint8_t tx_message_data[8];
static CAN_TxHeaderTypeDef cap_tx_message;
static uint8_t cap_can_send_data[8];
static CAN_TxHeaderTypeDef super_power_tx_message;//���ϳ���
static uint8_t super_power_can_send_data[8];



/**
 * @brief           ����3508����ۼ���ת�Ƕ�
 * @param[out]      motor:����ṹ����ָ��
 */
void Calc_motor_Angle(motor_measure_t *motor)
{
  if (motor->ecd - motor->last_ecd > 4095.5)
  {
    motor->round--;
  }
  else if (motor->ecd - motor->last_ecd < -4095.5)
  {
    motor->round++;
  }
  motor->angle = (motor->round * ANGLE_T + motor->ecd) * 0.04395067757f; //ת��Ϊ360��һȦ
}

/**
 * @brief          hal��CAN�ص�����,���յ������
 * @param[in]      hcan:CAN���ָ��
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;

  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  if (hcan == &hcan1)
  {
    switch (rx_header.StdId)
    {
    case CAN_3508_M1_ID:
    {
      get_motor_measure(&motor_chassis[0], rx_data); //��ȡ����
      // Calc_motor_Angle(&motor_chassis[0]);//�Ƕȼ���
      detect_hook(CHASSIS_MOTOR1_TOE); //������
      break;
    }
    case CAN_3508_M2_ID:
    {
      get_motor_measure(&motor_chassis[1], rx_data);
      // Calc_motor_Angle(&motor_chassis[1]);
      detect_hook(CHASSIS_MOTOR2_TOE);
      break;
    }
    case CAN_3508_M3_ID:
    {
      get_motor_measure(&motor_chassis[2], rx_data);
      // Calc_motor_Angle(&motor_chassis[2]);
      detect_hook(CHASSIS_MOTOR3_TOE);
      break;
    }
    case CAN_3508_M4_ID:
    {
      get_motor_measure(&motor_chassis[3], rx_data);
      // Calc_motor_Angle(&motor_chassis[3]);
      detect_hook(CHASSIS_MOTOR4_TOE);
      break;
    }
    case CAN_YAW_MOTOR_ID:
    {
      get_motor_measure(&motor_gimbal[0], rx_data);
      // Calc_motor_Angle(&motor_gimbal[0]);
      detect_hook(YAW_GIMBAL_MOTOR_TOE);
      break;
    }
    case CAN_PIT_MOTOR_ID:
    {
      get_motor_measure(&motor_gimbal[1], rx_data);
      // Calc_motor_Angle(&motor_gimbal[1]);
      detect_hook(PITCH_GIMBAL_MOTOR_TOE);
      break;
    }
    case CAN_TRIGGER_MOTOR_ID:
    {
      get_motor_measure(&motor_trigger, rx_data);
      Calc_motor_Angle(&motor_trigger);
      detect_hook(TRIGGER_MOTOR_TOE);
      _207_angle[1] = _207_angle[0];
      _207_angle[0] = motor_trigger.ecd * 0.044;
      if (_207_angle[0] - _207_angle[1] > 180)
      {
        round_207--;
      }
      else if (_207_angle[0] - _207_angle[1] < -180)
      {
        round_207++;
      }
      this_angle_207 = round_207 * 360 + _207_angle[0];
      break;
    }
    case 0x211://Ϫ�س�������
    {
			 // extern float PowerData[4];
      uint16_t *pPowerData = (uint16_t *)rx_data;
      PowerData[0] = (float)pPowerData[0] / 100.f;
      PowerData[1] = (float)pPowerData[1] / 100.f;
      PowerData[2] = (float)pPowerData[2] / 100.f;
      PowerData[3] = (float)pPowerData[3] / 100.f;
      break;
    }


    default:
    {
      break;
    }
    }
  }
  else if (hcan == &hcan2)
  {
    switch (rx_header.StdId)
    {
    case CAN_LEFT_FRICTION_ID:
    {
      get_motor_measure(&motor_friction[0], rx_data); //��ȡ����
      detect_hook(LEFT_FRICTION_MOTOR_TOE);           //������
      break;
    }
    case CAN_RIGHT_FRICTION_ID:
    {
      get_motor_measure(&motor_friction[1], rx_data); //��ȡ����
      detect_hook(RIGHT_FRICTION_MOTOR_TOE);          //������
      break;
    }

    case CAN_6020_M1_ID:
    {
      get_motor_measure(&motor_chassis_turn[0], rx_data); //��ȡ����
      detect_hook(CHASSIS_MOTOR1_TURN_TOE);           //������
      break;
    }
    case CAN_6020_M2_ID:
    {
      get_motor_measure(&motor_chassis_turn[1], rx_data); //��ȡ����
      detect_hook(CHASSIS_MOTOR2_TURN_TOE);           //������
      break;
    }
    case CAN_6020_M3_ID:
    {
      get_motor_measure(&motor_chassis_turn[2], rx_data); //��ȡ����
      detect_hook(CHASSIS_MOTOR3_TURN_TOE);           //������
      break;
    }
    case CAN_6020_M4_ID:
    {
      get_motor_measure(&motor_chassis_turn[3], rx_data); //��ȡ����
      detect_hook(CHASSIS_MOTOR4_TURN_TOE);           //������
      break;
    }
		case POWER_ENABLE_ID:
		{
			Super_Power_Data.ccr = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
			break;
		}
		case POWER_P_SET_ID:
		{
			Super_Power_Data.p_set = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
			break;
		}
		case POWER_V_SET_ID:
		{
			Super_Power_Data.v_set = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
			break;
		}
		case POWER_DATA_IN_ID:
		{
			Super_Power_Data.p_in = (float)(rx_data[0] << 8 | rx_data[1])/100.0f;
			Super_Power_Data.v_in = (float)(rx_data[2] << 8 | rx_data[3])/100.0f;
			Super_Power_Data.i_in = (float)(rx_data[4] << 8 | rx_data[5])/100.0f;
			break;
		}
		case POWER_DATA_OUT_ID:
		{
			Super_Power_Data.p_out = (float)(rx_data[0] << 8 | rx_data[1])/100.0f;
			Super_Power_Data.v_out = (float)(rx_data[2] << 8 | rx_data[3])/100.0f;
			Super_Power_Data.i_out = (float)(rx_data[4] << 8 | rx_data[5])/100.0f;
			break;
		}
		case POWER_STATE_ID:
		{
			Super_Power_Data.temp = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
			break;
		}
		case POWER_ERROE_ID:
		{
			Super_Power_Data.state = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
			Super_Power_Data.error = (uint16_t)(rx_data[2] << 8 | rx_data[3]);
			break;
		}
		

    default:
    {
      break;
    }
    }
  }
}
/*								���������������								*/
void PM01_IN_READ(void)
{
  uint32_t send_mail_box;
  super_power_tx_message.StdId = 0x611;
  super_power_tx_message.IDE = CAN_ID_STD;
  super_power_tx_message.RTR = CAN_RTR_REMOTE;
  super_power_tx_message.DLC = 0x00;
  HAL_CAN_AddTxMessage(&hcan2, &super_power_tx_message, super_power_can_send_data, &send_mail_box);
}
void PM01_OUT_READ(void)
{
  uint32_t send_mail_box;
  super_power_tx_message.StdId = 0x612;
  super_power_tx_message.IDE = CAN_ID_STD;
  super_power_tx_message.RTR = CAN_RTR_REMOTE;
  super_power_tx_message.DLC = 0x00;
  HAL_CAN_AddTxMessage(&hcan2, &super_power_tx_message, super_power_can_send_data, &send_mail_box);
}
void PM01_TEMP_READ(void)
{
  uint32_t send_mail_box;
  super_power_tx_message.StdId = 0x613;
  super_power_tx_message.IDE = CAN_ID_STD;
  super_power_tx_message.RTR = CAN_RTR_REMOTE;
  super_power_tx_message.DLC = 0x00;
  HAL_CAN_AddTxMessage(&hcan2, &super_power_tx_message, super_power_can_send_data, &send_mail_box);
}
void PM01_STATE_READ(void)
{
  uint32_t send_mail_box;
  super_power_tx_message.StdId = POWER_ERROE_ID;
  super_power_tx_message.IDE = CAN_ID_STD;
  super_power_tx_message.RTR = CAN_RTR_REMOTE;
  super_power_tx_message.DLC = 0x00;
  HAL_CAN_AddTxMessage(&hcan2, &super_power_tx_message, super_power_can_send_data, &send_mail_box);
}
/**
  * @brief          ���������
  * @param[in]      new_cmd   0x00: ͣ��
															0x01: ���У�����������ؿ��أ�ֻ���������ݳ�磩
															0x02: ���У���������ؿ��أ���������ʹ�ø�ָ�
	                  save_flg: 0x00: ��������EEPROM  0x01: ������EEPROM
  * @retval         none
  */
void PM01_cmd_send( uint16_t new_cmd, uint8_t save_flg )//Ĭ��Ϊ2��Ӧ�ò����������Ҳ��
{
	uint32_t send_mail_box;
	super_power_tx_message.StdId = POWER_ENABLE_ID;
	super_power_tx_message.IDE   = CAN_ID_STD;
	super_power_tx_message.RTR   = CAN_RTR_DATA;
	super_power_tx_message.DLC   = 0x04;
	super_power_can_send_data[0] = (uint8_t)(new_cmd >> 8   );
	super_power_can_send_data[1] = (uint8_t)(new_cmd &  0xFF);
	super_power_can_send_data[2] = 0x00;
	super_power_can_send_data[3] = (save_flg == 0x01);			
	HAL_CAN_AddTxMessage(&hcan2, &super_power_tx_message, super_power_can_send_data, &send_mail_box);
	
}

/**
  * @brief          ���ù���
  * @param[in]      new_power���µĹ���ֵ
                    save_flg: 0x00: ��������EEPROM  0x01: ������EEPROM
  * @retval         none
  */
void PM01_power_set( uint16_t new_power, uint8_t save_flg )
{
	uint32_t send_mail_box;
	super_power_tx_message.StdId = POWER_P_SET_ID;
	super_power_tx_message.IDE   = CAN_ID_STD;
	super_power_tx_message.RTR   = CAN_RTR_DATA;
	super_power_tx_message.DLC   = 0x04;
	super_power_can_send_data[0] = (uint8_t)(new_power >> 8   );
	super_power_can_send_data[1] = (uint8_t)(new_power &  0xFF);
	super_power_can_send_data[2] = 0x00;
	super_power_can_send_data[3] = (save_flg == 0x01);			

	HAL_CAN_AddTxMessage(&hcan2, &super_power_tx_message, super_power_can_send_data, &send_mail_box);
	
}
/**
  * @brief          ���������ѹ
  * @param[in]      new_volt���µĵ�ѹֵ
                    save_flg: 0x00: ��������EEPROM  0x01: ������EEPROM
  * @retval         none
  */
void PM01_voltage_set( uint16_t new_voltage, uint8_t save_flg )
{
	uint32_t send_mail_box;
	super_power_tx_message.StdId = POWER_V_SET_ID;
	super_power_tx_message.IDE   = CAN_ID_STD;
	super_power_tx_message.RTR   = CAN_RTR_DATA;
	super_power_tx_message.DLC   = 0x04;
	super_power_can_send_data[0] = (uint8_t)(new_voltage >> 8   );
	super_power_can_send_data[1] = (uint8_t)(new_voltage &  0xFF);
	super_power_can_send_data[2] = 0x00;
	super_power_can_send_data[3] = (save_flg == 0x01);			
	HAL_CAN_AddTxMessage(&hcan2, &super_power_tx_message, super_power_can_send_data, &send_mail_box);
	
}
/*								���������������								*/
/**
 * @brief Ħ���ֿ��������ͺ��� can2
 *
 * @param left_friction
 * @param rigit_friction
 * @param rev
 */
void CAN_cmd_friction(int16_t left_friction, int16_t rigit_friction)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_FRICTION_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (left_friction >> 8);
  gimbal_can_send_data[1] = left_friction;
  gimbal_can_send_data[2] = (rigit_friction >> 8);
  gimbal_can_send_data[3] = rigit_friction;
  gimbal_can_send_data[4] = 0;
  gimbal_can_send_data[5] = 0;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}


/**
 * @brief ����������������ͺ��� can1
 *
 * @param shoot
 */
void CAN_cmd_shoot(int16_t shoot)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = 0x1FF;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = 0;
  gimbal_can_send_data[1] = 0;
  gimbal_can_send_data[2] = 0;
  gimbal_can_send_data[3] = 0;
  gimbal_can_send_data[4] = (shoot >> 8);
  gimbal_can_send_data[5] = shoot;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208) can1
 * @param[in]      yaw: (0x209) 6020������Ƶ���, ��Χ [-30000,30000]
 * @param[in]      pitch: (0x20A) 6020������Ƶ���, ��Χ [-30000,30000]
 * @retval         none
 */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (yaw >> 8);
  gimbal_can_send_data[1] = yaw;
  gimbal_can_send_data[2] = (pitch >> 8);
  gimbal_can_send_data[3] = pitch;
  gimbal_can_send_data[4] = 0;
  gimbal_can_send_data[5] = 0;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204) can1
 * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
 * @retval         none
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          ����6020������Ƶ���(0x207,0x208) can2
 * @param[in]      motor1: (0x207) 6020������Ƶ���, ��Χ [-25000,25000]
 * @param[in]      motor2: (0x208) 6020������Ƶ���, ��Χ [-25000,25000]
 * @retval         none
 */
void CAN_cmd_chassis_turn_1(int16_t motor1, int16_t motor2)//ID3/4 6020��� ���Ʊ��ı�ʶ��0X1FF
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_TURN_1_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0;
  chassis_can_send_data[1] = 0;
  chassis_can_send_data[2] = 0;
  chassis_can_send_data[3] = 0;
  chassis_can_send_data[4] = motor1 >> 8;
  chassis_can_send_data[5] = motor1;
  chassis_can_send_data[6] = motor2 >> 8;
  chassis_can_send_data[7] = motor2;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN_turn, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
void CAN_cmd_chassis_turn_2(int16_t motor3, int16_t motor4)//ID5/6 6020��� ���Ʊ��ı�ʶ��0X2FF
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_TURN_2_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor3 >> 8;
  chassis_can_send_data[1] = motor3;
  chassis_can_send_data[2] = motor4 >> 8;
  chassis_can_send_data[3] = motor4;
  chassis_can_send_data[4] = 0;
  chassis_can_send_data[5] = 0;
  chassis_can_send_data[6] = 0;
  chassis_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN_turn, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          ����yaw 6020�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_gimbal[0];
}

/**
 * @brief          ����pitch 6020�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_gimbal[1];
}

/**
 * @brief          ���ز������ 2006�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_trigger;
}

const motor_measure_t *get_left_friction_motor_measure_point(void)
{
  return &motor_friction[0];
}

const motor_measure_t *get_right_friction_motor_measure_point(void)
{
  return &motor_friction[1];
}

/**
 * @brief          ���ص��̵�� 3508�������ָ��
 * @param[in]      i: ������,��Χ[0,3]
 * @retval         �������ָ��
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}

/**
 * @brief          ���ص��̵�� 6020�������ָ��
 * @param[in]      i: ������,��Χ[0,3]
 * @retval         �������ָ��
 */
const motor_measure_t *get_chassis_turn_motor_measure_point(uint8_t i)
{
  return &motor_chassis_turn[(i & 0x03)];
}

/**
 * @brief     ���ͳ��������趨����
 * @param Power   3000~13000 ��Ӧ30W~130W
 * 10Hz
 */
void super_cap_send_power(uint16_t Power)
{
  uint32_t send_mail_box;
  cap_tx_message.StdId = 0x210;
  cap_tx_message.IDE = CAN_ID_STD;
  cap_tx_message.RTR = CAN_RTR_DATA;
  cap_tx_message.DLC = 0x08;
  cap_can_send_data[0] = Power >> 8;
  cap_can_send_data[1] = Power;
  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &cap_tx_message, cap_can_send_data, &send_mail_box);
}

