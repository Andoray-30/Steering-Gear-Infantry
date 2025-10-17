/**
 * @file chassis_task.c/h
 * @author �ܱ���
 * @brief ���̿��������߳�
 * @version 0.1
 * @date 2022-03-06
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */
/********************************************************************************************************************
//                                                          _ooOoo_
//                                                         o8888888o
//                                                         88" . "88
//                                                         (| -_- |)
//                                                          O\ = /O
//                                                      ____/`---'\____
//                                                    .   ' \\| |// `.
//                                                     / \\||| : |||// \
//                                                   / _||||| -:- |||||- \
//                                                     | | \\\ - /// | |
//                                                   | \_| ''\---/'' | |
//                                                    \ .-\__ `-` ___/-. /
//                                                 ___`. .' /--.--\ `. . __
//                                              ."" '< `.___\_<|>_/___.' >'"".
//                                             | | : `- \`.;`\ _ /`;.`/ - ` : | |
//                                               \ \ `-. \_ __\ /__ _/ .-` / /
//                                       ======`-.____`-.___\_____/___.-`____.-'======
//                                                          `=---='
//
//                                              ���汣��            ����BUG
********************************************************************************************************************/
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "chassis_power_control.h"
#include "cmsis_os.h"
#include "main.h"
#include "arm_math.h"
#include "detect_task.h"
#include "INS_task.h"
#include "custom_ui_draw.h"
#include "CAN_receive.h"
#include "referee.h"
#include "bsp_adc.h"

float chassis[3];
extern tank_ui_t tank;
extern super_capacity_t super_capacity;
extern aim_dynamic_t aim_dynamic;
extern ext_game_robot_state_t robot_state;
uint16_t data_dianrong_giegie=0;
float x;//���ڹ�����ϣ�xΪת�ٳ��Ե���
float y;//���ڹ�����ϣ�yΪ����ƽ����
uint32_t color;//���ڵ�����ɫ���ѹ�ı�
float min_V = MIN_CAP_VOLTAGE;
/**
 * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
 * @retval         none
 */
static void chassis_init(void);

/**
 * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
 * @retval         none
 */
static void chassis_set_mode(void);

/**
 * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @param[out]     chassis_move_update:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_feedback_update(void);

/**
 * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
 * @retval         none
 */
static void chassis_set_contorl(void);

static void chassis_vector_to_M3508_wheel_speed(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_speed[4]);


/**
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_control_loop(void);


/**
 * @brief          ����ecd��offset_ecd֮�����ԽǶ�
 * @param[in]      ecd: �����ǰ����
 * @param[in]      offset_ecd: �����ֵ����
 * @retval         ��ԽǶȣ���λrad
 */
static float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

/**
 * @brief          �Ƕȷ�Χ����
 * @param[in]      angel: ��ǰ�Ƕ�
 * @param[in]      max: ����������ֵ
 * @retval         ��ԽǶȣ���λrad
 */
static float Angle_Limit(float angle ,float max);


//�����˶�����
chassis_move_t chassis_move;

/**
 * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: ��
 * @retval         nonex
 */
void chassis_task(void const *pvParameters)
{
  //����һ��ʱ��
  vTaskDelay(CHASSIS_TASK_INIT_TIME);
  //���̳�ʼ��
  chassis_init();
  //�жϵ��̵���Ƿ�����
  while (1)
  {
    //���õ��̿���ģʽ
    chassis_set_mode();
    //�������ݸ���
    chassis_feedback_update();
    //���̿���������
    chassis_set_contorl();
    //���̿���PID����
    chassis_control_loop();
    cal_draw_gimbal_relative_angle_tangle(&tank);
    cal_capacity(&super_capacity);
    cal_draw_pitch_aim(&aim_dynamic);

    //���̵������
    chassis[0] = chassis_move.motor_chassis[0].speed;
    chassis[1] = chassis_move.motor_chassis_turn[0].give_current;
    chassis[2] = chassis_move.motor_chassis[2].give_current;

    //��ң�������߻��ʱ�򣬷��͸����̵�������.
    if (toe_is_error(DBUS_TOE))
    {
			CAN_cmd_chassis(0, 0, 0, 0);
    }
    else
    {
      //���Ϳ��Ƶ���
			CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                       chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);

			CAN_cmd_chassis_turn_1(chassis_move.motor_chassis_turn[0].give_current,chassis_move.motor_chassis_turn[1].give_current);
			CAN_cmd_chassis_turn_2(chassis_move.motor_chassis_turn[2].give_current,chassis_move.motor_chassis_turn[3].give_current);
//			CAN_cmd_chassis_turn_1(0,0);
//			CAN_cmd_chassis_turn_2(0,0);
//			CAN_cmd_chassis(0, 0, 0, 0);

    }

    //ϵͳ��ʱ
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);
  }
}
/**
 * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
 * @retval         none
 */
static void chassis_init(void)
{
  const static float chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
  const static float chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
#if XIDI
#else
	PM01_cmd_send(2,0x00);//��������
#endif
  uint8_t i;

  //��ȡ��̨�������ָ��
  chassis_move.chassis_yaw_motor = get_yaw_motor_point();
  chassis_move.chassis_pitch_motor = get_pitch_motor_point();
  //��ȡ����3508�������ָ�룬��ʼ��PID
  for (i = 0; i < 4; i++)
  {
    chassis_move.motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
    PID_init(&chassis_move.motor_speed_pid[i], CHASSIS_MOTOR_SPEED_KP, CHASSIS_MOTOR_SPEED_KI, CHASSIS_MOTOR_SPEED_KD, CHASSIS_MOTOR_SPEED_MAX_OUT, CHASSIS_MOTOR_SPEED_MAX_IOUT);
  }
  //��ȡ����6020�������ָ�룬��ʼ��PID
  for (i = 0; i < 4; i++)
  {
    chassis_move.motor_chassis_turn[i].chassis_motor_measure = get_chassis_turn_motor_measure_point(i);
    PID_init(&chassis_move.motor_turn_speed_pid[i], CHASSIS_TURN_MOTOR_SPEED_KP, CHASSIS_TURN_MOTOR_SPEED_KI, CHASSIS_TURN_MOTOR_SPEED_KD, CHASSIS_TURN_MOTOR_SPEED_MAX_OUT, CHASSIS_TURN_MOTOR_SPEED_MAX_IOUT);//ת�����ٶȻ�
    PID_init(&chassis_move.motor_turn_angle_pid[i], CHASSIS_TURN_MOTOR_ANGLE_KP, CHASSIS_TURN_MOTOR_ANGLE_KI, CHASSIS_TURN_MOTOR_ANGLE_KD, CHASSIS_TURN_MOTOR_ANGLE_MAX_OUT, CHASSIS_TURN_MOTOR_ANGLE_MAX_IOUT);//ת�����ǶȻ�
  }

  //��ʼ���Ƕ�PID(��ת������̨)
  PID_init(&chassis_move.chassis_angle_pid, CHASSIS_ANGLE_KP, CHASSIS_ANGLE_KI, CHASSIS_ANGLE_KD, CHASSIS_ANGLE_MAX_OUT, CHASSIS_ANGLE_MAX_IOUT);
  //��һ���˲�����б����������
  first_order_filter_init(&chassis_move.chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
  first_order_filter_init(&chassis_move.chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);


  chassis_move.vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
  chassis_move.vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

  chassis_move.vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
  chassis_move.vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

  chassis_move.top_max_speed = TOP_MAX_CHASSIS_SPEED;
  chassis_move.top_min_speed = -TOP_MAX_CHASSIS_SPEED;

  chassis_move.motor_chassis_turn[0].offset_ecd = CHASSIS_1_TURN_MOTOR_OFFSET_ECD;
  chassis_move.motor_chassis_turn[1].offset_ecd = CHASSIS_2_TURN_MOTOR_OFFSET_ECD;
  chassis_move.motor_chassis_turn[2].offset_ecd = CHASSIS_3_TURN_MOTOR_OFFSET_ECD;
  chassis_move.motor_chassis_turn[3].offset_ecd = CHASSIS_4_TURN_MOTOR_OFFSET_ECD;

  chassis_move.chassis_power = 0.0f;
  chassis_move.chassis_power_buffer = 0.0f;

  //����һ������
  chassis_feedback_update();
}

/**
 * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
 * @retval         none
 */
static void chassis_set_mode()
{
  chassis_behaviour_mode_set(&chassis_move);
}

/**
 * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @retval         none
 */
static void chassis_feedback_update(void)
{
  uint8_t i = 0;
  for (i = 0; i < 4; i++)
  {
    //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
    chassis_move.motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move.motor_chassis[i].chassis_motor_measure->speed_rpm;
    chassis_move.motor_chassis[i].accel = chassis_move.motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    chassis_move.motor_chassis_turn[i].relative_angle = motor_ecd_to_angle_change(chassis_move.motor_chassis_turn[i].chassis_motor_measure->ecd, chassis_move.motor_chassis_turn[i].offset_ecd);
	chassis_move.motor_chassis_turn[i].give_current_now = chassis_move.motor_chassis_turn[i].chassis_motor_measure->given_current * 3.0f/16384.0f;
  }

  //���������̬�Ƕ�
  chassis_move.chassis_yaw = rad_format(INS_data.angle_yaw - gimbal_control.gimbal_yaw_motor.relative_angle);
  chassis_move.chassis_pitch = rad_format(INS_data.angle_pitch - gimbal_control.gimbal_pitch_motor.relative_angle);
  chassis_move.chassis_roll = INS_data.angle_roll;
	
#ifdef POWER_FITTING	
  //����ƽ����
  chassis_move.chassis_current_double = chassis_move.motor_chassis_turn[0].give_current_now * chassis_move.motor_chassis_turn[0].give_current_now
                                  + chassis_move.motor_chassis_turn[1].give_current_now * chassis_move.motor_chassis_turn[1].give_current_now
                                  + chassis_move.motor_chassis_turn[2].give_current_now * chassis_move.motor_chassis_turn[2].give_current_now
                                  + chassis_move.motor_chassis_turn[3].give_current_now * chassis_move.motor_chassis_turn[3].give_current_now;

  chassis_move.chassis_current_and_rpm = chassis_move.motor_chassis_turn[0].give_current_now * chassis_move.motor_chassis_turn[0].chassis_motor_measure->speed_rpm
                                  + chassis_move.motor_chassis_turn[1].give_current_now * chassis_move.motor_chassis_turn[1].chassis_motor_measure->speed_rpm
                                  + chassis_move.motor_chassis_turn[2].give_current_now * chassis_move.motor_chassis_turn[2].chassis_motor_measure->speed_rpm
                                  + chassis_move.motor_chassis_turn[3].give_current_now * chassis_move.motor_chassis_turn[3].chassis_motor_measure->speed_rpm;
#endif 
	
  get_chassis_power_and_buffer_and_power_limit(&chassis_move.chassis_power, &chassis_move.chassis_power_buffer, &chassis_move.chassis_power_limit, &chassis_move.chassis_output);
#if POWER_CAP//�Ƿ��е���
	#if XIDI
  if(PowerData[1] > MIN_CAP_VOLTAGE)
  {
		if(chassis_move.chassis_power_limit >= 60.0f)
		{
			chassis_move.chassis_3508_power_set = (float)chassis_move.chassis_power_limit + 1.5f* (PowerData[1]*PowerData[1] - MIN_CAP_VOLTAGE*MIN_CAP_VOLTAGE);
			chassis_move.chassis_6020_power_set = (float)chassis_move.chassis_power_limit + 1.5f* (PowerData[1]*PowerData[1] - MIN_CAP_VOLTAGE*MIN_CAP_VOLTAGE);
		}
		else
		{
			chassis_move.chassis_3508_power_set = 60.0f + 1.5f* (PowerData[1]*PowerData[1] - MIN_CAP_VOLTAGE*MIN_CAP_VOLTAGE);
			chassis_move.chassis_6020_power_set = 60.0f + 1.5f* (PowerData[1]*PowerData[1] - MIN_CAP_VOLTAGE*MIN_CAP_VOLTAGE);
		}
  }
  else
  {
		if(chassis_move.chassis_power_limit >= 60.0f)
		{
			chassis_move.chassis_3508_power_set = chassis_move.chassis_power_limit - 20.0f;
			chassis_move.chassis_6020_power_set = 300.0f;
		}
		else
		{
			chassis_move.chassis_3508_power_set = 25.0f;
			chassis_move.chassis_6020_power_set = 10.0f;
		}
 }
	#else
	if(Super_Power_Data.v_out > min_V)
  {
		chassis_move.chassis_3508_power_set = (float)chassis_move.chassis_power_limit + 0.3f* (Super_Power_Data.v_out*Super_Power_Data.v_out - MIN_CAP_VOLTAGE*MIN_CAP_VOLTAGE);
		chassis_move.chassis_6020_power_set = 300.f;
  }
  else
  {
		if(chassis_move.chassis_power_limit >= 60.0f)
		{
			chassis_move.chassis_3508_power_set = chassis_move.chassis_power_limit - 10.0f;
			chassis_move.chassis_6020_power_set = 300.0f;
		}
		else
		{
			chassis_move.chassis_3508_power_set = 40.0f;
			chassis_move.chassis_6020_power_set = 200.0f;
		}
 }
	#endif
#else
  chassis_move.chassis_power_set = chassis_move.chassis_power_limit  - 15.0f;//����15w���ʺ�ȫ��������������ת����
#endif
	
		
	if(PowerData[1]>=22.4f)
		{
			color = 2;
		}
		else if(PowerData[1]>= MIN_CAP_VOLTAGE && PowerData[1] < 22.4f)
		{
			color = 3;
		}
		else
		{
			color = 8;
		}
	
}

//�����ת��ת���ڲ�ʱ ��������
int8_t dirt[4] = { -1, 1, 1, -1};
//�������ٶȽ���
static void chassis_vector_to_M3508_wheel_speed(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_speed[4])
{

	wheel_speed[0] = dirt[0] * sqrt(	pow(vy_set + wz_set * MOTOR_DISTANCE_TO_CENTER * SQRT2,2)
                     +	pow(vx_set + wz_set * MOTOR_DISTANCE_TO_CENTER * SQRT2,2)
                     );
  wheel_speed[1] = dirt[1] * sqrt(	pow(vy_set + wz_set * MOTOR_DISTANCE_TO_CENTER * SQRT2,2)
                     +	pow(vx_set - wz_set * MOTOR_DISTANCE_TO_CENTER * SQRT2,2)
                     );
  wheel_speed[2] = dirt[2] * sqrt(	pow(vy_set - wz_set * MOTOR_DISTANCE_TO_CENTER * SQRT2,2)
                     +	pow(vx_set - wz_set * MOTOR_DISTANCE_TO_CENTER * SQRT2,2)
                     );
  wheel_speed[3] = dirt[3] * sqrt(	pow(vy_set - wz_set * MOTOR_DISTANCE_TO_CENTER * SQRT2,2)
                     +	pow(vx_set + wz_set * MOTOR_DISTANCE_TO_CENTER * SQRT2,2) 
                     );
}

//ת����Ŀ��ǶȽ���
void chassis_vector_to_M6020_wheel_angle(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_angle[4])
{

		//6020Ŀ��Ƕȼ��� 6020ecdΪ 0-8192
    if(!(vx_set == 0 && vy_set == 0 && wz_set == 0))//��ֹ����Ϊ��
    {
			//atan2������Ľ���ǻ��� ��Χ-PI-PI�����㻡��Ŀ��ֵ��
      wheel_angle[0] = atan2((vy_set + wz_set * MOTOR_DISTANCE_TO_CENTER * SQRT2),(vx_set + wz_set * MOTOR_DISTANCE_TO_CENTER * SQRT2));		
      wheel_angle[1] = atan2((vy_set + wz_set * MOTOR_DISTANCE_TO_CENTER * SQRT2),(vx_set - wz_set * MOTOR_DISTANCE_TO_CENTER * SQRT2));
      wheel_angle[2] = atan2((vy_set - wz_set * MOTOR_DISTANCE_TO_CENTER * SQRT2),(vx_set - wz_set * MOTOR_DISTANCE_TO_CENTER * SQRT2));
      wheel_angle[3] = atan2((vy_set - wz_set * MOTOR_DISTANCE_TO_CENTER * SQRT2),(vx_set + wz_set * MOTOR_DISTANCE_TO_CENTER * SQRT2));	
    }  
		else
    {
      wheel_angle[0] = atan2(1,1);		
      wheel_angle[1] = atan2(1,-1);		
      wheel_angle[2] = atan2(-1,-1);		
      wheel_angle[3] = atan2(-1,1);
    }  

		//�Ż� �ӻ� �������ת���ж� relative_angle��Χ-PI - PI(ʹ��(PI - 0.01f)����ֹת�����ֱ��ת3508�ӿ�Ч��--�д��ϳ�����)
	  if( (ABS( (fp32)chassis_move.motor_chassis_turn[0].relative_angle - wheel_angle[0] ) > PI/2) && (ABS( (fp32)chassis_move.motor_chassis_turn[0].relative_angle - wheel_angle[0] ) < 3*PI/2))
	 {	
			dirt[0] = 1;
			wheel_angle[0] = Angle_Limit( wheel_angle[0] - PI, PI );
	 }
	 else
		 dirt[0] = -1;
		
	  if( (ABS( (fp32)chassis_move.motor_chassis_turn[1].relative_angle - wheel_angle[1] ) > PI/2) && (ABS( (fp32)chassis_move.motor_chassis_turn[1].relative_angle - wheel_angle[1] ) < 3*PI/2))
	 {	
			dirt[1] = -1;
			wheel_angle[1] = Angle_Limit( wheel_angle[1] - PI, PI );
	 }
	 else
		 dirt[1] = 1;

	  if( (ABS( (fp32)chassis_move.motor_chassis_turn[2].relative_angle - wheel_angle[2] ) > PI/2) && (ABS( (fp32)chassis_move.motor_chassis_turn[2].relative_angle - wheel_angle[2] ) < 3*PI/2))
	 {	
			dirt[2] = -1;
			wheel_angle[2] = Angle_Limit( wheel_angle[2] - PI, PI );
	 }
	 else
		 dirt[2] = 1;

	  if( (ABS( (fp32)chassis_move.motor_chassis_turn[3].relative_angle - wheel_angle[3] ) > PI/2) && (ABS( (fp32)chassis_move.motor_chassis_turn[3].relative_angle - wheel_angle[3] ) < 3*PI/2))
	 {	
			dirt[3] = 1;
			wheel_angle[3] = Angle_Limit( wheel_angle[3] - PI, PI );
	 }
	 else
		 dirt[3] = -1;
}

/**
 * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
 * @retval         none
 */
static void chassis_set_contorl(void)
{
  chassis_behaviour_control_set(&chassis_move);
}
////////////////���ID/////////////////   ////////6020ID///////
/////////1********ǰ*******2///////////   /////3****ǰ****4////
/////////*                 *///////////   /////*   blin   *////   
/////////*        x        *///////////   /////*          *////
/////////��       ����y      ��//////////   /////6****��****5////
/////////*                 *///////////   /////////////////////
/////////*                 *///////////
/////////4********��********3//////////
///////////////////////////////////////


/**
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_control_loop(void)
{
  float max_vector = 0.0f, vector_rate = 0.0f;
  float temp = 0.0f;
  float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float wheel_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t i = 0;
	
  chassis_vector_to_M3508_wheel_speed(chassis_move.vx_set, chassis_move.vy_set, chassis_move.wz_set, wheel_speed);
  chassis_vector_to_M6020_wheel_angle(chassis_move.vx_set, chassis_move.vy_set, chassis_move.wz_set, wheel_angle);

  if (chassis_move.chassis_mode == CHASSIS_OPEN || chassis_move.chassis_mode == CHASSIS_ZERO_FORCE)
  {
    chassis_move.motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
    // raw����ֱ�ӷ���
    return;
  }

  //�������ӿ�������ٶȣ�������������ٶ�
  for (i = 0; i < 4; i++)
  {
		chassis_move.motor_chassis[i].speed_rpm_set = wheel_speed[i]/CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;

    temp = fabs(wheel_speed[i]);
    if (max_vector < temp)
    {
      max_vector = temp;
    }
  }

  if (max_vector > MAX_WHEEL_SPEED)
  {
    vector_rate = MAX_WHEEL_SPEED / max_vector;
    for (i = 0; i < 4; i++)
    {
      chassis_move.motor_chassis[i].speed_rpm_set *= vector_rate;
    }
  }

  //�������ӿ��ƽǶ�,ͬʱ����ǶȻ�
  for (i = 0; i < 4; i++)
  {
    chassis_move.motor_chassis_turn[i].relative_angle_set = wheel_angle[i];
    chassis_move.motor_chassis_turn[i].speed_rpm_set = PID_calc_angle_AGV(&chassis_move.motor_turn_angle_pid[i], chassis_move.motor_chassis_turn[i].relative_angle, chassis_move.motor_chassis_turn[i].relative_angle_set);
  }
	
  calfromPID(&chassis_move);
	calfromPID_6020(&chassis_move);
	//���ʿ���
  if(chassis_move.chassis_power_buffer != 0)
  {
    chassis_power_control(&chassis_move);
		chassis_power_6020_control(&chassis_move);
		
  }

  for (i = 0; i < 4; i++)
  {
    PID_calc(&chassis_move.motor_speed_pid[i], chassis_move.motor_chassis[i].chassis_motor_measure->speed_rpm, chassis_move.motor_chassis[i].speed_rpm_set);
    PID_calc(&chassis_move.motor_turn_speed_pid[i], chassis_move.motor_chassis_turn[i].chassis_motor_measure->speed_rpm, chassis_move.motor_chassis_turn[i].speed_rpm_set);
  }

  //��ֵ����ֵ
  for (i = 0; i < 4; i++)
  {
    chassis_move.motor_chassis[i].give_current = (int16_t)(chassis_move.motor_speed_pid[i].out);
    chassis_move.motor_chassis_turn[i].give_current = (int16_t)(chassis_move.motor_turn_speed_pid[i].out);
  }
}


/**
 * @brief          ����ecd��offset_ecd֮�����ԽǶ�
 * @param[in]      ecd: �����ǰ����
 * @param[in]      offset_ecd: �����ֵ����
 * @retval         ��ԽǶȣ���λrad
 */
static float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
  int32_t relative_ecd = ecd - offset_ecd;
  if (relative_ecd > 4096)
  {
    relative_ecd -= 8191;
  }
  else if (relative_ecd < -4096)
  {
    relative_ecd += 8191;
  }

  return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
 * @brief          �Ƕȷ�Χ����
 * @param[in]      angel: ��ǰ�Ƕ�
 * @param[in]      max: ����������ֵ
 * @retval         ��ԽǶȣ���λrad
 */
static float Angle_Limit(float angle ,float max)
{
		if(angle > max)
			angle -= 2*PI;
		if(angle < -max)
			angle += 2*PI; 
		return angle;
}


