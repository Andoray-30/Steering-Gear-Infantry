/**
 * @file chassis_behaviour.c
 * @author 何清华,bobolin
 * @brief
 * @version 0.1
 * @date 2022-03-17
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */

#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "config.h"
#include "referee.h"
#include "referee_usart_task.h"
#include "chassis_power_control.h"


#define rc_deadband_limit(input, output, dealine)    \
  {                                                  \
    if ((input) > (dealine) || (input) < -(dealine)) \
    {                                                \
      (output) = (input);                            \
    }                                                \
    else                                             \
    {                                                \
      (output) = 0;                                  \
    }                                                \
  }

/**
 * @brief          更新键盘标志位
 * @param[in]      void
 * @retval         none
 */
void key_update(void);

/**
 * @brief          检测遥控器，设置底盘模式
 * @param[in]      rc_ctrl,  包括遥控器接收信息.
 * @retval         none
 */
void chassis_remote_control_mood_set(chassis_move_t *chassis_move_mode);
/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_zero_force_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_move_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_follow_gimbal_yaw_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘小陀螺的行为状态机下，底盘模式是一边旋转一边以云台指向方向运动
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_top_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_follow_yaw_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_open_set_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 * @param[out]     vx_set: 纵向速度指针
 * @param[out]     vy_set: 横向速度指针
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
 * @retval         none
 */
extern void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘模式选择
 * @param[in]      chassis_move_mode: 底盘数据
 * @retval         none
 */

/**
 * @brief          如果遥控器关闭，失能键盘
 * @param[in]      void
 * @retval         返回空
 */
void key_disable(void);

static int up2 = 0, medium2 = 0, down2 = 0; // 分别对应小陀螺，单轴模式，跟随模式
float angle_set_user = 0.0f;                // 一键掉头，90度，目标set记录值
float angle_set = 0.0f;                     // 45度跟随方向值
extern ext_game_robot_state_t robot_state;  // 步兵等级
extern gimbal_behaviour_e gimbal_behaviour_change;
int F_flag = 0, F_flag_last = 0;            // F按键上升沿下降沿判断标志位
int Q_flag = 0, Q_flag_last = 0;
int E_flag = 0, E_flag_last = 0;
int R_flag = 0, R_flag_last = 0;
int V_flag = 0, V_flag_last = 0;
int C_flag = 0, C_flag_last = 0;
int Shift_flag = 0, Shift_flag_last = 0;


extern float min_V;


int up2_last = 0;
// 用于键盘缓启动
float vx_set_use = 0.0;
float vy_set_use = 0.0;
// 用于遥控缓启动
float vx_set_use_rc = 0.0;
float vy_set_use_rc = 0.0;
float yaw_angle_last = 0.0;

int n = 0; // 找不到ui在哪，先定义

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
  if (chassis_move_mode == NULL)
  {
    return;
  }
  // 遥控器模式控制
  chassis_remote_control_mood_set(chassis_move_mode);

  // 根据不同的底盘控制模式调用函数
  chassis_behaviour_control_set(chassis_move_mode);

  // 更新键盘
  key_update();

  // 当云台在某些模式下，像初始化， 底盘不动
  if (gimbal_cmd_to_chassis_stop())
  {
    chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE;
  }
  /****************************F 小陀螺************************************************************/
//  if (F_flag - F_flag_last == 1)
//  {
//    if (up2 == 1 && medium2 == 0 && down2 == 0)
//    {
//      chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
//      up2 = 0;
//      medium2 = 1;
//      down2 = 0;
//    }
//    else
//    {
//      chassis_move_mode->chassis_mode = CHASSIS_TOP;
//      up2 = 1;
//      medium2 = 0;
//      down2 = 0;
//      gimbal_control.gimbal_yaw_motor.absolute_angle_set -= PI / 106.5;
//    }
//  }
	if(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT || switch_is_up(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL]))
	{
			chassis_move_mode->chassis_mode = CHASSIS_TOP;
      up2 = 1;
      medium2 = 0;
      down2 = 0;
		
		if(Shift_flag - Shift_flag_last == 1)
      gimbal_control.gimbal_yaw_motor.absolute_angle_set -= PI / 106.5;
	}
	else
	{
			chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
      up2 = 0;
      medium2 = 1;
      down2 = 0;
	}
  /***************************** V 一键调头 ******************************************************************/
  if (V_flag - V_flag_last == 1)
  {
    angle_set_user = gimbal_control.gimbal_yaw_motor.absolute_angle_set;
    gimbal_control.gimbal_yaw_motor.absolute_angle_set = rad_format(angle_set_user + PI);
  }
  /********************************* Q 左转90 ******************************************************/
  if (Q_flag - Q_flag_last == 1)
  {
    gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle_set + PI / 2.0;
  }
  /***************************************** E 右转90******************************************************/
  if (E_flag - E_flag_last == 1)
  {
    gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle_set - PI / 2.0;
  }
  /*************************************************** R 45度跟随开关 ******************************************************/
  if (R_flag - R_flag_last == 1)
  {
    if (up2 != 1)
    {
      if (angle_set == PI / 4)
      {
        angle_set = 0;
      }
      else if (up2 != 1)
      {
        angle_set = PI / 4;
      }
    }
  }
/*************************************************** C 加速 ******************************************************/
  if (rc_ctrl.key & KEY_PRESSED_OFFSET_C)
  {
    min_V = SUPER_MIN_CAP_VOLTAGE;
  }
	else
	{
		min_V = MIN_CAP_VOLTAGE;
	}
	/*************************************************** C 开启自瞄（切换自瞄与微瞄模式） ******************************************************/
//  if (C_flag - C_flag_last == 1)
//  {
//		if(gimbal_behaviour_change == GIMBAL_AUTO)
//		{
//			gimbal_behaviour_change = PITCH_MOVE;
//		}
//		else
//		{
//			gimbal_behaviour_change = GIMBAL_AUTO;
//		}
//  }

  // 如果遥控器关闭，不使用键盘
  key_disable();

  if (up2 == 1)
    chassis_move_mode->chassis_mode = CHASSIS_TOP; // 小陀螺
  if (medium2 == 1)
    chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL; // 跟随模式
  if (down2 == 1)
    chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_GIMBAL; // 单轴模式
	
	#if PT_link_en
	//及其特殊情况时，使用图传链路的fn1进行软件复位
	if (remote_data_t.fn_1 == 1)
  {
    __set_FAULTMASK(1);//禁止所有的可屏蔽中断
   NVIC_SystemReset();//软件复位
  }
	#endif
}

/**
 * @brief          更新键盘标志位
 * @param[in]      void
 * @retval         none
 */
void key_update(void)
{

  // F键标志位
  F_flag_last = F_flag;
  F_flag = ((!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) && rc_ctrl.key & KEY_PRESSED_OFFSET_F);

  // V键标志位
  V_flag_last = V_flag;
  V_flag = ((!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) && rc_ctrl.key & KEY_PRESSED_OFFSET_V);

  // R键标志位
  R_flag_last = R_flag;
  R_flag = ((!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) && rc_ctrl.key & KEY_PRESSED_OFFSET_R);

  // Q键标志位
  Q_flag_last = Q_flag;
  Q_flag = ((!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) && rc_ctrl.key & KEY_PRESSED_OFFSET_Q);

  // E键标志位
  E_flag_last = E_flag;
  E_flag = ((!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) && rc_ctrl.key & KEY_PRESSED_OFFSET_E);
	
	// C键标志位
  C_flag_last = C_flag;
  C_flag = ((!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) && rc_ctrl.key & KEY_PRESSED_OFFSET_C);
	
	// shift键标志位
  Shift_flag_last = Shift_flag;
  Shift_flag = (rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT);
}

/**
 * @brief          检测遥控器，设置底盘模式
 * @param[in]      rc_ctrl,  包括遥控器接收信息.
 * @retval         none
 */
void chassis_remote_control_mood_set(chassis_move_t *chassis_move_mode)
{
  // 遥控器设置模式(非必要不建议改)
  if (switch_is_up(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) // 上
  {
#ifdef AUTO_DEBUG
    chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
#else
    chassis_move_mode->chassis_mode = CHASSIS_TOP;
#endif
  }
  else if (switch_is_mid(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) // 中
  {
    chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
  }
  else if (switch_is_down(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) // 下
  {
    chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_GIMBAL;
  }
}

/**
 * @brief          设置控制量.根据不同底盘控制模式，会调用不同的控制函数.
 * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
 * @retval         none
 */
void chassis_behaviour_control_set(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_NO_MOVE)
  {
    chassis_no_move_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_FOLLOW_GIMBAL)
  {
    chassis_follow_gimbal_yaw_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_TOP)
  {
    chassis_top_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_NO_FOLLOW_GIMBAL)
  {
    chassis_no_follow_yaw_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_OPEN)
  {
    chassis_open_set_control(chassis_move_rc_to_vector);
  }
  if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_ZERO_FORCE)
  {
    chassis_zero_force_control(chassis_move_rc_to_vector);
  }
}

/**
 * @brief          如果遥控器关闭，失能键盘
 * @param[in]      void
 * @retval         返回空
 */
void key_disable(void)
{
  if (switch_is_down(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL]))
  {
    up2 = 0;
    medium2 = 0;
    down2 = 0;
    F_flag = 0, F_flag_last = 0; // F按键上升沿下降沿判断标志位
    Q_flag = 0, Q_flag_last = 0;
    E_flag = 0, E_flag_last = 0;
    R_flag = 0, R_flag_last = 0;
  }
}

/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_move_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  chassis_move_rc_to_vector->vx_set = 0.0f;
  chassis_move_rc_to_vector->vy_set = 0.0f;
  chassis_move_rc_to_vector->wz_set = 0.0f;
}

/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_follow_gimbal_yaw_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  float vx_set = 0.0f, vy_set = 0.0f;

  // 遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
  chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_rc_to_vector);
  float sin_yaw = 0.0f, cos_yaw = 0.0f;
  // 旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
  sin_yaw = arm_sin_f32(-chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);
  cos_yaw = arm_cos_f32(-chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);

  chassis_move_rc_to_vector->vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
  chassis_move_rc_to_vector->vy_set = sin_yaw * vx_set + cos_yaw * vy_set;

  // 设置控制相对云台角度
  chassis_move_rc_to_vector->chassis_relative_angle_set = rad_format(angle_set);
  chassis_move_rc_to_vector->chassis_relative_angle = rad_format(chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);

  // 计算旋转PID角速度
  chassis_move_rc_to_vector->wz_set = PID_calc(&chassis_move_rc_to_vector->chassis_angle_pid, chassis_move_rc_to_vector->chassis_relative_angle, chassis_move_rc_to_vector->chassis_relative_angle_set);

  // 速度限幅
  chassis_move_rc_to_vector->vx_set = float_constrain(chassis_move_rc_to_vector->vx_set, chassis_move_rc_to_vector->vx_min_speed, chassis_move_rc_to_vector->vx_max_speed);
  chassis_move_rc_to_vector->vy_set = float_constrain(chassis_move_rc_to_vector->vy_set, chassis_move_rc_to_vector->vy_min_speed, chassis_move_rc_to_vector->vy_max_speed);
}

/**
 * @brief          底盘小陀螺的行为状态机下，底盘模式是一边旋转一边以云台指向方向运动
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_top_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  float vx_set = 0.0f, vy_set = 0.0f;

  // 遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
  chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_rc_to_vector);
  float sin_yaw = 0.0f, cos_yaw = 0.0f;
  // 旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
  sin_yaw = arm_sin_f32(-chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);
  cos_yaw = arm_cos_f32(-chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);

  chassis_move_rc_to_vector->vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
  chassis_move_rc_to_vector->vy_set = sin_yaw * vx_set + cos_yaw * vy_set;

  // 设置小陀螺转速
  if (vx_set == 0 & vy_set == 0 || rc_ctrl.key & KEY_PRESSED_OFFSET_F)
  {
    chassis_move_rc_to_vector->wz_set = CHASSIS_TOP_SPEED;
		
		if(remote_data_t.fn_2 == 1)
		{
			chassis_move_rc_to_vector->wz_set = -CHASSIS_TOP_SPEED;
		}
  }
  else
  {
    chassis_move_rc_to_vector->wz_set = CHASSIS_TOP_SPEED;
  }
	
}
/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_follow_yaw_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  float vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

  chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_rc_to_vector);
  wz_set = CHASSIS_WZ_RC_SEN * rc_ctrl.rc.ch[CHASSIS_WZ_CHANNEL];

  chassis_move_rc_to_vector->wz_set = wz_set;
  chassis_move_rc_to_vector->vx_set = float_constrain(vx_set, chassis_move_rc_to_vector->vx_min_speed, chassis_move_rc_to_vector->vx_max_speed);
  chassis_move_rc_to_vector->vy_set = float_constrain(vy_set, chassis_move_rc_to_vector->vy_min_speed, chassis_move_rc_to_vector->vy_max_speed);
}

/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_open_set_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  chassis_move_rc_to_vector->vx_set = rc_ctrl.rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  chassis_move_rc_to_vector->vy_set = rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  chassis_move_rc_to_vector->wz_set = rc_ctrl.rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  return;
}

/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_zero_force_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  chassis_move_rc_to_vector->vx_set = 0.0f;
  chassis_move_rc_to_vector->vy_set = 0.0f;
  chassis_move_rc_to_vector->wz_set = 0.0f;
}

/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 *
 * @param[out]     vx_set: 纵向速度指针
 * @param[out]     vy_set: 横向速度指针
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
 * @retval         none
 */

void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
  {
    return;
  }

  int16_t vx_channel, vy_channel;
  float vx_set_channel, vy_set_channel;

  // 死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
  rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

  //   vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
  //   vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;

  if (vx_set_use_rc < vx_channel * CHASSIS_VX_RC_SEN && vx_channel * CHASSIS_VX_RC_SEN > 0)
  {
    vx_set_use_rc += 0.005f;
    vx_set_channel = vx_set_use_rc;
  }
  else if (vx_set_use_rc > vx_channel * CHASSIS_VX_RC_SEN && vx_channel * CHASSIS_VX_RC_SEN < 0)
  {
    vx_set_use_rc -= 0.005f;
    vx_set_channel = vx_set_use_rc;
  }
  else if(vx_set_use_rc > 0.02f && vx_channel == 0)
  {
    vx_set_use_rc = vx_set_use_rc - 0.02f ;
    vx_set_channel = vx_set_use_rc;
  }
	else if(vx_set_use_rc < -0.02f && vx_channel == 0)
  {
    vx_set_use_rc = vx_set_use_rc + 0.02f ;
    vx_set_channel = vx_set_use_rc;
  }
	else
	{
		vx_set_use_rc = vx_channel * CHASSIS_VX_RC_SEN;
    vx_set_channel = vx_set_use_rc;
	}

  if (vy_set_use_rc < vy_channel * CHASSIS_VY_RC_SEN && vy_channel * CHASSIS_VY_RC_SEN > 0)
  {
    vy_set_use_rc += 0.005f;
    vy_set_channel = vy_set_use_rc;
  }
  else if (vy_set_use_rc > vy_channel * CHASSIS_VY_RC_SEN && vy_channel * CHASSIS_VY_RC_SEN < 0)
  {
    vy_set_use_rc -= 0.005f;
    vy_set_channel = vy_set_use_rc;
  }
	else if(vy_set_use_rc > 0.02f && vy_channel == 0)
  {
    vy_set_use_rc = vy_set_use_rc - 0.02f ;
    vy_set_channel = vy_set_use_rc;
  }
	else if(vy_set_use_rc < -0.02f && vy_channel == 0)
  {
    vy_set_use_rc = vy_set_use_rc + 0.02f ;
    vy_set_channel = vy_set_use_rc;
  }
  else
  {
    vy_set_use_rc = vy_channel * CHASSIS_VY_RC_SEN;
    vy_set_channel = vy_set_use_rc;
  }

  // 左右移动方向暂时不加缓启动
  //   vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;

  // 键盘控制
  if (rc_ctrl.key & KEY_PRESSED_OFFSET_W)
  {
    if (vx_set_channel < chassis_move_rc_to_vector->vx_max_speed)
    {
      vx_set_use += 0.01f;
      vx_set_channel = vx_set_use;
    }
    else
    {
      vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
  }
	
  if (rc_ctrl.key & KEY_PRESSED_OFFSET_S) // 后退
  {
    if (vx_set_channel > chassis_move_rc_to_vector->vx_min_speed)
    {
      vx_set_use -= 0.01f;
      vx_set_channel = vx_set_use;
    }
    else
    {
      vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }
  }
	
	//如果既没有前进，也没有后退，将速度置为0
  if (!(rc_ctrl.key & KEY_PRESSED_OFFSET_W) && !(rc_ctrl.key & KEY_PRESSED_OFFSET_S))
  {
		
		if(vx_set_use > 0.5f)
		{
			vx_set_use = vx_set_use - 0.2f ;
			vx_set_channel = vx_set_use;
		}
		else if(vx_set_use < -0.5f)
		{
			vx_set_use = vx_set_use + 0.2f ;
			vx_set_channel = vx_set_use;
		}
		
		else
    vx_set_use = 0.0f;
  }

  if (rc_ctrl.key & KEY_PRESSED_OFFSET_A) // 左移
  {
    if (vy_set_use > 0.0)
    {
      vy_set_use = 0;
    }
    // 迅速左移
    if (rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)
    {
      if (vy_set_channel < chassis_move_rc_to_vector->vy_max_speed)
      {
        vy_set_use -= 0.003f;
        vy_set_channel = vy_set_use;
      }
      else
      {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
      }
    }
    else
    {
      if (vy_set_channel < chassis_move_rc_to_vector->vy_max_speed)
      {
        vy_set_use -= 0.003f;
        vy_set_channel = vy_set_use;
      }
      else
      {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
      }
    }
  }

  if (rc_ctrl.key & KEY_PRESSED_OFFSET_D) // 右移
  {
    if (vy_set_use < 0.0)
    {
      vy_set_use = 0;
    }
    if (rc_ctrl.key & KEY_PRESSED_OFFSET_CTRL)
    {
      if (vy_set_channel > chassis_move_rc_to_vector->vy_min_speed)
      {
        vy_set_use += 0.008f;
        vy_set_channel = vy_set_use;
      }
      else
      {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
      }
    }
    else
    {
      if (vy_set_channel > chassis_move_rc_to_vector->vy_min_speed)
      {
        vy_set_use += 0.008f;
        vy_set_channel = vy_set_use;
      }
      else
      {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
      }
    }
  }
  if (!(rc_ctrl.key & KEY_PRESSED_OFFSET_D) && !(rc_ctrl.key & KEY_PRESSED_OFFSET_A))
  {
    vy_set_use = 0.0f;
  }

  // 一阶低通滤波代替斜坡作为底盘速度输入（平滑）
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
  // 在死区内，停止信号，不需要缓慢加速，直接减速到零
  if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
  }

  if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
  }

  // *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
  // *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
  *vx_set = vx_set_channel;
  *vy_set = vy_set_channel;
}
