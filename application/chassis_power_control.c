/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     bobolin         1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"


/**
 * @brief          检测转速异常数据
 * @param[in]      a+b+c
 * @retval         none
 */
float LIMIT_MAX_MIN(float x, float max, float min);

/**
 * @brief          转向轮限制功率
 * @param[in]      chassis_power_control: 底盘数据
 * @retval         none
 */
void chassis_power_control_turn(chassis_move_t *chassis_power_control)
{
    float limit_k = 0;
    if (chassis_power_control->chassis_power_buffer > 60.0f && chassis_power_control->chassis_power_buffer <= 250.0f)
    {
        limit_k = 1;
    }
    else
    {
        limit_k = 0.25f * ((float)chassis_power_control->chassis_power_buffer / 60.0f) * ((float)chassis_power_control->chassis_power_buffer / 60.0f) *
                 ((float)chassis_power_control->chassis_power_buffer / 60.0f) +
             0.5f * ((float)chassis_power_control->chassis_power_buffer / 60.0f) *
                        ((float)chassis_power_control->chassis_power_buffer / 60.0f) +
             0.25f * ((float)chassis_power_control->chassis_power_buffer / 60.0f);
    }
    chassis_power_control->motor_turn_speed_pid[0].out *= limit_k;
	chassis_power_control->motor_turn_speed_pid[1].out *= limit_k;
	chassis_power_control->motor_turn_speed_pid[2].out *= limit_k;
	chassis_power_control->motor_turn_speed_pid[3].out *= limit_k; 
}

/**
 * @brief          驱动轮限制功率
 * @param[in]      chassis_power_control: 底盘数据
 * @retval         none
 */
void chassis_power_control(chassis_move_t *chassis_power_control)
{
    // 发送期望转速之后产生的功率预测
    // abc为二次函数系数
    float a = 0, b = 0, c = 0;
    for (int i = 0; i < 4; i++)
    {
        a += MOTOR_3508_R * chassis_power_control->k_3508[i] * chassis_power_control->k_3508[i] * chassis_power_control->motor_chassis[i].speed_rpm_set * chassis_power_control->motor_chassis[i].speed_rpm_set;
        b += chassis_power_control->motor_chassis[i].speed_rpm_set * (2 * MOTOR_3508_R * chassis_power_control->k_3508[i] * chassis_power_control->m_3508[i] +
                                                                      MOTOR_3508_K * chassis_power_control->k_3508[i] * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm);
        c += (MOTOR_3508_R * chassis_power_control->m_3508[i] * chassis_power_control->m_3508[i] +
              MOTOR_3508_K * chassis_power_control->m_3508[i] * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm);
    }
    c += MOTOR_3508_P;
    // 这里相当于k_c=1时，预测功率 ,也就是当转速不衰减时，预测功率
    chassis_power_control->predict_send_power_3508 = LIMIT_MAX_MIN(a + b + c, 5000, -5000); // 过滤转速预测异常数据
    // 超功率衰减
    if (chassis_power_control->predict_send_power_3508 > chassis_power_control->chassis_3508_power_set)
    {
        // 模型衰减计算
        if (b * b < 4 * (c - chassis_power_control->chassis_3508_power_set) * a) // b方小于4ac：没有解
        {
            chassis_power_control->limit_k_3508 = LIMIT_MAX_MIN(-b / (2 * a), 1.0, 0.0);
        }
        else
        {
            float32_t sqrt_result;
            arm_sqrt_f32(b * b - 4 * (c - chassis_power_control->chassis_3508_power_set) * a, &sqrt_result);
					if(a != 0)
            chassis_power_control->limit_k_3508 = LIMIT_MAX_MIN((-b + sqrt_result) / 2 / a, 1.0, 0.0);
					else
						chassis_power_control->limit_k_3508 = 0.0f;
        }
        // 衰减
        for (int i = 0; i < 4; i++)
        {
            chassis_power_control->motor_chassis[i].speed_rpm_set *= chassis_power_control->limit_k_3508; // 衰减

        }
    }
}

/**
 * @brief          对于x进行限制
 * @param[in]      a+b+c
 * @retval         none
 */
float LIMIT_MAX_MIN(float x, float max, float min)
{
    if (x > max)
    {
        return max;
    }
    else if (x < min)
    {
        return min;
    }
    else
    {
        return x;
    }
}

/*
 *calfromPID
 *从pid控制算法中反解转速和电流映射关系常数K，M
 */
void calfromPID(chassis_move_t *chassis_power_control)
{
    float k_s_min;
    k_s_min = 1.0;
    for (int i = 0; i < 4; i++)
    {
        pid_type_def *P = &(chassis_power_control->motor_speed_pid[i]);
        short speed_now = chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm;
        float PreError = P->set - speed_now;
			
			  float SumError_last = P->error[0];

				if(P->Ki!=0)
        SumError_last = P->Iout / P->Ki;
				
        float SumError = SumError_last;

        SumError += (P->error[0] + P->error[1]) / 2; // 梯形积分提高精度

        if (fabs(SumError) >= P->max_iout) // 不积分的情况
        {
            chassis_power_control->m_3508[i] = (-P->Kp * speed_now + P->Ki * SumError_last) * RM3508_CURRENT_RATIO;
            chassis_power_control->k_3508[i] = P->Kp * RM3508_CURRENT_RATIO;
        }
        else
        {
            chassis_power_control->m_3508[i] = (-P->Kp * speed_now + P->Ki * (SumError_last + 0.5 * (-speed_now + P->error[0]))) * RM3508_CURRENT_RATIO;
            chassis_power_control->k_3508[i] = (P->Kp + P->Ki / 2) * RM3508_CURRENT_RATIO;
        }
        double k_s = 1.0;
        double K_W = chassis_power_control->k_3508[i] * P->set;
        double current = K_W * k_s + chassis_power_control->m_3508[i];
        if (fabs(current) > chassis_power_control -> motor_speed_pid[i].max_out * RM3508_CURRENT_RATIO)
        {
            double l = current > 0 ? 1 : -1;
            k_s = LIMIT_MAX_MIN((l * chassis_power_control -> motor_speed_pid[i].max_out * RM3508_CURRENT_RATIO - chassis_power_control->m_3508[i]) /K_W,
                                1.0, 0.0);
            if (k_s < k_s_min)
            {
                k_s_min = k_s;
            }
        }
    }
    for (int i = 0; i < 4; i++)
    {
        chassis_power_control->motor_speed_pid[i].set *= k_s_min; // 最大值越界衰减
    }
}

/*
 *calfromPID
 *从pid控制算法中反解转速和电流映射关系常数K，M
 */
void calfromPID_6020(chassis_move_t *chassis_power_control)
{
    float k_s_min;
    k_s_min = 1.0;
    for (int i = 0; i < 4; i++)
    {
        pid_type_def *P = &(chassis_power_control->motor_turn_speed_pid[i]);
        short speed_now = chassis_power_control->motor_chassis_turn[i].chassis_motor_measure->speed_rpm;
        float PreError = P->set - speed_now;
			
			  float SumError_last = P->error[0];

				if(P->Ki!=0)
        SumError_last = P->Iout / P->Ki;
				
        float SumError = SumError_last;

        SumError += (P->error[0] + P->error[1]) / 2; // 梯形积分提高精度

        if (fabs(SumError) >= P->max_iout) // 不积分的情况
        {
            chassis_power_control->m_6020[i] = (-P->Kp * speed_now + P->Ki * SumError_last) * RM6020_CURRENT_RATIO;
            chassis_power_control->k_6020[i] = P->Kp * RM6020_CURRENT_RATIO;
        }
        else
        {
            chassis_power_control->m_6020[i] = (-P->Kp * speed_now + P->Ki * (SumError_last + 0.5 * (-speed_now + P->error[0]))) * RM6020_CURRENT_RATIO;
            chassis_power_control->k_6020[i] = (P->Kp + P->Ki / 2) * RM6020_CURRENT_RATIO;
        }
        double k_s = 1.0;
        double K_W = chassis_power_control->k_6020[i] * P->set;
        double current = K_W * k_s + chassis_power_control->m_6020[i];
        if (fabs(current) > chassis_power_control -> motor_turn_speed_pid[i].max_out * RM6020_CURRENT_RATIO)
        {
            double l = current > 0 ? 1 : -1;
            k_s = LIMIT_MAX_MIN((l * chassis_power_control -> motor_turn_speed_pid[i].max_out * RM6020_CURRENT_RATIO - chassis_power_control->m_6020[i]) /K_W,
                                1.0, 0.0);
            if (k_s < k_s_min)
            {
                k_s_min = k_s;
            }
        }
    }
    for (int i = 0; i < 4; i++)
    {
        chassis_power_control->motor_turn_speed_pid[i].set *= k_s_min; // 最大值越界衰减
    }
}


/**
 * @brief          驱动轮限制功率
 * @param[in]      chassis_power_control: 底盘数据
 * @retval         none
 */
void chassis_power_6020_control(chassis_move_t *chassis_power_control)
{
    // 发送期望转速之后产生的功率预测
    // abc为二次函数系数
    float a = 0, b = 0, c = 0;
    for (int i = 0; i < 4; i++)
    {
        a += MOTOR_6020_R * chassis_power_control->k_6020[i] * chassis_power_control->k_6020[i] * chassis_power_control->motor_chassis_turn[i].speed_rpm_set * chassis_power_control->motor_chassis_turn[i].speed_rpm_set;
        b += chassis_power_control->motor_chassis_turn[i].speed_rpm_set * (2 * MOTOR_6020_R * chassis_power_control->k_6020[i] * chassis_power_control->m_6020[i] +
                                                                      MOTOR_6020_K * chassis_power_control->k_6020[i] * chassis_power_control->motor_chassis_turn[i].chassis_motor_measure->speed_rpm);
        c += (MOTOR_6020_R * chassis_power_control->m_6020[i] * chassis_power_control->m_6020[i] +
              MOTOR_6020_K * chassis_power_control->m_6020[i] * chassis_power_control->motor_chassis_turn[i].chassis_motor_measure->speed_rpm);
    }
    c += MOTOR_6020_P;
    // 这里相当于k_c=1时，预测功率 ,也就是当转速不衰减时，预测功率
    chassis_power_control->predict_send_power_6020 = LIMIT_MAX_MIN(a + b + c, 1000, -1000); // 过滤转速预测异常数据
    // 超功率衰减
    if (chassis_power_control->predict_send_power_6020 > chassis_power_control->chassis_6020_power_set)
    {
        // 模型衰减计算
        if (b * b < 4 * (c - chassis_power_control->chassis_6020_power_set) * a) // b方小于4ac：没有解
        {
            chassis_power_control->limit_k_6020 = LIMIT_MAX_MIN(-b / (2 * a), 1.0, 0.0);
        }
        else
        {
            float32_t sqrt_result;
            arm_sqrt_f32(b * b - 4 * (c - chassis_power_control->chassis_6020_power_set) * a, &sqrt_result);
					if(a != 0)
            chassis_power_control->limit_k_6020 = LIMIT_MAX_MIN((-b + sqrt_result) / 2 / a, 1.0, 0.0);
					else
						chassis_power_control->limit_k_6020 = 0.0f;
        }
        // 衰减
        for (int i = 0; i < 4; i++)
        {
            chassis_power_control->motor_chassis_turn[i].speed_rpm_set *= chassis_power_control->limit_k_6020; // 衰减

        }
    }
}
