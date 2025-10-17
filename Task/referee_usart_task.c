/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "detect_task.h"
#include "remote_control.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "custom_ui_draw.h"
#include "USART_receive.h"



/**
 * @brief          单字节解包
 * @param[in]      void
 * @retval         none
 */
static void referee_unpack_fifo_data(void);

extern UART_HandleTypeDef huart6;
uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

/*private variable*/
extern uint16_t send_id;
extern uint16_t receive_id;//画UI的标志位

/**
 * @brief          裁判系统任务
 * @param[in]      pvParameters: NULL
 * @param[out]
 * @retval         none
 */
void referee_usart_task(void const *argument)
{
    uint16_t time_heart_ms = 0;

    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
    while (1)
    {
        time_heart_ms += 10;
				recieve_auto = (recieve_auto <= 50) ? (recieve_auto+1) : 50;


        referee_unpack_fifo_data();
        
        draw_get_robot_id();//得到机器人的send_id和receive_id

        if ((!(rc_ctrl.key & KEY_PRESSED_OFFSET_SHIFT)) && rc_ctrl.key & KEY_PRESSED_OFFSET_G)
       {
            draw_init_all(send_id, receive_id);//初始化所有的UI，上电后仅执行一次
       }
			 
        if (time_heart_ms % 180 == 0)
        {
            draw_state_of_top_circle(send_id, receive_id, 2);//画个圆圈，能显示小陀螺模式的开启和关闭状态
						time_heart_ms = 0;
        }
				else if (time_heart_ms % 150 == 0)
        {
            draw_shoot_circle(send_id, receive_id, 2);//画个圆圈，能显示摩擦轮的开启和关闭状态
        }
        else if (time_heart_ms % 120 == 0)
        {
            draw_chassis_mode_state(send_id, receive_id, 2);//显示底盘模式
        }
        else if (time_heart_ms % 90 == 0)
        {
            draw_gimbal_relative_angle_tangle(send_id, receive_id, 2);//显示yaw轴相对角度的情况
        }
				else if (time_heart_ms % 60 == 0)
        {
            draw_auto_circle(send_id, receive_id, 2);
        }
        else if (time_heart_ms % 20 == 0) //50
        {
            draw_capacity_tangle(send_id, receive_id, 2);//显示超级电容的情况
        }
        osDelay(10);
    }
}

/**
 * @brief          单字节解包
 * @param[in]      void
 * @param[out]
 * @retval         none
 */
void referee_unpack_fifo_data(void)
{
    uint8_t byte = 0;
    uint8_t sof = HEADER_SOF;
    unpack_data_t *p_obj = &referee_unpack_obj;

    while (fifo_s_used(&referee_fifo))
    {
        byte = fifo_s_get(&referee_fifo);
        switch (p_obj->unpack_step)
        {
        case STEP_HEADER_SOF:
        {
            if (byte == sof)
            {
                p_obj->unpack_step = STEP_LENGTH_LOW;
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            else
            {
                p_obj->index = 0;
            }
        }
        break;

        case STEP_LENGTH_LOW:
        {
            p_obj->data_len = byte;
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_LENGTH_HIGH;
        }
        break;

        case STEP_LENGTH_HIGH:
        {
            p_obj->data_len |= (byte << 8);
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
            {
                p_obj->unpack_step = STEP_FRAME_SEQ;
            }
            else
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }
        }
        break;
        case STEP_FRAME_SEQ:
        {
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_HEADER_CRC8;
        }
        break;

        case STEP_HEADER_CRC8:
        {
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
            {
                if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
                {
                    p_obj->unpack_step = STEP_DATA_CRC16;
                }
                else
                {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }
        }
        break;

        case STEP_DATA_CRC16:
        {
            if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
            {
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;

                if (verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
                {
                    referee_data_solve(p_obj->protocol_packet);
                }
            }
        }
        break;

        default:
        {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
        }
        break;
        }
    }
}
/**
 * @brief          USART6中断服务程序
 * @param[in]      void
 * @param[out]
 * @retval         none
 */
void USART6_IRQHandler(void)
{
    static volatile uint8_t res;
    if (USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);

        static uint16_t this_time_rx_len = 0;

        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char *)usart6_buf[0], this_time_rx_len);
            detect_hook(REFEREE_TOE);
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char *)usart6_buf[1], this_time_rx_len);
            detect_hook(REFEREE_TOE);
        }
    }
}
