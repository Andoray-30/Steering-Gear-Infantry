/**
 * @file USART_receive.c
 * @author 何清华,bobolin
 * @brief 串口中断接收函数，用来处理单片机与其他设备的串口通信数据
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "USART_receive.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "main.h"
#include "detect_task.h"
#include "referee.h"
#include "string.h"
#include "gimbal_task.h"
#include "INS_task.h"


extern UART_HandleTypeDef huart1;
extern ext_game_robot_state_t robot_state;  //步兵等级

//auto_shoot_t auto_shoot = {0, 0.0f, 0.0f}; //自瞄数据
auto_shoot_t auto_shoot; //= {0, 0.0f, 0.0f}; //自瞄数据
user_send_data_t user_send_data;
auto_aim_data_t auto_aim_data;

int16_t c_switch = 7;

//接收原始数据，为6个字节，给了12个字节长度，防止DMA传输越界
uint8_t usart1_rx_buf[2][USART1_RX_BUF_NUM];
uint8_t recieve_auto = 0;


/**
 * @brief 用户数据解包
 *
 * @param buf 串口接收数据指针
 * @param auto_shoot 自瞄数据结构指针
 */
void user_data_solve(uint8_t *buf, auto_shoot_t *auto_shoot);

void user_usart_init(void)
{
    usart1_init(usart1_rx_buf[0], usart1_rx_buf[1], USART1_RX_BUF_NUM);
}

void USART1_IRQHandler(void)
{
    if (huart1.Instance->SR & UART_FLAG_RXNE) //接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if (USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
            //失效DMA
            __HAL_DMA_DISABLE(huart1.hdmarx);
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = USART1_RX_BUF_NUM - huart1.hdmarx->Instance->NDTR;
            //重新设定数据长度
            huart1.hdmarx->Instance->NDTR = USART1_RX_BUF_NUM;
            //设定缓冲区1
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            //使能DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);

            if (this_time_rx_len == USER_FRAME_LENGTH)
            {
                //读取数据
                user_data_solve(usart1_rx_buf[0], &auto_shoot);
								recieve_auto = 0;
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //失效DMA
            __HAL_DMA_DISABLE(huart1.hdmarx);
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = USART1_RX_BUF_NUM - huart1.hdmarx->Instance->NDTR;
            //重新设定数据长度
            huart1.hdmarx->Instance->NDTR = USART1_RX_BUF_NUM;
            //设定缓冲区0
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            //使能DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);
            if (this_time_rx_len == USER_FRAME_LENGTH)
            {
                //读取数据
                user_data_solve(usart1_rx_buf[1], &auto_shoot);
								recieve_auto = 0;

            }
        }
    }
}

/**
 * @brief 用户数据解包
 *
 * @param buf 串口接收数据指针
 * @param auto_shoot 自瞄数据结构指针
 */

void user_data_solve(uint8_t *buf, auto_shoot_t *auto_shoot)
{
    //校验
    if (buf[0] == 0xFF && buf[15] == 0xFE)
    {
       memcpy(&auto_shoot->fire_flag, buf+1, sizeof(uint8_t));
			 memcpy(&auto_shoot->yaw_add, buf+2, sizeof(float));
       memcpy(&auto_shoot->pitch_add, buf+6, sizeof(float));
    }

}
//自瞄发送
void user_data_pack_handle()
{
    static uint8_t tx_buf[sizeof(user_send_data_t)];
	 
	user_send_data.header=0xFF;
	//己方红方，发送1，己方蓝方，发送0
	if(get_robot_id()<50)
	{
		user_send_data.detect_color = 1;
  }
	else
	{
		user_send_data.detect_color = 0;

	}
	user_send_data.reset_tracker = 0;
	user_send_data.reserved = 0;
	
	user_send_data.pitch = INS_data.angle_pitch;//INS_data.angle_yaw;
	user_send_data.yaw = INS_data.angle_yaw;//INS_data.angle_yaw;
    user_send_data.end=0XFE;
	user_send_data.d1=0;
	user_send_data.d2=0;
	user_send_data.d3=0;

		
	memcpy(tx_buf , &user_send_data ,sizeof(user_send_data_t));
	append_CRC16_check_sum(tx_buf , sizeof(user_send_data_t));
		
	usart1_tx_dma_enable(tx_buf ,sizeof(user_send_data_t));
}




