/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */
    /**SPI1 GPIO Configuration    
    PB3     ------> SPI1_SCK
	PB4     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
	/**SPI2 GPIO Configuration
	PB12     ------> SPI2_CS	
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
	/* ���Ŷ��� */
	#define SPI_SS_PIN   GPIO_PIN_12
	#define SPI_SCK_PIN  GPIO_PIN_13
	#define SPI_MISO_PIN GPIO_PIN_14
	#define SPI_MOSI_PIN GPIO_PIN_15
	#define SPI_PORT     GPIOB
/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration    
    PB4     ------> SPI1_MISO
    PB3     ------> SPI1_SCK
    PA7     ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 DMA Init */
    /* SPI1_RX Init */
    hdma_spi1_rx.Instance = DMA2_Stream2;
    hdma_spi1_rx.Init.Channel = DMA_CHANNEL_3;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi1_rx);

    /* SPI1_TX Init */
    hdma_spi1_tx.Instance = DMA2_Stream3;
    hdma_spi1_tx.Init.Channel = DMA_CHANNEL_3;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi1_tx);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration    
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */
	/* ����Ĭ�ϵ�ƽ */
    MySPI_W_SS(1);   // SSĬ�ϸߵ�ƽ
    MySPI_W_SCK(0);  // SCKĬ�ϵ͵�ƽ
  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();
  
    /**SPI1 GPIO Configuration    
    PB4     ------> SPI1_MISO
    PB3     ------> SPI1_SCK
    PA7     ------> SPI1_MOSI 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4|GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);

    /* SPI1 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();
  
    /**SPI2 GPIO Configuration    
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */
	/* ����Ĭ�ϵ�ƽ */
    MySPI_W_SS(1);   // SSĬ�ϸߵ�ƽ
    MySPI_W_SCK(0);  // SCKĬ�ϵ͵�ƽ
  /* USER CODE END SPI2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/**
  * @brief  ����SS���ŵ�ƽ
  * @param  BitValue: Ҫ���õĵ�ƽ (0��1)
  * @retval None
  */
void MySPI_W_SS(uint8_t BitValue)
{
    HAL_GPIO_WritePin(SPI_PORT, SPI_SS_PIN, (BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

/**
  * @brief  ����SCK���ŵ�ƽ
  * @param  BitValue: Ҫ���õĵ�ƽ (0��1)
  * @retval None
  */
void MySPI_W_SCK(uint8_t BitValue)
{
    HAL_GPIO_WritePin(SPI_PORT, SPI_SCK_PIN, (BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

/**
  * @brief  ����MOSI���ŵ�ƽ
  * @param  BitValue: Ҫ���õĵ�ƽ (0���0)
  * @retval None
  */
void MySPI_W_MOSI(uint8_t BitValue)
{
    HAL_GPIO_WritePin(SPI_PORT, SPI_MOSI_PIN, (BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

/**
  * @brief  ��ȡMISO���ŵ�ƽ
  * @param  None
  * @retval MISO���ŵ�ƽ (0��1)
  */
uint8_t MySPI_R_MISO(void)
{
    return (HAL_GPIO_ReadPin(SPI_PORT, SPI_MISO_PIN) == GPIO_PIN_SET ? 1 : 0);
}
/**
  * @brief  SPI��ʼ�ź�
  * @param  None
  * @retval None
  */
extern void SPI_START(void)//�͵�ƽ����
{
	HAL_GPIO_WritePin(SPI_PORT,SPI_SCK_PIN, GPIO_PIN_RESET);
}
/**
  * @brief  SPI��ֹ�ź�
  * @param  None
  * @retval None
  */
extern void SPI_STOP(void)//�ߵ�ƽ����
{
	HAL_GPIO_WritePin(GPIOB,SPI_SCK_PIN,GPIO_PIN_SET);
}

/**
  * @brief  SPI����һ���ֽ�(SPIģʽ0)
  * @param  ByteSend: Ҫ���͵��ֽ�
  * @retval ���յ����ֽ�
  */
uint8_t MySPI_SwapByte(uint8_t ByteSend)
{
    uint8_t i, ByteReceive = 0x00;
    
    for (i = 0; i < 8; i++)
    {
        MySPI_W_MOSI(ByteSend & (0x80 >> i));//�Ƴ�����
        MySPI_W_SCK(1);  // SCK������
        if (MySPI_R_MISO() == 1) { ByteReceive |= (0x80 >> i); }//��������
        MySPI_W_SCK(0);  // SCK�½���
    }
    
    return ByteReceive;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
