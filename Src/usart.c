/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#include "mdio.h"
uint8_t data = 0;
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USART1_IRQn);

  }
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */
/**
  * 函数功能: 串口接收完成回调函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
//{
////	QUEUE_IN(cli_rx_buff, data);
////  HAL_UART_Transmit(&huart1,&data,1,0);
//	HAL_UART_Receive_IT(&huart1,&data,1);
//}

#define BUFLEN 100
uint8_t receive_buf[BUFLEN];
uint16_t receive_len;

/* 打开uart1接收中断 */
void uart1_start_receive_it(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	huart1.State = HAL_UART_STATE_READY;
	__HAL_DMA_DISABLE(huart1.hdmarx);
	__HAL_DMA_CLEAR_FLAG(rs485.huart->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(huart1.hdmarx));
	__HAL_UNLOCK(huart1.hdmarx);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1, receive_buf, BUFLEN);
}

/* uart1接收处理函数 */
void uart1_process(uint8_t *buf, uint16_t len)
{
	if(len >= BUFLEN - 1)
	{
		printf("command is to long!\r\n");
		return;
	}
	
	buf[len] = '\0';
//	printf("len = %d\n", len);
//	printf("buf = %s\n", buf);
	
	
	if(strncmp("help", (char *)buf, 4) == 0)
	{
		printf("==================================\r\n");
		printf("read  <phyAddr|all> <regAddr|all>\r\n");
		printf("write <phyAddr> <regAddr> <value>\r\n");
		printf("==================================\r\n");
		
	}else if(strncmp("read", (char *)buf, 4) == 0)
	{
		/* read all 00 */
		/* read 00  00 */
		/* read 00  all */
		/* read all all */
		
		/* read */
		strtok((char *)buf, " ");
		
		/* phyad */
		char *pphyad = NULL;
		pphyad = strtok(NULL, " ");
		if(NULL == pphyad)
		{
			printf("read: para error!\r\n");
			printf("read  <phyAddr|all> <regAddr|all>\r\n");
			return;
		}
		
		/* regad */
		char *pregad = NULL;
		pregad = strtok(NULL, " ");
		if(NULL == pregad)
		{
			printf("read: para error!\r\n");
			printf("read  <phyAddr|all> <regAddr|all>\r\n");
			return;
		}
		
		if(strncmp("all", pphyad, 3) == 0)
		{
			if(strncmp("all", pregad, 3) == 0)
			{
				uint16_t value = 0;
				for(int i = 0; i < 32; i++)
				{
					for(int j = 0; j < 32; j++)
					{
						value = read_data(i, j);
						printf("Read phy = 0x%02X reg = 0x%02X value = 0x%04X\r\n", i, j, value);
//						HAL_Delay(10);
					}
				}
			}else
			{
				
				uint32_t regad = 0;
				sscanf(pregad, "%x", &regad);
				
				uint16_t value = 0;
				for(int i = 0; i < 32; i++)
				{
					value = read_data(i, regad);
					printf("Read phy = 0x%02X reg = 0x%02X value = 0x%04X\r\n", i, regad, value);
				}
			}
			
		}
		else
		{
			uint32_t phyad = 0;
	//		printf("p = %s\n", p);
			sscanf(pphyad, "%x", &phyad);
	//		printf("regad = %x\n", regad);
			
			if(strncmp("all", pregad, 3) == 0)
			{
				uint16_t value = 0;
				for(int i = 0; i < 32; i++)
				{
					value = read_data(phyad, i);
					printf("Read phy = 0x%02X reg = 0x%02X value = 0x%04X\r\n", phyad, i, value);
//					HAL_Delay(1);
				}
			}else
			{
				
				uint32_t regad = 0;
				sscanf(pregad, "%x", &regad);
				
				uint16_t value = 0;

				value = read_data(phyad, regad);
				printf("Read phy = 0x%02X reg = 0x%02X value = 0x%04X\r\n", phyad, regad, value);
			}
		}
		
	}else if(strncmp("write", (char *)buf, 5) == 0)
	{
		/*  */
		/* write */
		
		strtok((char *)buf, " ");
		
		/* phyad */
		char *pphyad = NULL;
		pphyad = strtok(NULL, " ");
		if(NULL == pphyad)
		{
			printf("write: para error!\r\n");
			printf("write <phyAddr> <regAddr> <data>\r\n");
			return;
		}
		
		/* regad */
		char *pregad = NULL;
		pregad = strtok(NULL, " ");
		if(NULL == pregad)
		{
			printf("write: para error!\r\n");
			printf("write <phyAddr> <regAddr> <data>\r\n");
			return;
		}
		
		/* value */
		char *pvalue = NULL;
		pvalue = strtok(NULL, " ");
		if(NULL == pvalue)
		{
			printf("write: para error!\r\n");
			printf("write <phyAddr> <regAddr> <data>\r\n");
			return;
		}
		
		uint32_t phyad = 0;
		sscanf(pphyad, "%x", &phyad);
		
		uint32_t regad = 0;
		sscanf(pregad, "%x", &regad);
		
		
		uint32_t value = 0;
		sscanf(pvalue, "%x", &value);
		
		write_data(phyad, regad, value);
		printf("Write phy = 0x%02X reg = 0x%02X value = 0x%04X\r\n", phyad, regad, value);
		
		value = read_data(phyad, regad);
		printf("Read  phy = 0x%02X reg = 0x%02X value = 0x%04X\r\n", phyad, regad, value);

	}
	printf("\r\n");
	
}


/* uart1接收函数 */
void uart1_receive(void)
{
	/* 接收到的数据长度 */
	receive_len = BUFLEN - huart1.hdmarx->Instance->CNDTR;

	/* uart1接收处理函数 */
	uart1_process(receive_buf, receive_len);
	
//	debug_data("485_rx", rs485.rs485_message.buf, rs485.rs485_message.len);
	
	/* 开始串口空闲中断DMA接收 */
	uart1_start_receive_it();
}


/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
