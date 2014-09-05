/**
  ******************************************************************************
  * @file    stm32f10x_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    07/16/2010 
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "main.h"
#include "string.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#define UARTRCV_INT 1
#define UARTSEND_INT 1

#define SINPORT_0_PE0 (2)									/*	定义SIN模块端口对应的SIN输入线	*/
#define SINPORT_1_PE1 (1)									/*	PE1	-	>	SIN1									*/
#define SINPORT_2_PE2 (4)									/*	PE2	-	>	SIN2									*/
#define SINPORT_3_PE3 (5)
#define SINPORT_4_PE4 (6)
#define SINPORT_5_PE5 (7)
#define SINPORT_6_PE6 (0)
#define SINPORT_7_PE7 (3)
#define SINPORT_8_PD8 (10)
#define SINPORT_9_PD9 (11)
#define SINPORT_10_PD10 (8)
#define SINPORT_11_PD11 (9)
#define SINPORT_12_PD12 (15)
#define SINPORT_13_PD13 (14)
#define SINPORT_14_PD14 (13)
#define SINPORT_15_PD15 (12)


extern uint16_t frequency_s[16];
extern uint16_t delay100Ms;
extern uint8_t ReceiveOK;
extern uint8_t ReceiveBuffer[64];
extern uint8_t ReceiveBufferHead;
extern uint8_t ReceiveBufferTail;
extern uint8_t ReceiveBufferLen;
extern uint8_t ReceiveBufferIndex;
extern uint8_t ReceiveStart;
extern uint8_t SendOK;

uint8_t data[136];
uint8_t SendIndex;
/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/




/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{}


/*********************************************************************************************
* name			:		TIM2_IRQHandler
* func			:		处理定时器中断事件（每1ms中断一次）
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void TIM2_IRQHandler(void)
{
	
	if(TIM_GetITStatus(TIM2,TIM_FLAG_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);//清中断
		if(delay100Ms)
			delay100Ms=0;
	}
}
	
/*********************************************************************************************
* name			:		SysTick_Handler
* func			:		处理定时器中断事件（每1秒中断一次）
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/*********************************************************************************************
* name			:		EXTI_IRQHandler
* func			:		处理响应中断线路的中断事件
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void EXTI_IRQHandler(uint8_t num)
{
	frequency_s[num]+=1;//每中断一次，相应引脚中断次数加1
}
/*********************************************************************************************
* name			:		EXTIx_IRQHandler
* func			:		处理响应中断线路的中断事件
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)
	{
		EXTI_IRQHandler(SINPORT_0_PE0);
		EXTI_ClearITPendingBit(EXTI_Line0);
		
	}
}
void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1)!=RESET)
	{
		EXTI_IRQHandler(SINPORT_1_PE1);
		EXTI_ClearITPendingBit(EXTI_Line1);
		
	}
}
void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2)!=RESET)
	{
		EXTI_IRQHandler(SINPORT_2_PE2);
		EXTI_ClearITPendingBit(EXTI_Line2);
		
	}
}
void EXTI3_IRQHandler(void)
{
	
	if(EXTI_GetITStatus(EXTI_Line3)!=RESET)
	{
		EXTI_IRQHandler(SINPORT_3_PE3);
		EXTI_ClearITPendingBit(EXTI_Line3);
		
	}
}
void EXTI4_IRQHandler(void)
{
	
	if(EXTI_GetITStatus(EXTI_Line4)!=RESET)
	{
		EXTI_IRQHandler(SINPORT_4_PE4);
		EXTI_ClearITPendingBit(EXTI_Line4);
		
	}
}
void EXTI9_5_IRQHandler(void)
{
	uint8_t i;
	for(i=5;i<=9;i++)//找出是哪个引脚引起的中断
	{
		if(EXTI_GetITStatus(1<<i)!=RESET)
		{
			
			switch(i)
			{
				case 5:
					EXTI_IRQHandler(SINPORT_5_PE5);
					break;
				case 6:
					EXTI_IRQHandler(SINPORT_6_PE6);
					break;
				case 7:
					EXTI_IRQHandler(SINPORT_7_PE7);
					break;
				case 8:
					EXTI_IRQHandler(SINPORT_8_PD8);
					break;
				case 9:
					EXTI_IRQHandler(SINPORT_9_PD9);
					break;
				default:
					break;
			}
			EXTI_ClearITPendingBit(1<<i);
		}
	}
}
void EXTI15_10_IRQHandler(void)
{
	uint8_t i;
	for(i=10;i<=15;i++)//找出是哪个引脚引起的中断
	{
		if(EXTI_GetITStatus(1<<i)!=RESET)
		{
			
			switch(i)
			{
				case 10:
					EXTI_IRQHandler(SINPORT_10_PD10);
					break;
				case 11:
					EXTI_IRQHandler(SINPORT_11_PD11);
					break;
				case 12:
					EXTI_IRQHandler(SINPORT_12_PD12);
					break;
				case 13:
					EXTI_IRQHandler(SINPORT_13_PD13);
					break;
				case 14:
					EXTI_IRQHandler(SINPORT_14_PD14);
					break;
				case 15:
					EXTI_IRQHandler(SINPORT_15_PD15);
					break;
				default:
					break;
			}
			EXTI_ClearITPendingBit(1<<i);
		}
	}
}


/*********************************************************************************************
* name			:		USART1_IRQHandler
* func			:		USART1接收到数据时产生的中断
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void USART1_IRQHandler(void)
{	
	/*读取中断状态位数据*/
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除接收中断位
		ReceiveBuffer[ReceiveBufferIndex]=USART_ReceiveData(USART1);//读当前串口数据
		ReceiveBufferIndex+=1;
		ReceiveBufferIndex&=0x3f;//游标大小不能超过63
		if(ReceiveOK==0)
		{
			if(ReceiveBuffer[ReceiveBufferIndex-1]==0x5b)//如果找到了头
			{
				ReceiveBufferHead=ReceiveBufferIndex-1;//设定头标记
			}
			else if(ReceiveBuffer[ReceiveBufferIndex-1]==0xb5)//如果读到了尾
			{
				ReceiveBufferTail=ReceiveBufferIndex-1;//设定尾标记
				if(ReceiveBufferTail-ReceiveBufferHead==ReceiveBufferLen)
				{
					ReceiveOK=1;//告诉主函数收包成功
				}
				else//丢弃包，重新初始化
				{
					ReceiveOK=0;
					ReceiveBufferHead=0;
					ReceiveBufferTail=0;
					ReceiveBufferLen=7;
					ReceiveBufferIndex=0;
				}
			}
			else
			{
				
			}
		}
		else if(ReceiveOK==1)
		{
			
		}
		
	}
#ifdef UARTSEND_INT
	else if(USART_GetITStatus(USART1,USART_IT_TXE)!=RESET)
	{
		SendIndex+=1;
		USART_ClearITPendingBit(USART1,USART_IT_TXE);//清除接收中断位
		USART_SendData(USART1,data[SendIndex]);
		if(SendIndex>=135)
		{
			USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
			SendOK=1;
			SendIndex=0;
		}
	}
#endif
	
	//溢出-如果发生溢出需要先读SR,再读DR寄存器 则可清除不断入中断的问题
  if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)==SET)
  {
     USART_ClearFlag(USART1,USART_FLAG_ORE);    //读SR
     ReceiveBuffer[ReceiveBufferIndex]=USART_ReceiveData(USART1);                //读DR
  }
}


/*********************************************************************************************
* name			:		Usart_Send_Struct
* func			:		USART1通过中断发送结构体
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void Usart_Send_Struct(DataStruct dstruct)
{
	uint8_t len;
#ifndef UARTSEND_INT
	uint8_t i;
#endif
	SendIndex=0;
	SendOK=0;
	len=sizeof(dstruct);
	memcpy(data,&dstruct,len);
	
#ifdef UARTSEND_INT
	
	USART_SendData(USART1,data[SendIndex]);
	USART_ITConfig(USART1,USART_IT_TXE,ENABLE);
#else
	for(i=0;i<len;i++)
	{
			USART_SendData(USART1,data[i]);
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	}
#endif
	
}
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

