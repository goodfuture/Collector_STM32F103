/*********************************************************************************************
* File name	: system_stm32f10x.c
* HardWare	: STM32F103VBT 
* Author		: Lius
* History		:	none
*	Data			:	2014-06-07
* Descript	: STM32F103VBT 采集器，初始化芯片相关设置
*********************************************************************************************/

#include "stm32f10x.h"
#include "main.h"

/*****************定义系统时钟频率**********************/   
/* #define SYSCLK_FREQ_24MHz  24000000 */
/* #define SYSCLK_FREQ_36MHz  36000000 */
/* #define SYSCLK_FREQ_48MHz  48000000 */
/* #define SYSCLK_FREQ_60MHz  60000000 */
	 #define SYSCLK_FREQ_72MHz  72000000 

/****************ADC1数据寄存器基地址*******************/
#define ADC1_DR_Address     0x4001244C

/******************extern data**************************/
extern uint16_t adcValue[10][16];//每个通道采集10次，一共16个通道

/**************************函数声明***********************/
void RCC_Configuration(void);//配置系统时钟
void GPIO_Configuration(void);//设定gpio配置
void USART1_Configuration(void);//初始化USART
void EXTI_Configuration(void);//初始化中断通道
void NVIC_Configuration(void);//初始化中断向量
void TIM_Configuration(void);//定时器初始化配置
void ADC1_Configuration(void);//初始化ADC1；
//void DMA_Configuration(void);//初始化DMA

/*********************************************************************************************
* name			:		SystemInit
* func			:		系统默认初始化函数
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void SystemInit (void)
{
  /* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
  /* Configure the Flash Latency cycles and enable prefetch buffer */
	RCC_Configuration();//初始化系统时钟
	GPIO_Configuration();
}

/*********************************************************************************************
* name			:		GPIO_Configuration
* func			:		初始化用到的GPIO端口
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void GPIO_Configuration(void)
{
	/*********定义端口设置结构体，用来配置引脚*******************/	
	GPIO_InitTypeDef GPIO_InitStructA;//定义gpioA端口设置结构体
	GPIO_InitTypeDef GPIO_InitStructB;//定义gpioB端口设置结构体
	GPIO_InitTypeDef GPIO_InitStructC;//定义gpioC端口设置结构体
	GPIO_InitTypeDef GPIO_InitStructD;//定义gpioD端口设置结构体
	
	/*开启GPIO口时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);
	
	/*关闭JTAG-DP ， 启用SWDP*/
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	/***************配置SEL模块所有引脚为推挽输出************/
	
	/*配置PD4-6为推挽输出*/
	GPIO_InitStructD.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructD.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructD.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructD);
	/*锁存指定引脚*/
	GPIO_PinLockConfig(GPIOD,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6);
	/*配置PB3 4 5 8 9 12 13 14 15*/
	GPIO_InitStructB.GPIO_Pin=GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructB.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructB.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructB);
	/*锁存指定引脚*/
	GPIO_PinLockConfig(GPIOB,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
	
	GPIO_InitStructC.GPIO_Pin=GPIO_Pin_12;
	GPIO_InitStructC.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructC.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructC);
	/*锁存指定引脚*/
	GPIO_PinLockConfig(GPIOC,GPIO_Pin_12);
	
	GPIO_InitStructD.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructD.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructD.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructD);
	/*锁存指定引脚*/
	GPIO_PinLockConfig(GPIOD,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2);
	
	/***************配置OUT模块所有引脚为推挽输出************/
	/*给端口赋确定值*/
	/*OUTPUT模块*/
	GPIO_WriteBit(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12,Bit_RESET);
	GPIO_WriteBit(GPIOC,GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9,Bit_RESET);
	
	/*配置GPIOC7-9引脚*/
	GPIO_InitStructC.GPIO_Pin=GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;//选择引脚
	GPIO_InitStructC.GPIO_Mode=GPIO_Mode_Out_PP;//设置引脚模式为推挽输出
	GPIO_InitStructC.GPIO_Speed=GPIO_Speed_50MHz;//设置频率
	GPIO_Init(GPIOC, &GPIO_InitStructC); //初始化GPIOC端口
	/*锁存指定引脚*/
	GPIO_PinLockConfig(GPIOC,GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);
	
	/*配置GPIOA8-12引脚*/
	GPIO_InitStructA.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;//选择引脚
	GPIO_InitStructA.GPIO_Mode=GPIO_Mode_Out_PP;//设置引脚模式为推挽输出
	GPIO_InitStructA.GPIO_Speed=GPIO_Speed_50MHz;//设置频率
	GPIO_Init(GPIOA, &GPIO_InitStructA); //初始化GPIOA端口
	/*锁存指定引脚*/
	GPIO_PinLockConfig(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12);
	
	/*给端口赋确定值*/
	/*OUTPUT模块*/
	GPIO_WriteBit(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12,Bit_RESET);
	GPIO_WriteBit(GPIOC,GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9,Bit_RESET);
	
	/*SEL模块*/
	GPIO_WriteBit(GPIOD,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2,Bit_RESET);
	GPIO_WriteBit(GPIOB,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15,Bit_RESET);
	GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_RESET);
	
		
}

/*********************************************************************************************
* name			:		USART1_Configuration
* func			:		初始化串口
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void USART1_Configuration(void)
{
	
	/*********************设定USART1相应参数******************/
	USART_InitTypeDef USART1_InitStruct;//设置定义结构体
	USART_ClockInitTypeDef USART_ClockInitStruct;//设置串口与时间有关结构体
	GPIO_InitTypeDef GPIO_InitStructB;//定义gpioB端口设置结构体
	NVIC_InitTypeDef NVIC_InitStruct;//中断配置结构体
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	/*************配置USART1的输入和输出重映射到相应引脚*********/
	
	/*启用USART1软件映射-----TX->PB6,RX->PB7 */
	GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);
	
	/*配置USART1_TX的复用输出功能*/
	GPIO_InitStructB.GPIO_Pin=GPIO_Pin_6;//引脚为GPIOB6
	GPIO_InitStructB.GPIO_Mode=GPIO_Mode_AF_PP;//设置为复用输出
	GPIO_InitStructB.GPIO_Speed=GPIO_Speed_50MHz;//设置频率，最大波特率为115200
	GPIO_Init(GPIOB,&GPIO_InitStructB);//初始化端口
	/*锁存指定引脚*/
	GPIO_PinLockConfig(GPIOB,GPIO_Pin_6);
	
	/*配置USART1_RX的浮空输入功能*/
	GPIO_InitStructB.GPIO_Pin=GPIO_Pin_7;//引脚为GPIOB7
	GPIO_InitStructB.GPIO_Mode=GPIO_Mode_IN_FLOATING;//设置为浮空输入
	GPIO_InitStructB.GPIO_Speed=GPIO_Speed_2MHz;//设置频率
	GPIO_Init(GPIOB,&GPIO_InitStructB);//初始化端口
	/*锁存指定引脚*/
	GPIO_PinLockConfig(GPIOB,GPIO_Pin_7);
	
	/*串口中断配置，用于终端接收数据*/
	NVIC_InitStruct.NVIC_IRQChannel=USART1_IRQn;//配置中断向量通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	
	/*按照配置结构体初始化中断向量*/
	NVIC_Init(&NVIC_InitStruct);
	
	/*配置串口数据*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//开启串口时钟
	USART1_InitStruct.USART_BaudRate=115200;//设定波特率
	USART1_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件标记位
	USART1_InitStruct.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//收发都使能
	USART1_InitStruct.USART_Parity=USART_Parity_No;//设置为无校验位；
	USART1_InitStruct.USART_StopBits=USART_StopBits_1;//停止位为1
	USART1_InitStruct.USART_WordLength=USART_WordLength_8b;//字长为八位
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;//
  USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;//
  USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;//
  USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable;//
	USART_Init(USART1,&USART1_InitStruct);//根据设置初始化串口
	USART_ClockInit(USART1,&USART_ClockInitStruct);
	
	/*开启USART1端口*/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断接收
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除接收中断位
	USART_Cmd(USART1,ENABLE);
	
}

/*********************************************************************************************
* name			:		EXTI_Configuration
* func			:		初始化中断通道
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void EXTI_Configuration(void)
{

	/*定义中断引脚配置结构体*/
	EXTI_InitTypeDef EXTI_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
	
	/*************设置sin模块所有引脚为浮空输入*****************/
	
	/*端口E相关引脚设置*/
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//选择端口E0-7号引脚
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING; //设置模式为浮空输入
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;//设置速度为50MHz （输入可以不用配置速度）
	GPIO_Init(GPIOE,&GPIO_InitStruct);//按照结构体配置端口E;
	
	/*端口D相关引脚配置*/
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;//选择端口D8-15号引脚
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;//设置模式为浮空输入
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;//设置速度为50MHz  （输入可以不用配置速度）
	GPIO_Init(GPIOD,&GPIO_InitStruct);//按照结构体配置端口D;
	
		/*将GPIOE0-7引脚配置为中断引脚*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource0);
		/*配置中断引脚相应参数*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line0;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//设定中断模式为中断处理模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//设定中断触发模式为上升沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStruct);//按照结构体初始化中断

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource1);
		/*配置中断引脚相应参数*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line1;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//设定中断模式为中断处理模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//设定中断触发模式为上升沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStruct);//按照结构体初始化中断

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource2);
		/*配置中断引脚相应参数*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line2;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//设定中断模式为中断处理模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//设定中断触发模式为上升沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStruct);//按照结构体初始化中断

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource3);
		/*配置中断引脚相应参数*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line3;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//设定中断模式为中断处理模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//设定中断触发模式为上升沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStruct);//按照结构体初始化中断

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource4);
		/*配置中断引脚相应参数*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line4;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//设定中断模式为中断处理模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//设定中断触发模式为上升沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStruct);//按照结构体初始化中断

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource5);
		/*配置中断引脚相应参数*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line5;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//设定中断模式为中断处理模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//设定中断触发模式为上升沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStruct);//按照结构体初始化中断

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource6);
		/*配置中断引脚相应参数*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line6;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//设定中断模式为中断处理模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//设定中断触发模式为上升沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStruct);//按照结构体初始化中断

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource7);
		/*配置中断引脚相应参数*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line7;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//设定中断模式为中断处理模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//设定中断触发模式为上升沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStruct);//按照结构体初始化中断

	/*将GPIOD8-15引脚配置为中断引脚*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource8);
		/*配置中断引脚相应参数*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line8;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//设定中断模式为中断处理模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//设定中断触发模式为上升沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStruct);//按照结构体初始化中断

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource9);
		/*配置中断引脚相应参数*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line9;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//设定中断模式为中断处理模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//设定中断触发模式为上升沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStruct);//按照结构体初始化中断

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource10);
		/*配置中断引脚相应参数*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line10;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//设定中断模式为中断处理模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//设定中断触发模式为上升沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStruct);//按照结构体初始化中断

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource11);
		/*配置中断引脚相应参数*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line11;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//设定中断模式为中断处理模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//设定中断触发模式为上升沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStruct);//按照结构体初始化中断

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource12);
		/*配置中断引脚相应参数*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line12;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//设定中断模式为中断处理模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//设定中断触发模式为上升沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStruct);//按照结构体初始化中断

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource13);
		/*配置中断引脚相应参数*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line13;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//设定中断模式为中断处理模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//设定中断触发模式为上升沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStruct);//按照结构体初始化中断

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource14);
		/*配置中断引脚相应参数*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line14;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//设定中断模式为中断处理模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//设定中断触发模式为上升沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStruct);//按照结构体初始化中断

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource15);
		/*配置中断引脚相应参数*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line15;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//设定中断模式为中断处理模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//设定中断触发模式为上升沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStruct);//按照结构体初始化中断
	
	/*锁存指定引脚*/
	GPIO_PinLockConfig(GPIOD,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
	GPIO_PinLockConfig(GPIOE,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);

	/*清除中断线路标记位*/
	//EXTI_ClearITPendingBit(EXTI_Line0|EXTI_Line1|EXTI_Line2|EXTI_Line3|EXTI_Line4|EXTI_Line5|EXTI_Line6|EXTI_Line7|EXTI_Line8|EXTI_Line9|EXTI_Line10|EXTI_Line11|EXTI_Line12|EXTI_Line13|EXTI_Line14|EXTI_Line15);

}


/*********************************************************************************************
* name			:		NVIC_Configuration
* func			:		初始化中断向量
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void NVIC_Configuration()
{
	
	/*定义中断向量配置结构体*/
	NVIC_InitTypeDef NVIC_InitStruct;
	
	/*指定中断优先级分组为第一组，一位表示抢占（主）优先级，三位表示响应（从）优先级*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	/*配置中断向量结构体相应参数*/
	NVIC_InitStruct.NVIC_IRQChannel=EXTI0_IRQn;//配置中断向量通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI1_IRQn;//配置中断向量通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI2_IRQn;//配置中断向量通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI3_IRQn;//配置中断向量通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI4_IRQn;//配置中断向量通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI9_5_IRQn;//配置中断向量通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI15_10_IRQn;//配置中断向量通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);

}

/*********************************************************************************************
* name			:		TIM_Configuration
* func			:		初始化通用定时器
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void TIM_Configuration(void)
{
	/*变量定义区*/
	NVIC_InitTypeDef NVIC_InitStruct;//中断配置结构体
	TIM_TimeBaseInitTypeDef TIM_BaseInitStruct;//定时器设置结构体
	
	/*启用TIM2时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	/*配置中断相应参数*/
	NVIC_InitStruct.NVIC_IRQChannel=TIM2_IRQn;//设置中断通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级0
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;//响应优先级0
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;//允许中断
	NVIC_Init(&NVIC_InitStruct);//初始化中断
	
	/*TIM2定时器相关配置*/
	TIM_BaseInitStruct.TIM_Period = 1000; //100ms定时时间 
  TIM_BaseInitStruct.TIM_Prescaler = 7199;//频率1kHz    
  TIM_BaseInitStruct.TIM_ClockDivision = 0; //时钟分割
  TIM_BaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;  //计数方向向上计数
  TIM_TimeBaseInit(TIM2, &TIM_BaseInitStruct);//根据结构体初始化tim2
	
	/*清空中断，以免开始立即中断*/
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	
	/*使能TIM2中断源*/
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	/*开启TIM2中断*/
	TIM_Cmd(TIM2,ENABLE);
}


/*********************************************************************************************
* name			:		ADC1_Configuration
* func			:		初始化ADC1
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void ADC1_Configuration(void)
{
	/*********定义初始化结构体**************/
	ADC_InitTypeDef  ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/**********使能ADC和GPIO时钟************/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//设定ADC时钟为72/6=12M,因为ADC时钟最大不能超过14M;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	/********初始化GPIOA0-7和GPIOB0-1和GPIOC0-5****/
	GPIO_InitStructure.GPIO_Pin  =GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AIN;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	/*锁存指定引脚*/
	GPIO_PinLockConfig(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	
	GPIO_InitStructure.GPIO_Pin  =GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AIN;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	/*锁存指定引脚*/
	GPIO_PinLockConfig(GPIOB,GPIO_Pin_0|GPIO_Pin_1);
	
	GPIO_InitStructure.GPIO_Pin  =GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AIN;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	/*锁存指定引脚*/
	GPIO_PinLockConfig(GPIOC,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
	
	
	/****************配置ADC相应参数****************/
	/*********以下参数需要根据实际应用进行修改******/
	ADC_DeInit(ADC1);//重设ADC1
	ADC_InitStructure.ADC_Mode              =ADC_Mode_Independent;  //独立模式
	ADC_InitStructure.ADC_ScanConvMode      =DISABLE;      //数模转换工作在扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;      //连续转换
	ADC_InitStructure.ADC_ExternalTrigConv  =ADC_ExternalTrigConv_None; //外部触发关闭
	ADC_InitStructure.ADC_DataAlign         =ADC_DataAlign_Right;   //右对齐
	ADC_InitStructure.ADC_NbrOfChannel      =1;       //扫描通道数
	ADC_Init(ADC1,&ADC_InitStructure);//根据设置初始化ADC
	
	ADC_Cmd  (ADC1,ENABLE);//使能ADC1
	
	ADC_ResetCalibration(ADC1);//复位ADC1
	
	while(ADC_GetResetCalibrationStatus(ADC1));//等待ADC复位完成
	
	/* ADC校准*/
  ADC_StartCalibration(ADC1);
	
  /* 等待ADC校准完成 */
  while(ADC_GetCalibrationStatus(ADC1));
	
}

/*********************************************************************************************
* name			:		RCC_Configuration
* func			:		配置系统时钟
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void RCC_Configuration(void)
{
ErrorStatus HSEStartUpStatus;
	/*复位系统时钟*/
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);//打开外部时钟
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if( HSEStartUpStatus==SUCCESS)//如果HSE启动成功
	{
		/* Enable Prefetch Buffer */
		FLASH->ACR |= FLASH_ACR_PRFTBE;

		/* Flash 2 wait state */
		FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
		FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;
		
		/*配置系统以及外设时钟*/
		RCC_HCLKConfig(RCC_SYSCLK_Div1);//配置HCLK=SYSCLK
		RCC_PCLK2Config(RCC_HCLK_Div1);//配置APB2=HCLK
		RCC_PCLK1Config(RCC_HCLK_Div2);//配置APB1=HCLK/2=36M,APB1最高支持36M
		
		/************设置系统时钟相关系数*************/
		/*设定不同时钟频率可在此修改*/
#ifdef SYSCLK_FREQ_24MHz
			RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_2);//PLLCLK=HSE*6=72M
#elif SYSCLK_FREQ_36MHz
			RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_3);//PLLCLK=HSE*6=72M
#elif SYSCLK_FREQ_48MHz
			RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_4);//PLLCLK=HSE*6=72M
#elif SYSCLK_FREQ_60MHz
			RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_5);//PLLCLK=HSE*6=72M
#elif SYSCLK_FREQ_72MHz
			RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_6);//PLLCLK=HSE*6=72M
#endif
		RCC_PLLCmd(ENABLE);//使能PLL
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET);//等待PLL设定完成
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//设定倍频后的时钟作为系统时钟
		while(RCC_GetSYSCLKSource()!=0x08);//等待将PLL做为系统时钟
	}
	else/*HSE启动失败*/
	{
	}
}
   
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
