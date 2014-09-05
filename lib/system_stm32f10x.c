/*********************************************************************************************
* File name	: system_stm32f10x.c
* HardWare	: STM32F103VBT 
* Author		: Lius
* History		:	none
*	Data			:	2014-06-07
* Descript	: STM32F103VBT �ɼ�������ʼ��оƬ�������
*********************************************************************************************/

#include "stm32f10x.h"
#include "main.h"

/*****************����ϵͳʱ��Ƶ��**********************/   
/* #define SYSCLK_FREQ_24MHz  24000000 */
/* #define SYSCLK_FREQ_36MHz  36000000 */
/* #define SYSCLK_FREQ_48MHz  48000000 */
/* #define SYSCLK_FREQ_60MHz  60000000 */
	 #define SYSCLK_FREQ_72MHz  72000000 

/****************ADC1���ݼĴ�������ַ*******************/
#define ADC1_DR_Address     0x4001244C

/******************extern data**************************/
extern uint16_t adcValue[10][16];//ÿ��ͨ���ɼ�10�Σ�һ��16��ͨ��

/**************************��������***********************/
void RCC_Configuration(void);//����ϵͳʱ��
void GPIO_Configuration(void);//�趨gpio����
void USART1_Configuration(void);//��ʼ��USART
void EXTI_Configuration(void);//��ʼ���ж�ͨ��
void NVIC_Configuration(void);//��ʼ���ж�����
void TIM_Configuration(void);//��ʱ����ʼ������
void ADC1_Configuration(void);//��ʼ��ADC1��
//void DMA_Configuration(void);//��ʼ��DMA

/*********************************************************************************************
* name			:		SystemInit
* func			:		ϵͳĬ�ϳ�ʼ������
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void SystemInit (void)
{
  /* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */
  /* Configure the Flash Latency cycles and enable prefetch buffer */
	RCC_Configuration();//��ʼ��ϵͳʱ��
	GPIO_Configuration();
}

/*********************************************************************************************
* name			:		GPIO_Configuration
* func			:		��ʼ���õ���GPIO�˿�
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void GPIO_Configuration(void)
{
	/*********����˿����ýṹ�壬������������*******************/	
	GPIO_InitTypeDef GPIO_InitStructA;//����gpioA�˿����ýṹ��
	GPIO_InitTypeDef GPIO_InitStructB;//����gpioB�˿����ýṹ��
	GPIO_InitTypeDef GPIO_InitStructC;//����gpioC�˿����ýṹ��
	GPIO_InitTypeDef GPIO_InitStructD;//����gpioD�˿����ýṹ��
	
	/*����GPIO��ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);
	
	/*�ر�JTAG-DP �� ����SWDP*/
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	/***************����SELģ����������Ϊ�������************/
	
	/*����PD4-6Ϊ�������*/
	GPIO_InitStructD.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructD.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructD.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructD);
	/*����ָ������*/
	GPIO_PinLockConfig(GPIOD,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6);
	/*����PB3 4 5 8 9 12 13 14 15*/
	GPIO_InitStructB.GPIO_Pin=GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructB.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructB.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructB);
	/*����ָ������*/
	GPIO_PinLockConfig(GPIOB,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
	
	GPIO_InitStructC.GPIO_Pin=GPIO_Pin_12;
	GPIO_InitStructC.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructC.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructC);
	/*����ָ������*/
	GPIO_PinLockConfig(GPIOC,GPIO_Pin_12);
	
	GPIO_InitStructD.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructD.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructD.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructD);
	/*����ָ������*/
	GPIO_PinLockConfig(GPIOD,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2);
	
	/***************����OUTģ����������Ϊ�������************/
	/*���˿ڸ�ȷ��ֵ*/
	/*OUTPUTģ��*/
	GPIO_WriteBit(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12,Bit_RESET);
	GPIO_WriteBit(GPIOC,GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9,Bit_RESET);
	
	/*����GPIOC7-9����*/
	GPIO_InitStructC.GPIO_Pin=GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;//ѡ������
	GPIO_InitStructC.GPIO_Mode=GPIO_Mode_Out_PP;//��������ģʽΪ�������
	GPIO_InitStructC.GPIO_Speed=GPIO_Speed_50MHz;//����Ƶ��
	GPIO_Init(GPIOC, &GPIO_InitStructC); //��ʼ��GPIOC�˿�
	/*����ָ������*/
	GPIO_PinLockConfig(GPIOC,GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);
	
	/*����GPIOA8-12����*/
	GPIO_InitStructA.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;//ѡ������
	GPIO_InitStructA.GPIO_Mode=GPIO_Mode_Out_PP;//��������ģʽΪ�������
	GPIO_InitStructA.GPIO_Speed=GPIO_Speed_50MHz;//����Ƶ��
	GPIO_Init(GPIOA, &GPIO_InitStructA); //��ʼ��GPIOA�˿�
	/*����ָ������*/
	GPIO_PinLockConfig(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12);
	
	/*���˿ڸ�ȷ��ֵ*/
	/*OUTPUTģ��*/
	GPIO_WriteBit(GPIOA,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12,Bit_RESET);
	GPIO_WriteBit(GPIOC,GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9,Bit_RESET);
	
	/*SELģ��*/
	GPIO_WriteBit(GPIOD,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2,Bit_RESET);
	GPIO_WriteBit(GPIOB,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15,Bit_RESET);
	GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_RESET);
	
		
}

/*********************************************************************************************
* name			:		USART1_Configuration
* func			:		��ʼ������
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void USART1_Configuration(void)
{
	
	/*********************�趨USART1��Ӧ����******************/
	USART_InitTypeDef USART1_InitStruct;//���ö���ṹ��
	USART_ClockInitTypeDef USART_ClockInitStruct;//���ô�����ʱ���йؽṹ��
	GPIO_InitTypeDef GPIO_InitStructB;//����gpioB�˿����ýṹ��
	NVIC_InitTypeDef NVIC_InitStruct;//�ж����ýṹ��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	/*************����USART1������������ӳ�䵽��Ӧ����*********/
	
	/*����USART1���ӳ��-----TX->PB6,RX->PB7 */
	GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);
	
	/*����USART1_TX�ĸ����������*/
	GPIO_InitStructB.GPIO_Pin=GPIO_Pin_6;//����ΪGPIOB6
	GPIO_InitStructB.GPIO_Mode=GPIO_Mode_AF_PP;//����Ϊ�������
	GPIO_InitStructB.GPIO_Speed=GPIO_Speed_50MHz;//����Ƶ�ʣ��������Ϊ115200
	GPIO_Init(GPIOB,&GPIO_InitStructB);//��ʼ���˿�
	/*����ָ������*/
	GPIO_PinLockConfig(GPIOB,GPIO_Pin_6);
	
	/*����USART1_RX�ĸ������빦��*/
	GPIO_InitStructB.GPIO_Pin=GPIO_Pin_7;//����ΪGPIOB7
	GPIO_InitStructB.GPIO_Mode=GPIO_Mode_IN_FLOATING;//����Ϊ��������
	GPIO_InitStructB.GPIO_Speed=GPIO_Speed_2MHz;//����Ƶ��
	GPIO_Init(GPIOB,&GPIO_InitStructB);//��ʼ���˿�
	/*����ָ������*/
	GPIO_PinLockConfig(GPIOB,GPIO_Pin_7);
	
	/*�����ж����ã������ն˽�������*/
	NVIC_InitStruct.NVIC_IRQChannel=USART1_IRQn;//�����ж�����ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	
	/*�������ýṹ���ʼ���ж�����*/
	NVIC_Init(&NVIC_InitStruct);
	
	/*���ô�������*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//��������ʱ��
	USART1_InitStruct.USART_BaudRate=115200;//�趨������
	USART1_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��Ӳ�����λ
	USART1_InitStruct.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//�շ���ʹ��
	USART1_InitStruct.USART_Parity=USART_Parity_No;//����Ϊ��У��λ��
	USART1_InitStruct.USART_StopBits=USART_StopBits_1;//ֹͣλΪ1
	USART1_InitStruct.USART_WordLength=USART_WordLength_8b;//�ֳ�Ϊ��λ
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;//
  USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;//
  USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;//
  USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable;//
	USART_Init(USART1,&USART1_InitStruct);//�������ó�ʼ������
	USART_ClockInit(USART1,&USART_ClockInitStruct);
	
	/*����USART1�˿�*/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����жϽ���
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);//��������ж�λ
	USART_Cmd(USART1,ENABLE);
	
}

/*********************************************************************************************
* name			:		EXTI_Configuration
* func			:		��ʼ���ж�ͨ��
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void EXTI_Configuration(void)
{

	/*�����ж��������ýṹ��*/
	EXTI_InitTypeDef EXTI_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
	
	/*************����sinģ����������Ϊ��������*****************/
	
	/*�˿�E�����������*/
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//ѡ��˿�E0-7������
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING; //����ģʽΪ��������
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;//�����ٶ�Ϊ50MHz ��������Բ��������ٶȣ�
	GPIO_Init(GPIOE,&GPIO_InitStruct);//���սṹ�����ö˿�E;
	
	/*�˿�D�����������*/
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;//ѡ��˿�D8-15������
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;//����ģʽΪ��������
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;//�����ٶ�Ϊ50MHz  ��������Բ��������ٶȣ�
	GPIO_Init(GPIOD,&GPIO_InitStruct);//���սṹ�����ö˿�D;
	
		/*��GPIOE0-7��������Ϊ�ж�����*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource0);
		/*�����ж�������Ӧ����*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line0;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//�趨�ж�ģʽΪ�жϴ���ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//�趨�жϴ���ģʽΪ�����ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//ʹ���ж�
	EXTI_Init(&EXTI_InitStruct);//���սṹ���ʼ���ж�

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource1);
		/*�����ж�������Ӧ����*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line1;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//�趨�ж�ģʽΪ�жϴ���ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//�趨�жϴ���ģʽΪ�����ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//ʹ���ж�
	EXTI_Init(&EXTI_InitStruct);//���սṹ���ʼ���ж�

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource2);
		/*�����ж�������Ӧ����*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line2;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//�趨�ж�ģʽΪ�жϴ���ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//�趨�жϴ���ģʽΪ�����ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//ʹ���ж�
	EXTI_Init(&EXTI_InitStruct);//���սṹ���ʼ���ж�

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource3);
		/*�����ж�������Ӧ����*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line3;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//�趨�ж�ģʽΪ�жϴ���ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//�趨�жϴ���ģʽΪ�����ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//ʹ���ж�
	EXTI_Init(&EXTI_InitStruct);//���սṹ���ʼ���ж�

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource4);
		/*�����ж�������Ӧ����*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line4;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//�趨�ж�ģʽΪ�жϴ���ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//�趨�жϴ���ģʽΪ�����ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//ʹ���ж�
	EXTI_Init(&EXTI_InitStruct);//���սṹ���ʼ���ж�

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource5);
		/*�����ж�������Ӧ����*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line5;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//�趨�ж�ģʽΪ�жϴ���ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//�趨�жϴ���ģʽΪ�����ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//ʹ���ж�
	EXTI_Init(&EXTI_InitStruct);//���սṹ���ʼ���ж�

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource6);
		/*�����ж�������Ӧ����*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line6;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//�趨�ж�ģʽΪ�жϴ���ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//�趨�жϴ���ģʽΪ�����ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//ʹ���ж�
	EXTI_Init(&EXTI_InitStruct);//���սṹ���ʼ���ж�

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource7);
		/*�����ж�������Ӧ����*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line7;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//�趨�ж�ģʽΪ�жϴ���ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//�趨�жϴ���ģʽΪ�����ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//ʹ���ж�
	EXTI_Init(&EXTI_InitStruct);//���սṹ���ʼ���ж�

	/*��GPIOD8-15��������Ϊ�ж�����*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource8);
		/*�����ж�������Ӧ����*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line8;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//�趨�ж�ģʽΪ�жϴ���ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//�趨�жϴ���ģʽΪ�����ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//ʹ���ж�
	EXTI_Init(&EXTI_InitStruct);//���սṹ���ʼ���ж�

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource9);
		/*�����ж�������Ӧ����*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line9;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//�趨�ж�ģʽΪ�жϴ���ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//�趨�жϴ���ģʽΪ�����ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//ʹ���ж�
	EXTI_Init(&EXTI_InitStruct);//���սṹ���ʼ���ж�

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource10);
		/*�����ж�������Ӧ����*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line10;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//�趨�ж�ģʽΪ�жϴ���ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//�趨�жϴ���ģʽΪ�����ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//ʹ���ж�
	EXTI_Init(&EXTI_InitStruct);//���սṹ���ʼ���ж�

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource11);
		/*�����ж�������Ӧ����*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line11;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//�趨�ж�ģʽΪ�жϴ���ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//�趨�жϴ���ģʽΪ�����ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//ʹ���ж�
	EXTI_Init(&EXTI_InitStruct);//���սṹ���ʼ���ж�

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource12);
		/*�����ж�������Ӧ����*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line12;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//�趨�ж�ģʽΪ�жϴ���ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//�趨�жϴ���ģʽΪ�����ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//ʹ���ж�
	EXTI_Init(&EXTI_InitStruct);//���սṹ���ʼ���ж�

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource13);
		/*�����ж�������Ӧ����*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line13;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//�趨�ж�ģʽΪ�жϴ���ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//�趨�жϴ���ģʽΪ�����ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//ʹ���ж�
	EXTI_Init(&EXTI_InitStruct);//���սṹ���ʼ���ж�

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource14);
		/*�����ж�������Ӧ����*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line14;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//�趨�ж�ģʽΪ�жϴ���ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//�趨�жϴ���ģʽΪ�����ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//ʹ���ж�
	EXTI_Init(&EXTI_InitStruct);//���սṹ���ʼ���ж�

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource15);
		/*�����ж�������Ӧ����*/
	EXTI_InitStruct.EXTI_Line=EXTI_Line15;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;//�趨�ж�ģʽΪ�жϴ���ģʽ
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;//�趨�жϴ���ģʽΪ�����ش���
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;//ʹ���ж�
	EXTI_Init(&EXTI_InitStruct);//���սṹ���ʼ���ж�
	
	/*����ָ������*/
	GPIO_PinLockConfig(GPIOD,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
	GPIO_PinLockConfig(GPIOE,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);

	/*����ж���·���λ*/
	//EXTI_ClearITPendingBit(EXTI_Line0|EXTI_Line1|EXTI_Line2|EXTI_Line3|EXTI_Line4|EXTI_Line5|EXTI_Line6|EXTI_Line7|EXTI_Line8|EXTI_Line9|EXTI_Line10|EXTI_Line11|EXTI_Line12|EXTI_Line13|EXTI_Line14|EXTI_Line15);

}


/*********************************************************************************************
* name			:		NVIC_Configuration
* func			:		��ʼ���ж�����
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void NVIC_Configuration()
{
	
	/*�����ж��������ýṹ��*/
	NVIC_InitTypeDef NVIC_InitStruct;
	
	/*ָ���ж����ȼ�����Ϊ��һ�飬һλ��ʾ��ռ���������ȼ�����λ��ʾ��Ӧ���ӣ����ȼ�*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	/*�����ж������ṹ����Ӧ����*/
	NVIC_InitStruct.NVIC_IRQChannel=EXTI0_IRQn;//�����ж�����ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI1_IRQn;//�����ж�����ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI2_IRQn;//�����ж�����ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI3_IRQn;//�����ж�����ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI4_IRQn;//�����ж�����ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI9_5_IRQn;//�����ж�����ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI15_10_IRQn;//�����ж�����ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);

}

/*********************************************************************************************
* name			:		TIM_Configuration
* func			:		��ʼ��ͨ�ö�ʱ��
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void TIM_Configuration(void)
{
	/*����������*/
	NVIC_InitTypeDef NVIC_InitStruct;//�ж����ýṹ��
	TIM_TimeBaseInitTypeDef TIM_BaseInitStruct;//��ʱ�����ýṹ��
	
	/*����TIM2ʱ��*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	/*�����ж���Ӧ����*/
	NVIC_InitStruct.NVIC_IRQChannel=TIM2_IRQn;//�����ж�ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�0
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;//��Ӧ���ȼ�0
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;//�����ж�
	NVIC_Init(&NVIC_InitStruct);//��ʼ���ж�
	
	/*TIM2��ʱ���������*/
	TIM_BaseInitStruct.TIM_Period = 1000; //100ms��ʱʱ�� 
  TIM_BaseInitStruct.TIM_Prescaler = 7199;//Ƶ��1kHz    
  TIM_BaseInitStruct.TIM_ClockDivision = 0; //ʱ�ӷָ�
  TIM_BaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;  //�����������ϼ���
  TIM_TimeBaseInit(TIM2, &TIM_BaseInitStruct);//���ݽṹ���ʼ��tim2
	
	/*����жϣ����⿪ʼ�����ж�*/
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	
	/*ʹ��TIM2�ж�Դ*/
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	/*����TIM2�ж�*/
	TIM_Cmd(TIM2,ENABLE);
}


/*********************************************************************************************
* name			:		ADC1_Configuration
* func			:		��ʼ��ADC1
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void ADC1_Configuration(void)
{
	/*********�����ʼ���ṹ��**************/
	ADC_InitTypeDef  ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/**********ʹ��ADC��GPIOʱ��************/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//�趨ADCʱ��Ϊ72/6=12M,��ΪADCʱ������ܳ���14M;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	/********��ʼ��GPIOA0-7��GPIOB0-1��GPIOC0-5****/
	GPIO_InitStructure.GPIO_Pin  =GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AIN;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	/*����ָ������*/
	GPIO_PinLockConfig(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	
	GPIO_InitStructure.GPIO_Pin  =GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AIN;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	/*����ָ������*/
	GPIO_PinLockConfig(GPIOB,GPIO_Pin_0|GPIO_Pin_1);
	
	GPIO_InitStructure.GPIO_Pin  =GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AIN;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	/*����ָ������*/
	GPIO_PinLockConfig(GPIOC,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
	
	
	/****************����ADC��Ӧ����****************/
	/*********���²�����Ҫ����ʵ��Ӧ�ý����޸�******/
	ADC_DeInit(ADC1);//����ADC1
	ADC_InitStructure.ADC_Mode              =ADC_Mode_Independent;  //����ģʽ
	ADC_InitStructure.ADC_ScanConvMode      =DISABLE;      //��ģת��������ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;      //����ת��
	ADC_InitStructure.ADC_ExternalTrigConv  =ADC_ExternalTrigConv_None; //�ⲿ�����ر�
	ADC_InitStructure.ADC_DataAlign         =ADC_DataAlign_Right;   //�Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel      =1;       //ɨ��ͨ����
	ADC_Init(ADC1,&ADC_InitStructure);//�������ó�ʼ��ADC
	
	ADC_Cmd  (ADC1,ENABLE);//ʹ��ADC1
	
	ADC_ResetCalibration(ADC1);//��λADC1
	
	while(ADC_GetResetCalibrationStatus(ADC1));//�ȴ�ADC��λ���
	
	/* ADCУ׼*/
  ADC_StartCalibration(ADC1);
	
  /* �ȴ�ADCУ׼��� */
  while(ADC_GetCalibrationStatus(ADC1));
	
}

/*********************************************************************************************
* name			:		RCC_Configuration
* func			:		����ϵͳʱ��
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void RCC_Configuration(void)
{
ErrorStatus HSEStartUpStatus;
	/*��λϵͳʱ��*/
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);//���ⲿʱ��
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if( HSEStartUpStatus==SUCCESS)//���HSE�����ɹ�
	{
		/* Enable Prefetch Buffer */
		FLASH->ACR |= FLASH_ACR_PRFTBE;

		/* Flash 2 wait state */
		FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
		FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;
		
		/*����ϵͳ�Լ�����ʱ��*/
		RCC_HCLKConfig(RCC_SYSCLK_Div1);//����HCLK=SYSCLK
		RCC_PCLK2Config(RCC_HCLK_Div1);//����APB2=HCLK
		RCC_PCLK1Config(RCC_HCLK_Div2);//����APB1=HCLK/2=36M,APB1���֧��36M
		
		/************����ϵͳʱ�����ϵ��*************/
		/*�趨��ͬʱ��Ƶ�ʿ��ڴ��޸�*/
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
		RCC_PLLCmd(ENABLE);//ʹ��PLL
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET);//�ȴ�PLL�趨���
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//�趨��Ƶ���ʱ����Ϊϵͳʱ��
		while(RCC_GetSYSCLKSource()!=0x08);//�ȴ���PLL��Ϊϵͳʱ��
	}
	else/*HSE����ʧ��*/
	{
	}
}
   
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
