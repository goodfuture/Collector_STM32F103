
/*********************************************************************************************
* File name	: main.c
* HardWare	: STM32F103VBT 
* Author		: Lius
* History		:	none
*	Data			:	2014-06-07
* Descript	: STM32F103VBT ²É¼¯Æ÷£¬Ö÷º¯Êý²¿·Ö£¬Ö÷³ÌÐòÈë¿Ú
*********************************************************************************************/

/**************************Í·ÎÄ¼þ°üº¬Çø******************************************************/
#include "main.h"
#include "stm32f10x_it.h"

/**************************ºê²ÎÊý¶¨ÒåÇø******************************************************/
#define CLI()      __set_PRIMASK(1)//¹Ø±ÕÖÐ¶Ï  
#define SEI()      __set_PRIMASK(0)//¿ªÆôÖÐ¶Ï

#define SENDBYINT 1												/*	ÊÇ·ñÀûÓÃÖÐ¶Ï·¢ËÍ								*/

#define GETADTIMES 1											/*	Ã¿´Î»ñÈ¡ADÖµÈ¡Öµ´ÎÊý						*/

#define SELPORT_0_PD4 (0)									/*	¶¨ÒåSELÄ£¿é¶Ë¿Ú¶ÔÓ¦µÄSELÊä³öÏß	*/
#define SELPORT_1_PD5 (1)									/*	PD5	-	>	SEL1									*/
#define SELPORT_2_PD6 (2)									/*	PD6	-	>	SEL2									*/
#define SELPORT_3_PB3 (3)
#define SELPORT_4_PB4 (7)
#define SELPORT_5_PB5 (6)
#define SELPORT_6_PB8 (5)
#define SELPORT_7_PB9 (4)
#define SELPORT_8_PC12 (11)
#define SELPORT_9_PD0 (10)
#define SELPORT_10_PD1 (9)
#define SELPORT_11_PD2 (8)
#define SELPORT_12_PB12 (15)
#define SELPORT_13_PB13 (14)
#define SELPORT_14_PB14 (13)
#define SELPORT_15_PB15 (12)

#define OUTPUT_0_PC7	(7)									/*	¶¨ÒåOUTPUTÄ£¿é¶Ë¿Ú¶ÔÓ¦µÄÊä³öÏß	*/
#define OUTPUT_1_PC8	(6)									/*	PC8	-	>	OUTPUT6								*/
#define OUTPUT_2_PC9	(5)									/*	PC9	-	>	OUTPUT5								*/
#define OUTPUT_3_PA8	(4)
#define OUTPUT_4_PA9	(3)
#define OUTPUT_5_PA10	(2)
#define OUTPUT_6_PA11	(1)
#define OUTPUT_7_PA12	(0)

#define ADINPORT_0_PA0 (7)									/*	¶¨ÒåADINÄ£¿é¶Ë¿Ú¶ÔÓ¦µÄADINÊäÈëÏß	*/
#define ADINPORT_1_PA1 (6)									/*	PA1	-	>	ADIN1									*/
#define ADINPORT_2_PA2 (5)									/*	PA2	-	>	ADIN2									*/
#define ADINPORT_3_PA3 (0)
#define ADINPORT_4_PA4 (15)
#define ADINPORT_5_PA5 (14)
#define ADINPORT_6_PA6 (13)
#define ADINPORT_7_PA7 (10)
#define ADINPORT_8_PB0 (11)
#define ADINPORT_9_PB1 (8)
#define ADINPORT_10_PC0 (4)
#define ADINPORT_11_PC1 (1)
#define ADINPORT_12_PC2 (2)
#define ADINPORT_13_PC3 (3)
#define ADINPORT_14_PC4 (12)
#define ADINPORT_15_PC5 (9)


/**************************º¯ÊýÉùÃ÷Çø*********************************************************/
void GetAverage(void);
uint16_t GetVolt(uint16_t value);
uint16_t Get_Adc(uint8_t ch);
uint16_t Get_Adc_Average(uint8_t ch,uint8_t times);
void Get_Adc_Value(uint16_t adcAverageValue[]);
uint16_t Get_Sin_Value(void);
void Control_Output(uint8_t data);
void Control_SEL(uint16_t data);
void USART_print(uint8_t* data,uint16_t len);
void	GPIO_SET_01(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,uint16_t data);

/****************************±äÁ¿¶¨ÒåÇø*******************************************************/
uint16_t frequency_s[16];
uint8_t delay100Ms;
uint8_t ReceiveOK;
uint8_t ReceiveBuffer[64];
uint8_t ReceiveBufferHead;
uint8_t ReceiveBufferTail;
uint8_t ReceiveBufferLen;
uint8_t ReceiveBufferIndex;
uint8_t ReceiveStart;
uint8_t SendOK;

/******************************ÖØ¶¨Ïò¿âº¯ÊýÖÐµÄprintfº¯Êý**********************************/
//#pragma import(__use_no_semihosting) 
//_sys_exit(int x) 
//{ 
//x = x; 
//} 
//struct __FILE 
//{ 
//int handle; 
///* Whatever you require here. If the only file you are using is */ 
///* standard output using printf() for debugging, no file handling */ 
///* is required. */ 
//}; 
///* FILE is typedef’ d in stdio.h. */ 
//FILE __stdout;
//int fputc(int ch, FILE *f)
//{
//			while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
//			USART_SendData(USART1,ch);
//			return ch;
//}

/********************************************************************************************/



/*********************************************************************************************
* name			:		main
* func			:		Ö÷º¯Êý
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
int main()
{
	/**************±äÁ¿ÉùÃ÷²¿·Ö***************/
	uint8_t i,j;//ÁÙÊ±Ñ­»·±äÁ¿
	uint16_t freCount[16][10];//´æ·ÅÊ®ÁùÂ·ÆµÂÊÊ®´Î²ÉÑù½á¹û
	uint16_t adcValue[10][16];//´æ·ÅÊ®´ÎAD²ÉÑùÖµ
	DataStruct dataStruct;//Êý¾Ý´¢´æ½á¹¹Ìå
	uint8_t freIndex;//ÆµÂÊÍ³¼ÆÊý×éµ±Ç°Î»ÖÃ
	RcvData rcvData;//½ÓÊÕÊý¾Ý½á¹¹Ìå
	uint16_t freSum;//ÆµÂÊºÍ
	uint32_t adSum;//ADºÍ

	/***********Ó²¼þ³õÊ¼»¯²¿·Ö***************/
	
	EXTI_Configuration();//ÖÐ¶ÏÏòÁ¿³õÊ¼»¯
	NVIC_Configuration();//ÖÐ¶ÏÍ¨µÀ³õÊ¼»¯
	USART1_Configuration();//³õÊ¼»¯USART
	ADC1_Configuration();//³õÊ¼»¯ADC1
	TIM_Configuration();//³õÊ¼»¯¶¨Ê±Æ÷
		
	/**************±äÁ¿³õÊ¼»¯²¿·Ö*************/
	delay100Ms=1;
	ReceiveOK=0;
	ReceiveBufferHead=0;
	ReceiveBufferTail=0;
	ReceiveBufferLen=7;
	ReceiveBufferIndex=0;
	ReceiveStart=0;
	SendOK=1;
	
	dataStruct.sel=0;
	freIndex=0;
	rcvData.clearFreCount=0;
	rcvData.outPut=0;
	freSum=0;
	adSum=0;
	for(i=0;i<7;i++)
	{
		ReceiveBuffer[i]=0;
	}
	for(i=0;i<16;i++)
	{
		dataStruct.count[i]=0;
		for(j=0;j<10;j++)
		{
			freCount[i][j]=0;
			adcValue[j][i]=0;
		}
	}
	
	/****************³ÌÐòÔËÐÐ²¿·Ö**************/
	while(1)
	{
		if(delay100Ms==0)//Èç¹û100MSµ¹¼ÆÊ±½ØÖ¹£¬½«ÆµÂÊÖµ´¢´æÈëÊý×é
		{
			for(i=0;i<16;i++)//½«ÆµÂÊÖµ´æÈë½á¹¹ÌåÖÐ£¬²¢½«ÆµÂÊÊý¾ÝÇåÁã
			{
				freCount[i][freIndex]=frequency_s[i];
				dataStruct.count[i]+=frequency_s[i];
				frequency_s[i]=0;
			}
			Get_Adc_Value(adcValue[freIndex]);
			delay100Ms=1;
			freIndex+=1;
			if(freIndex>=10)
			{
				freIndex=0;
			}
		}
		if(ReceiveOK==1)//Èç¹û´®¿Ú½ÓÊÕÍê±Ï
		{
			/*¶Á³öÊý¾Ý°üÖÐµÄÊý¾Ý*/
			rcvData.clearFreCount=((ReceiveBuffer[ReceiveBufferHead+1]&0xFE)<<8)|((ReceiveBuffer[ReceiveBufferHead+2]&0xFE)<<1)|((ReceiveBuffer[ReceiveBufferHead+3]&0xC0)>>6);
			rcvData.SEL=((ReceiveBuffer[ReceiveBufferHead+3]&0x3E)<<10)|((ReceiveBuffer[ReceiveBufferHead+4]&0xFE)<<2)|((ReceiveBuffer[ReceiveBufferHead+5]&0xF0)>>4);
			rcvData.outPut=((ReceiveBuffer[ReceiveBufferHead+5]&0x0E)<<4)|((ReceiveBuffer[ReceiveBufferHead+6]&0xf8)>>3);
			
			dataStruct.sel=0;
			dataStruct.head=0xA55A;
			dataStruct.tail=0x5EE5;
			for(i=0;i<16;i++)//¸ù¾ÝÃüÁî¸ø½á¹¹Ìå¸³Öµ
			{				
				/*Fre²¿·Ö*/
				freSum=0;
				adSum=0;
				for(j=0;j<10;j++)
				{
					freSum+=freCount[i][j];
					adSum+=adcValue[j][i];
				}
				dataStruct.Fre[i]=freSum;
				
				/*AD²¿·Ö*/
				dataStruct.AD[i]=(adSum/10);

				/*ÊÇ·ñÇå¿ÕÆµÂÊ¼ÆÊý²¿·Ö*/
				if(rcvData.clearFreCount&(1<<i))//Èç¹ûÎªÕæ£¬Çå¿Õµ±Ç°ÆµÂÊ¼ÆÊýÖµ
				{
					dataStruct.count[i]=0;
				}
				
				/*¸ù¾Ýµ±Ç°ADÖµÈ·¶¨¸ßµÍµçÆ½*/
				if(dataStruct.AD[i]>2048)
					dataStruct.sel|=1<<i;
				else
					dataStruct.sel&=~(0<<i);
			}
			
			/*¸ù¾ÝoutputÈ·¶¨output¶Ë¿ÚÖµ*/
			Control_Output(rcvData.outPut);
			
			/*½«´¦ÀíÊý¾Ý·¢ËÍ¸øÉÏÎ»»ú*/
#ifdef SENDBYINT
			if(SendOK)
				Usart_Send_Struct(dataStruct);
#else
			USART_print((uint8_t*)(&dataStruct),sizeof(dataStruct));//½«½á¹¹ÌåÍ¨¹ý´®¿Ú·¢ËÍ³öÈ¥
#endif
			
			/*½«´®¿Ú½ÓÊÕÊý¾Ý³õÊ¼»¯*/
			ReceiveOK=0;
			ReceiveBufferHead=0;
			ReceiveBufferTail=0;
			ReceiveBufferLen=7;
			ReceiveBufferIndex=0;
		}
	}
}

/*********************************************************************************************
* name			:		Control_Output
* func			:		¿ØÖÆoutput
* para			:		uint8_t
* ret				:		none
* comment		:	
*********************************************************************************************/
void Control_Output(uint8_t data)
{
	/*¸ù¾ÝdataµÄµÍ°ËÎ»È·¶¨out¿ÚµÄÊý¾Ý*/
	GPIO_SET_01(GPIOC,GPIO_Pin_7,(data&(1<<OUTPUT_0_PC7)));
	GPIO_SET_01(GPIOC,GPIO_Pin_8,(data&(1<<OUTPUT_1_PC8)));
	GPIO_SET_01(GPIOC,GPIO_Pin_9,(data&(1<<OUTPUT_2_PC9)));
	GPIO_SET_01(GPIOA,GPIO_Pin_8,(data&(1<<OUTPUT_3_PA8)));
	GPIO_SET_01(GPIOA,GPIO_Pin_9,(data&(1<<OUTPUT_4_PA9)));
	GPIO_SET_01(GPIOA,GPIO_Pin_10,(data&(1<<OUTPUT_5_PA10)));
	GPIO_SET_01(GPIOA,GPIO_Pin_11,(data&(1<<OUTPUT_6_PA11)));
	GPIO_SET_01(GPIOA,GPIO_Pin_12,(data&(1<<OUTPUT_7_PA12)));
}

/*********************************************************************************************
* name			:		Control_SEL
* func			:		¿ØÖÆSEL
* para			:		uint16_t
* ret				:		none
* comment		:	
*********************************************************************************************/
void Control_SEL(uint16_t data)
{
	/*¸ù¾ÝdataÈ·¶¨sel¿ÚµÄÊý¾Ý*/
	GPIO_SET_01(GPIOD,GPIO_Pin_4,(data&(1<<SELPORT_0_PD4)));
	GPIO_SET_01(GPIOD,GPIO_Pin_5,(data&(1<<SELPORT_1_PD5)));
	GPIO_SET_01(GPIOD,GPIO_Pin_6,(data&(1<<SELPORT_2_PD6)));
	GPIO_SET_01(GPIOB,GPIO_Pin_3,(data&(1<<SELPORT_3_PB3)));
	GPIO_SET_01(GPIOB,GPIO_Pin_4,(data&(1<<SELPORT_4_PB4)));
	GPIO_SET_01(GPIOB,GPIO_Pin_5,(data&(1<<SELPORT_5_PB5)));
	GPIO_SET_01(GPIOB,GPIO_Pin_8,(data&(1<<SELPORT_6_PB8)));
	GPIO_SET_01(GPIOB,GPIO_Pin_9,(data&(1<<SELPORT_7_PB9)));
	GPIO_SET_01(GPIOC,GPIO_Pin_12,(data&(1<<SELPORT_8_PC12)));
	GPIO_SET_01(GPIOD,GPIO_Pin_0,(data&(1<<SELPORT_9_PD0)));
	GPIO_SET_01(GPIOD,GPIO_Pin_1,(data&(1<<SELPORT_10_PD1)));
	GPIO_SET_01(GPIOD,GPIO_Pin_2,(data&(1<<SELPORT_11_PD2)));
	GPIO_SET_01(GPIOB,GPIO_Pin_12,(data&(1<<SELPORT_12_PB12)));
	GPIO_SET_01(GPIOB,GPIO_Pin_13,(data&(1<<SELPORT_13_PB13)));
	GPIO_SET_01(GPIOB,GPIO_Pin_14,(data&(1<<SELPORT_14_PB14)));
	GPIO_SET_01(GPIOB,GPIO_Pin_15,(data&(1<<SELPORT_15_PB15)));
}

/*********************************************************************************************
* name			:		GPIO_SET_01()
* func			:		¿ØÖÆIO¿Ú01µçÆ½
* para			:		GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,uint8_t data
* ret				:		none
* comment		:	
*********************************************************************************************/
void	GPIO_SET_01(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,uint16_t data)
{
	if(data!=0x00)
	{
		GPIO_WriteBit(GPIOx,GPIO_Pin,Bit_SET);
	}
	else if(data==0x00)
	{
		GPIO_WriteBit(GPIOx,GPIO_Pin,Bit_RESET);
	}
	else
	{
		return;
	}
}


/*********************************************************************************************
* name			:		Get_Adc
* func			:		»ñµÃADCÖ¸¶¨Í¨µÀµÄÖµ
* para			:		uint8_t
* ret				:		uint16_t
* comment		:	
*********************************************************************************************/
uint16_t Get_Adc(uint8_t ch)   
{
  	//ÉèÖÃÖ¸¶¨ADCµÄ¹æÔò×éÍ¨µÀ£¬Ò»¸öÐòÁÐ£¬²ÉÑùÊ±¼ä
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCÍ¨µÀ,²ÉÑùÊ±¼äÎª239.5ÖÜÆÚ	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//Ê¹ÄÜÖ¸¶¨µÄADC1µÄÈí¼þ×ª»»Æô¶¯¹¦ÄÜ	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//µÈ´ý×ª»»½áÊø

	return ADC_GetConversionValue(ADC1);	//·µ»Ø×î½üÒ»´ÎADC1¹æÔò×éµÄ×ª»»½á¹û
}

/*********************************************************************************************
* name			:		Get_Adc_Average
* func			:		»ñµÃADCÖ¸¶¨Í¨µÀµÄn´Î²É¼¯µÄÆ½¾ùÖµ
* para			:		uint8_t ch,uint8_t times
* ret				:		uint16_t
* comment		:	
*********************************************************************************************/
uint16_t Get_Adc_Average(uint8_t ch,uint8_t times)
{
	uint8_t t;
	uint32_t temp_val=0;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
	}
	return temp_val/times;
} 	 

/*********************************************************************************************
* name			:		Get_Adc_Value
* func			:		½«ADC16¸öÍ¨µÀµÄ²É¼¯Æ½¾ùÖµ´æÈëÊý×éadcAverageValueÖÐ,ÒªÓëÆµÂÊ¶Ë¿ÚºÅÒ»Ò»¶ÔÓ¦
* para			:		none
* ret				:		none
* comment		:	
*********************************************************************************************/
void Get_Adc_Value(uint16_t adcAverageValue[])
{
	adcAverageValue[ADINPORT_0_PA0]=Get_Adc_Average(0,GETADTIMES);
	adcAverageValue[ADINPORT_1_PA1]=Get_Adc_Average(1,GETADTIMES);
	adcAverageValue[ADINPORT_2_PA2]=Get_Adc_Average(2,GETADTIMES);
	adcAverageValue[ADINPORT_3_PA3]=Get_Adc_Average(3,GETADTIMES);
	adcAverageValue[ADINPORT_4_PA4]=Get_Adc_Average(4,GETADTIMES);
	adcAverageValue[ADINPORT_5_PA5]=Get_Adc_Average(5,GETADTIMES);
	adcAverageValue[ADINPORT_6_PA6]=Get_Adc_Average(6,GETADTIMES);
	adcAverageValue[ADINPORT_7_PA7]=Get_Adc_Average(7,GETADTIMES);
	adcAverageValue[ADINPORT_8_PB0]=Get_Adc_Average(8,GETADTIMES);
	adcAverageValue[ADINPORT_9_PB1]=Get_Adc_Average(9,GETADTIMES);
	adcAverageValue[ADINPORT_10_PC0]=Get_Adc_Average(10,GETADTIMES);
	adcAverageValue[ADINPORT_11_PC1]=Get_Adc_Average(11,GETADTIMES);
	adcAverageValue[ADINPORT_12_PC2]=Get_Adc_Average(12,GETADTIMES);
	adcAverageValue[ADINPORT_13_PC3]=Get_Adc_Average(13,GETADTIMES);
	adcAverageValue[ADINPORT_14_PC4]=Get_Adc_Average(14,GETADTIMES);
	adcAverageValue[ADINPORT_15_PC5]=Get_Adc_Average(15,GETADTIMES);
}
