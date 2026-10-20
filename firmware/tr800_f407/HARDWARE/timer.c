#include "timer.h"

//Input the capture flag for channel 1, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//Í¨µÀ1ÊäÈë²¶»ñ±êÖ¾£¬¸ßÁ½Î»×ö²¶»ñ±êÖ¾£¬µÍ6Î»×öÒç³ö±êÖ¾		
u8 TIM1CH1_CAPTURE_STA = 0;	
u16 TIM1CH1_CAPTURE_UPVAL;
u16 TIM1CH1_CAPTURE_DOWNVAL;

//Input the capture flag for channel 2, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//Í¨µÀ2ÊäÈë²¶»ñ±êÖ¾£¬¸ßÁ½Î»×ö²¶»ñ±êÖ¾£¬µÍ6Î»×öÒç³ö±êÖ¾	
u8 TIM1CH2_CAPTURE_STA = 0;		
u16 TIM1CH2_CAPTURE_UPVAL;
u16 TIM1CH2_CAPTURE_DOWNVAL;

//Input the capture flag for channel 3, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//Í¨µÀ3ÊäÈë²¶»ñ±êÖ¾£¬¸ßÁ½Î»×ö²¶»ñ±êÖ¾£¬µÍ6Î»×öÒç³ö±êÖ¾	
u8 TIM1CH3_CAPTURE_STA = 0;		
u16 TIM1CH3_CAPTURE_UPVAL;
u16 TIM1CH3_CAPTURE_DOWNVAL;

//Input the capture flag for channel 4, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//Í¨µÀ4ÊäÈë²¶»ñ±êÖ¾£¬¸ßÁ½Î»×ö²¶»ñ±êÖ¾£¬µÍ6Î»×öÒç³ö±êÖ¾
u8 TIM1CH4_CAPTURE_STA = 0;			
u16 TIM1CH4_CAPTURE_UPVAL;
u16 TIM1CH4_CAPTURE_DOWNVAL;

u32 TIM1_T1;
u32 TIM1_T2;
u32 TIM1_T3;
u32 TIM1_T4;

//Variables related to remote control acquisition of model aircraft
//º½Ä£Ò£¿Ø²É¼¯Ïà¹Ø±äÁ¿
int Remoter_Ch1=1500,Remoter_Ch2=1500,Remoter_Ch3=1500,Remoter_Ch4=1500;
//Model aircraft remote control receiver variable
//º½Ä£Ò£¿Ø½ÓÊÕ±äÁ¿
int L_Remoter_Ch1=1500,L_Remoter_Ch2=1500,L_Remoter_Ch3=1500,L_Remoter_Ch4=1500;  

/**************************************************************************
Function: Model aircraft remote control initialization function, timer 1 input capture initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
º¯Êý¹¦ÄÜ£ºº½Ä£Ò£¿Ø³õÊ¼»¯º¯Êý£¬¶¨Ê±Æ÷1ÊäÈë²¶»ñ³õÊ¼»¯
Èë¿Ú²ÎÊý£ºarr£º×Ô¶¯ÖØ×°Öµ£¬psc£ºÊ±ÖÓÔ¤·ÖÆµÊý 
·µ »Ø Öµ£ºÎÞ
**************************************************************************/ 
void TIM1_Cap_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1Ê±ÖÓÊ¹ÄÜ    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//Ê¹ÄÜPORTEÊ±ÖÓ	
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14; //GPIOA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//¸´ÓÃ¹¦ÄÜ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//ËÙ¶È100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //ÍÆÍì¸´ÓÃÊä³ö
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //ÏÂÀ­
	GPIO_Init(GPIOE,&GPIO_InitStructure); 

	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);

	/*** Initialize timer 1 || ³õÊ¼»¯¶¨Ê±Æ÷1 ***/
	//Set the counter to automatically reload //Éè¶¨¼ÆÊýÆ÷×Ô¶¯ÖØ×°Öµ 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Pre-divider //Ô¤·ÖÆµÆ÷ 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	
	//Set the clock split: TDTS = Tck_tim //ÉèÖÃÊ±ÖÓ·Ö¸î:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	//TIM up count mode //TIMÏòÉÏ¼ÆÊýÄ£Ê½	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TimeBaseInitStruct
	//¸ù¾ÝTIM_TimeBaseInitStructÖÐÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯TIMxµÄÊ±¼ä»ùÊýµ¥Î»
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 

	/*** ³õÊ¼»¯TIM1ÊäÈë²¶»ñ²ÎÊý£¬Í¨µÀ1 || Initialize TIM1 for the capture parameter, channel 1 ***/
	//Select input //Ñ¡ÔñÊäÈë¶Ë 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
  //Rising edge capture //ÉÏÉýÑØ²¶»ñ
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency //ÅäÖÃÊäÈë·ÖÆµ,²»·ÖÆµ 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
  //IC1F=0000 Configure input filter //ÅäÖÃÊäÈëÂË²¨Æ÷
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;	  
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	/*** ³õÊ¼»¯TIM1ÊäÈë²¶»ñ²ÎÊý£¬Í¨µÀ2 || Initialize TIM1 for the capture parameter, channel 2 ***/
	//CC1S=01 Select input //Ñ¡ÔñÊäÈë¶Ë 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	//Rising edge capture //ÉÏÉýÑØ²¶»ñ
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency //ÅäÖÃÊäÈë·ÖÆµ,²»·ÖÆµ 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ÅäÖÃÊäÈëÂË²¨Æ÷
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	/*** ³õÊ¼»¯TIM1ÊäÈë²¶»ñ²ÎÊý£¬Í¨µÀ3 || Initialize TIM1 for the capture parameter, channel 3 ***/
	//Select input //Ñ¡ÔñÊäÈë¶Ë 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;   
	//Rising edge capture //ÉÏÉýÑØ²¶»ñ
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency //ÅäÖÃÊäÈë·ÖÆµ,²»·ÖÆµ 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  
	//IC1F=0000 Configure input filter //ÅäÖÃÊäÈëÂË²¨Æ÷£¬²»ÂË²¨  
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	/*** ³õÊ¼»¯TIM1ÊäÈë²¶»ñ²ÎÊý£¬Í¨µÀ4 || Initialize TIM1 for the capture parameter, channel 4 ***/
	//Select input //Ñ¡ÔñÊäÈë¶Ë 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 
	//Rising edge capture //ÉÏÉýÑØ²¶»ñ
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency //ÅäÖÃÊäÈë·ÖÆµ,²»·ÖÆµ 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	   
	//IC1F=0000 Configure input filter //ÅäÖÃÊäÈëÂË²¨Æ÷£¬²»ÂË²¨  
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

  /*** interrupt packet initialization || ÖÐ¶Ï·Ö×é³õÊ¼»¯ ***/
  //TIM1 interrupts //TIM1ÖÐ¶Ï
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn; 
  //Preempt priority 0 //ÏÈÕ¼ÓÅÏÈ¼¶0¼¶	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	//Level 0 from priority //´ÓÓÅÏÈ¼¶0¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	//IRQ channels are enabled //IRQÍ¨µÀ±»Ê¹ÄÜ
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	//Initializes the peripheral NVIC register according to the parameters specified in NVIC_InitStruct
	//¸ù¾ÝNVIC_InitStructÖÐÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯ÍâÉèNVIC¼Ä´æÆ÷ 
	NVIC_Init(&NVIC_InitStructure);   
	
  //Allow CC1IE,CC2IE,CC3IE,CC4IE to catch interrupts, not allowed update_interrupts
  //²»ÔÊÐí¸üÐÂÖÐ¶Ï£¬ÔÊÐíCC1IE,CC2IE,CC3IE,CC4IE²¶»ñÖÐ¶Ï	
	TIM_ITConfig(TIM1, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE);   
	//Advanced timer output must be enabled //¸ß¼¶¶¨Ê±Æ÷Êä³ö±ØÐëÊ¹ÄÜÕâ¾ä	
	TIM_CtrlPWMOutputs(TIM1,ENABLE); 	
	//Enable timer //Ê¹ÄÜ¶¨Ê±Æ÷
	TIM_Cmd(TIM1, ENABLE); 		
}
/**************************************************************************
Function: Model aircraft remote control receiving interrupt, namely timer 1 input capture interrupt
Input   : none
Output  : none
º¯Êý¹¦ÄÜ£ºº½Ä£Ò£¿Ø½ÓÊÕÖÐ¶Ï£¬¼´¶¨Ê±Æ÷1ÊäÈë²¶»ñÖÐ¶Ï
Èë¿Ú²ÎÊý£ºÎÞ
·µ »Ø Öµ£ºÎÞ
**************************************************************************/ 
void TIM1_CC_IRQHandler(void)
{
	//Á¬½Óº½Ä£Ò£Ò£¿ØÆ÷ºó£¬ÐèÒªÍÆÏÂÇ°½ø¸Ë£¬²Å¿ÉÒÔÕýÊ½º½Ä£¿ØÖÆÐ¡³µ
	//After connecting the remote controller of the model aircraft, 
	//you need to push down the forward lever to officially control the car of the model aircraft
	static u8 ch1_filter_times=0,ch2_filter_times=0,ch3_filter_times=0,ch4_filter_times=0;
	// First-stage TR800 runtime owns control-source switching.
	// Keep this interrupt for capture only.
	 
	//Channel 1 //Í¨µÀÒ»
	if ((TIM1CH1_CAPTURE_STA & 0X80) == 0) 			
	{
		if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET) //A capture event occurred on channel 1 //Í¨µÀ1·¢Éú²¶»ñÊÂ¼þ
		{
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); //Clear the interrupt flag bit //Çå³ýÖÐ¶Ï±êÖ¾Î»
			if (TIM1CH1_CAPTURE_STA & 0X40)	//A falling edge is caught //²¶»ñµ½Ò»¸öÏÂ½µÑØ
			{
				TIM1CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM1); //Record the timer value at this point //¼ÇÂ¼ÏÂ´ËÊ±µÄ¶¨Ê±Æ÷¼ÆÊýÖµ
				if (TIM1CH1_CAPTURE_DOWNVAL < TIM1CH1_CAPTURE_UPVAL)
				{
					TIM1_T1 = 9999;
				}
				else
					TIM1_T1 = 0;
				Remoter_Ch1 = TIM1CH1_CAPTURE_DOWNVAL - TIM1CH1_CAPTURE_UPVAL + TIM1_T1;	//Time to get the total high level //µÃµ½×ÜµÄ¸ßµçÆ½µÄÊ±¼ä


                if(abs(Remoter_Ch1-L_Remoter_Ch1)>500)
				{
					ch1_filter_times++;
					if( ch1_filter_times<=5 ) Remoter_Ch1=L_Remoter_Ch1; //Filter //ÂË²¨	
					else ch1_filter_times=0;
				}
				else
				{
					ch1_filter_times=0;
				}
				L_Remoter_Ch1=Remoter_Ch1;
				TIM1CH1_CAPTURE_STA = 0; //Capture flag bit to zero	//²¶»ñ±êÖ¾Î»ÇåÁã
				TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Rising); //Set to rising edge capture //ÉèÖÃÎªÉÏÉýÑØ²¶»ñ		  
			}
			else 
			{
				//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				//·¢Éú²¶»ñÊ±¼äµ«²»ÊÇÏÂ½µÑØ£¬µÚÒ»´Î²¶»ñµ½ÉÏÉýÑØ£¬¼ÇÂ¼´ËÊ±µÄ¶¨Ê±Æ÷¼ÆÊýÖµ
				TIM1CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM1); //Obtain rising edge data //»ñÈ¡ÉÏÉýÑØÊý¾Ý
				TIM1CH1_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //±ê¼ÇÒÑ²¶»ñµ½ÉÏÉýÑØ
				TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Falling); //Set to Falling Edge Capture //ÉèÖÃÎªÏÂ½µÑØ²¶»ñ
			}
		}
	}
  //Channel 2 //Í¨µÀ¶þ
	if ((TIM1CH2_CAPTURE_STA & 0X80) == 0)		
	{
		if (TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)	//A capture event occurred on channel 2 //Í¨µÀ2·¢Éú²¶»ñÊÂ¼þ
		{
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC2); //Clear the interrupt flag bit //Çå³ýÖÐ¶Ï±êÖ¾Î»
			if (TIM1CH2_CAPTURE_STA & 0X40)	//A falling edge is caught //²¶»ñµ½Ò»¸öÏÂ½µÑØ
			{
				TIM1CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM1); //Record the timer value at this point //¼ÇÂ¼ÏÂ´ËÊ±µÄ¶¨Ê±Æ÷¼ÆÊýÖµ
				if (TIM1CH2_CAPTURE_DOWNVAL < TIM1CH2_CAPTURE_UPVAL)
				{
					TIM1_T2 = 9999;
				}
				else
					TIM1_T2 = 0;
				if(abs(Remoter_Ch2-L_Remoter_Ch2)>500)
				{
					ch2_filter_times++;
					if( ch2_filter_times<=5 ) Remoter_Ch2=L_Remoter_Ch2; //Filter //ÂË²¨	
					else ch2_filter_times=0;
				}
				else
				{
					ch2_filter_times=0;
				}
				
				L_Remoter_Ch2=Remoter_Ch2;
				
				TIM1CH2_CAPTURE_STA = 0; //Capture flag bit to zero	//²¶»ñ±êÖ¾Î»ÇåÁã
				TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Rising); //Set to rising edge capture //ÉèÖÃÎªÉÏÉýÑØ²¶»ñ		  
			}
			else 
			{
				//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				//·¢Éú²¶»ñÊ±¼äµ«²»ÊÇÏÂ½µÑØ£¬µÚÒ»´Î²¶»ñµ½ÉÏÉýÑØ£¬¼ÇÂ¼´ËÊ±µÄ¶¨Ê±Æ÷¼ÆÊýÖµ
				TIM1CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM1); //Obtain rising edge data //»ñÈ¡ÉÏÉýÑØÊý¾Ý
				TIM1CH2_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //±ê¼ÇÒÑ²¶»ñµ½ÉÏÉýÑØ
				TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Falling); //Set to Falling Edge Capture //ÉèÖÃÎªÏÂ½µÑØ²¶»ñ
			}
		}
	}
  //Channel 3 //Í¨µÀÈý
	if ((TIM1CH3_CAPTURE_STA & 0X80) == 0)			
	{
		if (TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)	//A capture event occurred on channel 3 //Í¨µÀ3·¢Éú²¶»ñÊÂ¼þ
		{
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC3); //Clear the interrupt flag bit //Çå³ýÖÐ¶Ï±êÖ¾Î»
			if (TIM1CH3_CAPTURE_STA & 0X40)	//A falling edge is caught //²¶»ñµ½Ò»¸öÏÂ½µÑØ
			{
				TIM1CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM1); //Record the timer value at this point //¼ÇÂ¼ÏÂ´ËÊ±µÄ¶¨Ê±Æ÷¼ÆÊýÖµ
				if (TIM1CH3_CAPTURE_DOWNVAL < TIM1CH3_CAPTURE_UPVAL)
				{
					TIM1_T3 = 9999;
				}
				else
					TIM1_T3 = 0;
				Remoter_Ch3 = TIM1CH3_CAPTURE_DOWNVAL - TIM1CH3_CAPTURE_UPVAL + TIM1_T3; //Time to get the total high level //µÃµ½×ÜµÄ¸ßµçÆ½µÄÊ±¼ä

                if(abs(Remoter_Ch3-L_Remoter_Ch3)>500)
				{
					ch3_filter_times++;
					if( ch3_filter_times<=5 ) Remoter_Ch3=L_Remoter_Ch3; //Filter //ÂË²¨	
					else ch3_filter_times=0;
				}
				else
				{
					ch3_filter_times=0;
				}
				
				L_Remoter_Ch3=Remoter_Ch3;
				
				TIM1CH3_CAPTURE_STA = 0; //Capture flag bit to zero	//²¶»ñ±êÖ¾Î»ÇåÁã
				TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Rising); //Set to rising edge capture //ÉèÖÃÎªÉÏÉýÑØ²¶»ñ		  
			}
			else 
			{
				//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				//·¢Éú²¶»ñÊ±¼äµ«²»ÊÇÏÂ½µÑØ£¬µÚÒ»´Î²¶»ñµ½ÉÏÉýÑØ£¬¼ÇÂ¼´ËÊ±µÄ¶¨Ê±Æ÷¼ÆÊýÖµ
				TIM1CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM1); //Obtain rising edge data //»ñÈ¡ÉÏÉýÑØÊý¾Ý
				TIM1CH3_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //±ê¼ÇÒÑ²¶»ñµ½ÉÏÉýÑØ
				TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Falling); //Set to Falling Edge Capture //ÉèÖÃÎªÏÂ½µÑØ²¶»ñ
			}
		}
	}
	
	if(Car_Mode!=Akm_Car) //Ackerman car uses channel 4 of timer 1 as the PWM output control Servo //°¢¿ËÂüÐ¡³µ°Ñ¶¨Ê±Æ÷1µÄÍ¨µÀ4ÓÃ×÷ÁËPWMÊä³ö¿ØÖÆ¶æ»ú
	{
		//Channel 4 //Í¨µÀËÄ
		if ((TIM1CH4_CAPTURE_STA & 0X80) == 0)		
		{
			if (TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)	//A capture event occurred on channel 4 //Í¨µÀ4·¢Éú²¶»ñÊÂ¼þ
			{
				TIM_ClearITPendingBit(TIM1, TIM_IT_CC4); //Clear the interrupt flag bit //Çå³ýÖÐ¶Ï±êÖ¾Î»
				if (TIM1CH4_CAPTURE_STA & 0X40)	//A falling edge is caught //²¶»ñµ½Ò»¸öÏÂ½µÑØ
				{
					TIM1CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM1); //Record the timer value at this point //¼ÇÂ¼ÏÂ´ËÊ±µÄ¶¨Ê±Æ÷¼ÆÊýÖµ
					if (TIM1CH4_CAPTURE_DOWNVAL < TIM1CH4_CAPTURE_UPVAL)
					{
						TIM1_T4 = 9999;
					}
					else
						TIM1_T4 = 0;
					Remoter_Ch4 = TIM1CH4_CAPTURE_DOWNVAL - TIM1CH4_CAPTURE_UPVAL + TIM1_T4; //Time to get the total high level //µÃµ½×ÜµÄ¸ßµçÆ½µÄÊ±¼ä
					if(abs(Remoter_Ch4-L_Remoter_Ch4)>500)
					{
						ch4_filter_times++;
						if( ch4_filter_times<=5 ) Remoter_Ch4=L_Remoter_Ch4; //Filter //ÂË²¨	
						else ch4_filter_times=0;
					}
					else
					{
						ch4_filter_times=0;
					}
					
					L_Remoter_Ch4=Remoter_Ch4;					
					TIM1CH4_CAPTURE_STA = 0; //Capture flag bit to zero	//²¶»ñ±êÖ¾Î»ÇåÁã
					TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Rising); //Set to rising edge capture //ÉèÖÃÎªÉÏÉýÑØ²¶»ñ		  
				}
				else 
				{
					//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				  //·¢Éú²¶»ñÊ±¼äµ«²»ÊÇÏÂ½µÑØ£¬µÚÒ»´Î²¶»ñµ½ÉÏÉýÑØ£¬¼ÇÂ¼´ËÊ±µÄ¶¨Ê±Æ÷¼ÆÊýÖµ
					TIM1CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM1); //Obtain rising edge data //»ñÈ¡ÉÏÉýÑØÊý¾Ý
					TIM1CH4_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //±ê¼ÇÒÑ²¶»ñµ½ÉÏÉýÑØ
					TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Falling); //Set to Falling Edge Capture //ÉèÖÃÎªÏÂ½µÑØ²¶»ñ
				}
			}
		}
  }
}
/**************************************************************************
Function: TIM1 Update Interrupt
Input   : none
Output  : none
º¯Êý¹¦ÄÜ£º¶¨Ê±Æ÷1¸üÐÂÖÐ¶Ï
Èë¿Ú²ÎÊý£ºÎÞ
·µ»Ø  Öµ£ºÎÞ 
**************************************************************************/
void TIM1_UP_TIM10_IRQHandler(void) 
{ 
	//Clear the interrupt flag bit
	//Çå³ýÖÐ¶Ï±êÖ¾Î» 
  TIM1->SR&=~(1<<0);	    
}

