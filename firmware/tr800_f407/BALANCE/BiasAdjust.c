#include "BiasAdjust.h"
int    FrontAdjust=0,BackAdjust=0;
double FL_scale = 1.0;
double FR_scale = 1.0;
double BL_scale = 1.0;
double BR_scale = 1.0;

u16  DataAddress[2] = {0};
#define READ_FRONT_ADDR  0x08020000
#define WRITE_FRONT_ADDR 0x08020000  

#define READ_BACK_ADDR   0x08040000  
#define WRITE_BACK_ADDR  0x08040000
//#define SIZE sizeof(DataAddress)

#define TEXT_LENTH sizeof(DataAddress)	 		  	//数组长度	
#define SIZE TEXT_LENTH/4+((TEXT_LENTH%4)?1:0)




u16 f_bias,b_bias=200;
u8 data_h,data_l;
#define BIAS_SACLE   0.90

double dl_map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ProjectModeGpioInit(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
   GPIO_InitStructure.GPIO_Pin	   = GPIO_Pin_15;
   GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IN;
   GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;
   GPIO_Init(GPIOC, &GPIO_InitStructure);
   GPIO_ResetBits(GPIOC,GPIO_Pin_15);

   RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
   GPIO_InitStructure.GPIO_Pin	   = GPIO_Pin_1;
   GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IN;
   GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;
   GPIO_Init(GPIOE, &GPIO_InitStructure);
   GPIO_ResetBits(GPIOE,GPIO_Pin_1);
}



#if BIAS_MODE_0
int IsEnterProjectMode1(void)
{
   int ProjectMode;
   ProjectMode = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15);
   return ProjectMode;

}

int IsEnterProjectMode2(void)
{
   int ProjectMode;
   ProjectMode = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1);
   return ProjectMode;
}

void BiasAdjust(void)
{  
   if(FrontAdjust == 1){
	 	
	       if(RC_Bias > 1500)
	       	{	   
	           FR_scale = dl_map(2000-RC_Bias,0,500,BIAS_SACLE,1.0);
	           FL_scale = 1.0;
		    }
	       else if(RC_Bias <= 1500)
	       	{
			   FR_scale = 1.0;
	           FL_scale = dl_map(RC_Bias-1000,0,500,BIAS_SACLE,1.0);	          
		    }    
           
     	}  
     if(BackAdjust == 1){
	 	   
			 if(RC_Bias > 1500)
			   {	 
				 BR_scale = dl_map(2000-RC_Bias,0,500,BIAS_SACLE,1.0);
				 BL_scale = 1.0;
			  }
			 else if(RC_Bias <= 1500)
			  {
				 BR_scale = 1.0;
				 BL_scale = dl_map(RC_Bias-1000,0,500,BIAS_SACLE,1.0);		
			  }
			 
	     }
	 
}

void BiasadjustSave(void)
{
    if(FrontAdjust == 1 && IsEnterProjectMode1() == 1)
      {      
		 DataAddress[0] = RC_Bias;
		 USART_Cmd(USART2, DISABLE);	
	     STMFLASH_Write(WRITE_FRONT_ADDR,(u16*)DataAddress,SIZE);
		 USART_Cmd(USART2, ENABLE);	
	     Buzzer = 1;
	     delay_ms(500);
	     Buzzer = 0;
	     FrontAdjust = 0;
	  }
    if(BackAdjust == 1 && IsEnterProjectMode2() == 1)
   	{
		DataAddress[0] = RC_Bias;
		USART_Cmd(USART2, DISABLE);	
	    STMFLASH_Write(WRITE_BACK_ADDR,(u16*)DataAddress,SIZE);
		USART_Cmd(USART2, ENABLE);	
		BackAdjust = 0; 
	    Buzzer = 1;
		delay_ms(500);
	    Buzzer = 0;	
    }

}

void Bias_check(void)
{
	ProjectModeGpioInit();
	if(IsEnterProjectMode1() == 0)   FrontAdjust = 1;
	if(IsEnterProjectMode2() == 0)   BackAdjust = 1; 
	if(FrontAdjust == 0)
	{
		 STMFLASH_Read(READ_FRONT_ADDR,(u16*)DataAddress,1);
		 f_bias = DataAddress[0];
		 
		 if(f_bias > 2000) f_bias = 1500;
		 else if(f_bias < 1000) b_bias = 1500;
		 if(f_bias > 1500)
		  { 	
			FR_scale = dl_map(2000-f_bias,0,500,BIAS_SACLE,1.0);
			FL_scale = 1.0;
		  }
		 else if(f_bias <= 1500)
		  {
			FR_scale = 1.0;
			FL_scale = dl_map(f_bias-1000,0,500,BIAS_SACLE,1.0);			   
		  }  

		
	}
     if( BackAdjust == 0)
	  {
		STMFLASH_Read(READ_BACK_ADDR,(u16*)DataAddress,1);
		b_bias = DataAddress[0];

			if(b_bias > 2000) b_bias = 1500;
			else if(b_bias < 1000) b_bias = 1500;
			if(b_bias > 1500)
			  { 	
				BR_scale = dl_map(2000-b_bias,0,500,BIAS_SACLE,1.0);
				BL_scale = 1.0;
			  }
			 else if(b_bias <= 1500)
			  {
				BR_scale = 1.0;
				BL_scale = dl_map(b_bias-1000,0,500,BIAS_SACLE,1.0);	   
			  }   
		 
	  }
}

#else
bool EnterProjectMode1 = false;
bool EnterProjectMode2 = false;
int Pulsewidth_Bias_f =  1500;
int Pulsewidth_Bias_b =  1500;

bool IsEnterProjectMode2(void)
{
  return EnterProjectMode2;

}

bool IsEnterProjectMode1(void)
{
   return EnterProjectMode1;
}

void BiasAdjust(void)
{  
   if(FrontAdjust == 1){

     
		if(Pulsewidth_Bias_f > 1500)
       	{	   
           FR_scale = dl_map(2000-Pulsewidth_Bias_f,0,500,BIAS_SACLE,1.0);
           FL_scale = 1.0;
	    }
       else if(Pulsewidth_Bias_f <= 1500)
       	{
		   FR_scale = 1.0;
           FL_scale = dl_map(Pulsewidth_Bias_f-1000,0,500,BIAS_SACLE,1.0);	          
	    }   		        	       
      }  
     if(BackAdjust == 1){

         if(Pulsewidth_Bias_b > 1500)
            {	 
         	 BR_scale = dl_map(2000-Pulsewidth_Bias_b,0,500,BIAS_SACLE,1.0);
         	 BL_scale = 1.0;
           }
          else if(Pulsewidth_Bias_b <= 1500)
           {
         	 BR_scale = 1.0;
         	 BL_scale = dl_map(Pulsewidth_Bias_b-1000,0,500,BIAS_SACLE,1.0);		
           }  	
        }
}

void BiasadjustSave(void)
{
    if(FrontAdjust == 1 && IsEnterProjectMode1() == false)
      {      
		 DataAddress[0] = Pulsewidth_Bias_f;
		 USART_Cmd(USART2, DISABLE);
	     STMFLASH_Write(WRITE_FRONT_ADDR,(u16*)DataAddress,SIZE);
		 USART_Cmd(USART2, ENABLE);
		 
		 FrontAdjust = 0;
			  
	  }
    if(BackAdjust == 1 && IsEnterProjectMode2() == false)
   	{
		DataAddress[0] = Pulsewidth_Bias_b;
		USART_Cmd(USART2, DISABLE);
	    STMFLASH_Write(WRITE_BACK_ADDR,(u16*)DataAddress,SIZE);
		USART_Cmd(USART2, ENABLE);
		BackAdjust = 0; 
	
    }

}

void BiasAdjustClear(void)
{
	DataAddress[0] = 1500;
	USART_Cmd(USART2, DISABLE);
    STMFLASH_Write(WRITE_FRONT_ADDR,(u16*)DataAddress,SIZE);
    STMFLASH_Write(WRITE_BACK_ADDR,(u16*)DataAddress,SIZE); 
	USART_Cmd(USART2, ENABLE);  
}

void Bias_check(void)
{
     STMFLASH_Read(READ_FRONT_ADDR,(u16*)DataAddress,SIZE);
     f_bias = DataAddress[0];
	  
      if(f_bias > 2000) f_bias = 1500;
      else if(f_bias < 1000) b_bias = 1500;
      if(f_bias > 1500)
       { 	
     	FR_scale = dl_map(2000-f_bias,0,500,BIAS_SACLE,1.0);
     	FL_scale = 1.0;
       }
      else if(f_bias <= 1500)
       {
     	FR_scale = 1.0;
     	FL_scale = dl_map(f_bias-1000,0,500,BIAS_SACLE,1.0);			   
       }  
	
     STMFLASH_Read(READ_BACK_ADDR,(u16*)DataAddress,SIZE);
     b_bias = DataAddress[0];

     if(b_bias > 2000) b_bias = 1500;
     else if(b_bias < 1000) b_bias = 1500;
     if(b_bias > 1500)
       { 	
     	BR_scale = dl_map(2000-b_bias,0,500,BIAS_SACLE,1.0);
     	BL_scale = 1.0;
       }
     else if(b_bias <= 1500)
       {
     	BR_scale = 1.0;
     	BL_scale = dl_map(b_bias-1000,0,500,BIAS_SACLE,1.0);	   
       }    	   
}
#endif

