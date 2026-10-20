#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#include "system.h"

void KEY_Init(void);
u8 click(void);
void Delay_ms(void);
u8 click_N_Double (u8 time);
u8 click_N_Double_MPU6050 (u8 time);
u8 Long_Press(void);

/*--------KEY control pin--------*/
#define KEY1_PORT	GPIOC
#define KEY1_PIN	GPIO_Pin_2
#define KEY1		PCin(2) 

#define KEY2_PORT	GPIOB
#define KEY2_PIN	GPIO_Pin_14
#define KEY2		PBin(14) 

#define KEY3_PORT	GPIOC
#define KEY3_PIN	GPIO_Pin_3
#define KEY3		PCin(3) 


/*----------------------------------*/
#define DIP1_PORT	GPIOC
#define DIP1_PIN    GPIO_Pin_15
#define DIP1		PCin(15) 

#define DIP2_PORT	GPIOE
#define DIP2_PIN    GPIO_Pin_1
#define DIP2		PEin(1) 

#define DIP3_PORT	GPIOA
#define DIP3_PIN    GPIO_Pin_11
#define DIP3		PAin(11) 

#define DIP4_PORT	GPIOA
#define DIP4_PIN    GPIO_Pin_12
#define DIP4		PAin(12) 



#endif 
