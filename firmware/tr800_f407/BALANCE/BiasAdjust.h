#ifndef __BIASADJUST_H
#define __BIASADJUST_H
#include "delay.h"
#include "sys.h"
#include "system.h"
     
extern double FL_scale;
extern double FR_scale;
extern double BL_scale;
extern double BR_scale;
extern int FrontAdjust,BackAdjust;
extern u16 DataAddress[2];
extern u16 f_bias,b_bias;
extern int Pulsewidth_Bias_f;
extern int Pulsewidth_Bias_b;
extern bool EnterProjectMode1;
extern bool EnterProjectMode2;

void BiasAdjust(void);
void BiasadjustSave(void);
void Bias_check(void);

//int IsEnterProjectMode1(void);
//int IsEnterProjectMode2(void);

#endif

