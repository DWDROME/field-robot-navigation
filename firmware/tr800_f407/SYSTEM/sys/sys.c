#include "sys.h"  


//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
void WFI_SET(void)
{
	__asm volatile ("wfi");
}
//关闭所有中断(但是不包括fault和NMI中断)
void INTX_DISABLE(void)
{
	__asm volatile ("cpsid i" ::: "memory");
}
//开启所有中断
void INTX_ENABLE(void)
{
	__asm volatile ("cpsie i" ::: "memory");
}
//设置栈顶地址
//addr:栈顶地址
void MSR_MSP(u32 addr)
{
	__asm volatile ("msr msp, %0" :: "r"(addr) : "memory");
}
















