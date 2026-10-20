#include "sys.h"  


//THUMBÖ¸Áî²»Ö§³Ö»ã±àÄÚÁª
//²ÉÓÃÈçÏÂ·½·¨ÊµÏÖÖ´ÐÐ»ã±àÖ¸ÁîWFI  
void WFI_SET(void)
{
	__asm volatile ("wfi");
}
//¹Ø±ÕËùÓÐÖÐ¶Ï(µ«ÊÇ²»°üÀ¨faultºÍNMIÖÐ¶Ï)
void INTX_DISABLE(void)
{
	__asm volatile ("cpsid i" ::: "memory");
}
//¿ªÆôËùÓÐÖÐ¶Ï
void INTX_ENABLE(void)
{
	__asm volatile ("cpsie i" ::: "memory");
}
//ÉèÖÃÕ»¶¥µØÖ·
//addr:Õ»¶¥µØÖ·
void MSR_MSP(u32 addr)
{
	__asm volatile ("msr msp, %0" :: "r"(addr) : "memory");
}
















