/*
 * FreeRTOS Kernel
 * SPDX-License-Identifier: MIT
 */

#include "FreeRTOS.h"
#include "task.h"

#ifndef __ARM_FP
    #error This port can only be used when hardware floating point support is enabled.
#endif

#if ( configMAX_SYSCALL_INTERRUPT_PRIORITY == 0 )
    #error configMAX_SYSCALL_INTERRUPT_PRIORITY must not be zero.
#endif

#ifndef configSYSTICK_CLOCK_HZ
    #define configSYSTICK_CLOCK_HZ    configCPU_CLOCK_HZ
    #define portNVIC_SYSTICK_CLK_BIT   ( 1UL << 2UL )
#else
    #define portNVIC_SYSTICK_CLK_BIT   ( 0 )
#endif

#ifndef configOVERRIDE_DEFAULT_TICK_CONFIGURATION
    #define configOVERRIDE_DEFAULT_TICK_CONFIGURATION    0
#endif

#ifdef configTASK_RETURN_ADDRESS
    #define portTASK_RETURN_ADDRESS    configTASK_RETURN_ADDRESS
#else
    #define portTASK_RETURN_ADDRESS    prvTaskExitError
#endif

#define portNVIC_SYSTICK_CTRL_REG             ( *( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG             ( *( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG    ( *( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SHPR2_REG                    ( *( ( volatile uint32_t * ) 0xe000ed1c ) )
#define portNVIC_SHPR3_REG                    ( *( ( volatile uint32_t * ) 0xe000ed20 ) )

#define portNVIC_SYSTICK_INT_BIT              ( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT           ( 1UL << 0UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT       ( 1UL << 16UL )
#define portNVIC_PENDSVCLEAR_BIT              ( 1UL << 27UL )

#define portCPUID                             ( *( ( volatile uint32_t * ) 0xE000ed00 ) )
#define portCORTEX_M7_r0p1_ID                 ( 0x410FC271UL )
#define portCORTEX_M7_r0p0_ID                 ( 0x410FC270UL )

#define portNVIC_PENDSV_PRI                   ( ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 16UL )
#define portNVIC_SYSTICK_PRI                  ( ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 24UL )

#define portFIRST_USER_INTERRUPT_NUMBER       ( 16 )
#define portNVIC_IP_REGISTERS_OFFSET_16       ( 0xE000E3F0 )
#define portAIRCR_REG                         ( *( ( volatile uint32_t * ) 0xE000ED0C ) )
#define portMAX_8_BIT_VALUE                   ( ( uint8_t ) 0xff )
#define portTOP_BIT_OF_BYTE                   ( ( uint8_t ) 0x80 )
#define portMAX_PRIGROUP_BITS                 ( ( uint8_t ) 7 )
#define portPRIORITY_GROUP_MASK               ( 0x07UL << 8UL )
#define portPRIGROUP_SHIFT                    ( 8UL )

#define portVECTACTIVE_MASK                   ( 0xFFUL )

#define portFPCCR                             ( ( volatile uint32_t * ) 0xe000ef34 )
#define portASPEN_AND_LSPEN_BITS              ( 0x3UL << 30UL )

#define portINITIAL_XPSR                      ( 0x01000000 )
#define portINITIAL_EXEC_RETURN               ( 0xfffffffd )

#define portMAX_24_BIT_NUMBER                 ( 0xffffffUL )
#define portMISSED_COUNTS_FACTOR              ( 45UL )

#define portSTART_ADDRESS_MASK                ( ( StackType_t ) 0xfffffffeUL )

static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;

#if ( configUSE_TICKLESS_IDLE == 1 )
    static uint32_t ulTimerCountsForOneTick = 0;
    static uint32_t xMaximumPossibleSuppressedTicks = 0;
    static uint32_t ulStoppedTimerCompensation = 0;
#endif

#if ( configASSERT_DEFINED == 1 )
    static uint8_t ucMaxSysCallPriority = 0;
    static uint32_t ulMaxPRIGROUPValue = 0;
    static const volatile uint8_t * const pcInterruptPriorityRegisters = ( const volatile uint8_t * const ) portNVIC_IP_REGISTERS_OFFSET_16;
#endif

void vPortSetupTimerInterrupt( void );
void xPortPendSVHandler( void ) __attribute__( ( naked ) );
void xPortSysTickHandler( void );
void vPortSVCHandler( void ) __attribute__( ( naked ) );
static void prvPortStartFirstTask( void ) __attribute__( ( naked ) );
static void vPortEnableVFP( void ) __attribute__( ( naked ) );
static void prvTaskExitError( void );

StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
    pxTopOfStack--;
    *pxTopOfStack = portINITIAL_XPSR;
    pxTopOfStack--;
    *pxTopOfStack = ( ( StackType_t ) pxCode ) & portSTART_ADDRESS_MASK;
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) portTASK_RETURN_ADDRESS;
    pxTopOfStack -= 5;
    *pxTopOfStack = ( StackType_t ) pvParameters;
    pxTopOfStack--;
    *pxTopOfStack = portINITIAL_EXEC_RETURN;
    pxTopOfStack -= 8;
    return pxTopOfStack;
}

static void prvTaskExitError( void )
{
    volatile uint32_t ulDummy = 0;

    configASSERT( uxCriticalNesting == ~0UL );
    portDISABLE_INTERRUPTS();

    while( ulDummy == 0 )
    {
        /* trap here */
    }
}

void vPortSVCHandler( void )
{
    __asm volatile
    (
        " ldr r3, =pxCurrentTCB            \n"
        " ldr r1, [r3]                     \n"
        " ldr r0, [r1]                     \n"
        " ldmia r0!, {r4-r11, r14}         \n"
        " msr psp, r0                      \n"
        " isb                              \n"
        " mov r0, #0                       \n"
        " msr basepri, r0                  \n"
        " bx r14                           \n"
        " .ltorg                           \n"
    );
}

static void prvPortStartFirstTask( void )
{
    __asm volatile
    (
        " ldr r0, =0xE000ED08              \n"
        " ldr r0, [r0]                     \n"
        " ldr r0, [r0]                     \n"
        " msr msp, r0                      \n"
        " mov r0, #0                       \n"
        " msr control, r0                  \n"
        " cpsie i                          \n"
        " cpsie f                          \n"
        " dsb                              \n"
        " isb                              \n"
        " svc 0                            \n"
        " nop                              \n"
        " .ltorg                           \n"
    );
}

BaseType_t xPortStartScheduler( void )
{
    configASSERT( portCPUID != portCORTEX_M7_r0p1_ID );
    configASSERT( portCPUID != portCORTEX_M7_r0p0_ID );

#if ( configCHECK_HANDLER_INSTALLATION == 1 )
    {
        const void * const *pxVectorTable = ( const void * const * ) ( ( volatile void * ) 0xE000ED08 );
        configASSERT( pxVectorTable[ 11 ] == ( void * ) vPortSVCHandler );
        configASSERT( pxVectorTable[ 14 ] == ( void * ) xPortPendSVHandler );
    }
#endif

#if ( configASSERT_DEFINED == 1 )
    {
        volatile uint8_t ucOriginalPriority;
        volatile uint32_t ulImplementedPrioBits = 0;
        volatile uint8_t * const pucFirstUserPriorityRegister = ( volatile uint8_t * const ) ( portNVIC_IP_REGISTERS_OFFSET_16 + portFIRST_USER_INTERRUPT_NUMBER );
        volatile uint8_t ucMaxPriorityValue;

        ucOriginalPriority = *pucFirstUserPriorityRegister;
        *pucFirstUserPriorityRegister = portMAX_8_BIT_VALUE;
        ucMaxPriorityValue = *pucFirstUserPriorityRegister;
        ucMaxSysCallPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY & ucMaxPriorityValue;
        configASSERT( ucMaxSysCallPriority );
        configASSERT( ( configMAX_SYSCALL_INTERRUPT_PRIORITY & ( ~ucMaxPriorityValue ) ) == 0U );

        while( ( ucMaxPriorityValue & portTOP_BIT_OF_BYTE ) == portTOP_BIT_OF_BYTE )
        {
            ulImplementedPrioBits++;
            ucMaxPriorityValue <<= 1U;
        }

        if( ulImplementedPrioBits == 8 )
        {
            configASSERT( ( configMAX_SYSCALL_INTERRUPT_PRIORITY & 0x1U ) == 0U );
            ulMaxPRIGROUPValue = 0;
        }
        else
        {
            ulMaxPRIGROUPValue = portMAX_PRIGROUP_BITS - ulImplementedPrioBits;
        }

        ulMaxPRIGROUPValue <<= portPRIGROUP_SHIFT;
        ulMaxPRIGROUPValue &= portPRIORITY_GROUP_MASK;
        *pucFirstUserPriorityRegister = ucOriginalPriority;
    }
#endif

    portNVIC_SHPR3_REG |= portNVIC_PENDSV_PRI;
    portNVIC_SHPR3_REG |= portNVIC_SYSTICK_PRI;
    portNVIC_SHPR2_REG = 0;

    vPortSetupTimerInterrupt();
    uxCriticalNesting = 0;
    vPortEnableVFP();
    *( portFPCCR ) |= portASPEN_AND_LSPEN_BITS;
    prvPortStartFirstTask();
    vTaskSwitchContext();
    prvTaskExitError();

    return 0;
}

void vPortEndScheduler( void )
{
    configASSERT( uxCriticalNesting == 1000UL );
}

void vPortEnterCritical( void )
{
    portDISABLE_INTERRUPTS();
    uxCriticalNesting++;

    if( uxCriticalNesting == 1 )
    {
        configASSERT( ( portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK ) == 0 );
    }
}

void vPortExitCritical( void )
{
    configASSERT( uxCriticalNesting );
    uxCriticalNesting--;

    if( uxCriticalNesting == 0 )
    {
        portENABLE_INTERRUPTS();
    }
}

void xPortPendSVHandler( void )
{
    __asm volatile
    (
        " mrs r0, psp                         \n"
        " isb                                 \n"
        " ldr r3, =pxCurrentTCB               \n"
        " ldr r2, [r3]                        \n"
        " tst r14, #0x10                      \n"
        " it eq                               \n"
        " vstmdbeq r0!, {s16-s31}             \n"
        " stmdb r0!, {r4-r11, r14}            \n"
        " str r0, [r2]                        \n"
        " stmdb sp!, {r0, r3}                 \n"
        " mov r0, %0                          \n"
        " msr basepri, r0                     \n"
        " dsb                                 \n"
        " isb                                 \n"
        " bl vTaskSwitchContext               \n"
        " mov r0, #0                          \n"
        " msr basepri, r0                     \n"
        " ldmia sp!, {r0, r3}                 \n"
        " ldr r1, [r3]                        \n"
        " ldr r0, [r1]                        \n"
        " ldmia r0!, {r4-r11, r14}            \n"
        " tst r14, #0x10                      \n"
        " it eq                               \n"
        " vldmiaeq r0!, {s16-s31}             \n"
        " msr psp, r0                         \n"
        " isb                                 \n"
#ifdef WORKAROUND_PMU_CM001
#if WORKAROUND_PMU_CM001 == 1
        " push { r14 }                        \n"
        " pop { pc }                          \n"
#endif
#endif
        " bx r14                              \n"
        " .ltorg                              \n"
        :: "i" ( configMAX_SYSCALL_INTERRUPT_PRIORITY )
    );
}

void xPortSysTickHandler( void )
{
    portDISABLE_INTERRUPTS();

    if( xTaskIncrementTick() != pdFALSE )
    {
        portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
    }

    portENABLE_INTERRUPTS();
}

#if ( configUSE_TICKLESS_IDLE == 1 )
    __attribute__( ( weak ) ) void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
    {
        /* This project does not currently use tickless idle. Keep the symbol
         * so the port stays compatible if the config is enabled later. */
        ( void ) xExpectedIdleTime;
        configASSERT( 0 );
    }
#endif

__attribute__( ( weak ) ) void vPortSetupTimerInterrupt( void )
{
#if ( configUSE_TICKLESS_IDLE == 1 )
    ulTimerCountsForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );
    xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick;
    ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / ( configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ );
#endif

    portNVIC_SYSTICK_CTRL_REG = 0UL;
    portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
    portNVIC_SYSTICK_LOAD_REG = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
    portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT );
}

static void vPortEnableVFP( void )
{
    __asm volatile
    (
        " ldr.w r0, =0xE000ED88            \n"
        " ldr r1, [r0]                     \n"
        " orr r1, r1, #( 0xf << 20 )       \n"
        " str r1, [r0]                     \n"
        " bx r14                           \n"
        " .ltorg                           \n"
    );
}

#if ( configASSERT_DEFINED == 1 )
    void vPortValidateInterruptPriority( void )
    {
        uint32_t ulCurrentInterrupt;
        uint8_t ucCurrentPriority;

        __asm volatile ( "mrs %0, ipsr" : "=r" ( ulCurrentInterrupt ) :: "memory" );

        if( ulCurrentInterrupt >= portFIRST_USER_INTERRUPT_NUMBER )
        {
            ucCurrentPriority = pcInterruptPriorityRegisters[ ulCurrentInterrupt ];
            configASSERT( ucCurrentPriority >= ucMaxSysCallPriority );
            configASSERT( ( portAIRCR_REG & portPRIORITY_GROUP_MASK ) <= ulMaxPRIGROUPValue );
        }
    }
#endif
