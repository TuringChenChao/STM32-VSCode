/**
  ******************************************************************************
  * @file    UART/UART_TwoBoards_ComPolling/Src/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
   
/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_TwoBoards_ComPolling
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
enum { r0, r1, r2, r3, r12, lr, pc, psr,
       s0, s1, s2, s3, s4, s5, s6, s7,
       S8, s9, s10, s11, s12, s13, s14, s15,
       fpscr
     };

typedef struct TaskContextType {
    unsigned int r0;
    unsigned int r1;
    unsigned int r2;
    unsigned int r3;
    unsigned int r4;
    unsigned int r5;
    unsigned int r6;
    unsigned int r7;
    unsigned int r8;
    unsigned int r9;
    unsigned int r10;
    unsigned int r11;
    unsigned int r12;
    unsigned int sp;              /* after pop r0-r3, lr, pc, xpsr                   */
    unsigned int lr;              /* lr before exception                             */
    unsigned int pc;              /* pc before exception                             */
    unsigned int psr;             /* xpsr before exeption                            */
    unsigned int control;         /* nPRIV bit & FPCA bit meaningful, SPSEL bit = 0  */
    unsigned int exc_return;      /* current lr                                      */
    unsigned int msp;             /* msp                                             */
    unsigned int psp;             /* psp                                             */
    unsigned int fpscr;
    unsigned int s0;
    unsigned int s1;
    unsigned int s2;
    unsigned int s3;
    unsigned int s4;
    unsigned int s5;
    unsigned int s6;
    unsigned int s7;
    unsigned int s8;
    unsigned int s9;
    unsigned int s10;
    unsigned int s11;
    unsigned int s12;
    unsigned int s13;
    unsigned int s14;
    unsigned int s15;
    unsigned int s16;
    unsigned int s17;
    unsigned int s18;
    unsigned int s19;
    unsigned int s20;
    unsigned int s21;
    unsigned int s22;
    unsigned int s23;
    unsigned int s24;
    unsigned int s25;
    unsigned int s26;
    unsigned int s27;
    unsigned int s28;
    unsigned int s29;
    unsigned int s30;
    unsigned int s31;
} TaskContext;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static TaskContext taskContext = {0};
TaskContext *pTaskContext = &taskContext;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void exception_int_printf(uint32_t data)
{
	uint32_t temp = 0;
	uint32_t i = 0;

    ITM_SendChar('0');
    ITM_SendChar('x');
    for(i=0; i < 8; i++)
    {
        temp = (data >> ((7 - i)*4)) & 0x0000000f;
        if(temp > 10)
        {
                temp = temp - 10 + 'a';
        }
        else
        {
                temp = temp + '0';
        }
        ITM_SendChar(temp);
    }
	ITM_SendChar('\n');
}

void stackDump(uint32_t stack[])
{
    taskContext.r0   = stack[r0];
    taskContext.r1   = stack[r1];
    taskContext.r2   = stack[r2];
    taskContext.r3   = stack[r3];
    taskContext.r12  = stack[r12];
    taskContext.sp   = ((uint32_t)stack) + 0x20;
    taskContext.lr   = stack[lr];
    taskContext.pc   = stack[pc];
    taskContext.psr  = stack[psr];

	exception_int_printf(taskContext.pc);

#if 0
    /* FPU context? */
    if ( (taskContext.exc_return & 0x10) == 0 ) {
        taskContext.s0 = stack[s0];
        taskContext.s1 = stack[s1];
        taskContext.s2 = stack[s2];
        taskContext.s3 = stack[s3];
        taskContext.s4 = stack[s4];
        taskContext.s5 = stack[s5];
        taskContext.s6 = stack[s6];
        taskContext.s7 = stack[s7];
        taskContext.s8 = stack[s8];
        taskContext.s9 = stack[s9];
        taskContext.s10 = stack[s10];
        taskContext.s11 = stack[s11];
        taskContext.s12 = stack[s12];
        taskContext.s13 = stack[s13];
        taskContext.s14 = stack[s14];
        taskContext.s15 = stack[s15];
        taskContext.fpscr = stack[fpscr];
        taskContext.sp += 72; /* s0-s15, fpsr, reserved */
    }

    /* if CCR.STKALIGN=1, check PSR[9] to know if there is forced stack alignment */
    if ( (SCB->CCR & SCB_CCR_STKALIGN_Msk) && (taskContext.psr & 0x200)) {
        taskContext.sp += 4;
    }

    /* update CONTROL.SPSEL and psp if returning to thread mode */
    if (taskContext.exc_return & 0x4) {
        taskContext.control |= 0x2; /* CONTROL.SPSel */
        taskContext.psp = taskContext.sp;
    } else { /* update msp if returning to handler mode */
        taskContext.msp = taskContext.sp;
    }


    printf("r0  = 0x%08x\n\r", taskContext.r0);
    printf("r1  = 0x%08x\n\r", taskContext.r1);
    printf("r2  = 0x%08x\n\r", taskContext.r2);
    printf("r3  = 0x%08x\n\r", taskContext.r3);
    printf("r4  = 0x%08x\n\r", taskContext.r4);
    printf("r5  = 0x%08x\n\r", taskContext.r5);
    printf("r6  = 0x%08x\n\r", taskContext.r6);
    printf("r7  = 0x%08x\n\r", taskContext.r7);
    printf("r8  = 0x%08x\n\r", taskContext.r8);
    printf("r9  = 0x%08x\n\r", taskContext.r9);
    printf("r10 = 0x%08x\n\r", taskContext.r10);
    printf("r11 = 0x%08x\n\r", taskContext.r11);
    printf("r12 = 0x%08x\n\r", taskContext.r12);
    printf("lr  = 0x%08x\n\r", taskContext.lr);
    printf("pc  = 0x%08x\n\r", taskContext.pc);
    printf("psr = 0x%08x\n\r", taskContext.psr);
    printf("EXC_RET = 0x%08x\n\r", taskContext.exc_return);

    /* FPU context? */
    if ( (taskContext.exc_return & 0x10) == 0 ) {
        taskContext.control |= 0x4; /* CONTROL.FPCA */
        printf("s0  = 0x%08x\n\r", taskContext.s0);
        printf("s1  = 0x%08x\n\r", taskContext.s1);
        printf("s2  = 0x%08x\n\r", taskContext.s2);
        printf("s3  = 0x%08x\n\r", taskContext.s3);
        printf("s4  = 0x%08x\n\r", taskContext.s4);
        printf("s5  = 0x%08x\n\r", taskContext.s5);
        printf("s6  = 0x%08x\n\r", taskContext.s6);
        printf("s7  = 0x%08x\n\r", taskContext.s7);
        printf("s8  = 0x%08x\n\r", taskContext.s8);
        printf("s9  = 0x%08x\n\r", taskContext.s9);
        printf("s10 = 0x%08x\n\r", taskContext.s10);
        printf("s11 = 0x%08x\n\r", taskContext.s11);
        printf("s12 = 0x%08x\n\r", taskContext.s12);
        printf("s13 = 0x%08x\n\r", taskContext.s13);
        printf("s14 = 0x%08x\n\r", taskContext.s14);
        printf("s15 = 0x%08x\n\r", taskContext.s15);
        printf("s16 = 0x%08x\n\r", taskContext.s16);
        printf("s17 = 0x%08x\n\r", taskContext.s17);
        printf("s18 = 0x%08x\n\r", taskContext.s18);
        printf("s19 = 0x%08x\n\r", taskContext.s19);
        printf("s20 = 0x%08x\n\r", taskContext.s20);
        printf("s21 = 0x%08x\n\r", taskContext.s21);
        printf("s22 = 0x%08x\n\r", taskContext.s22);
        printf("s23 = 0x%08x\n\r", taskContext.s23);
        printf("s24 = 0x%08x\n\r", taskContext.s24);
        printf("s25 = 0x%08x\n\r", taskContext.s25);
        printf("s26 = 0x%08x\n\r", taskContext.s26);
        printf("s27 = 0x%08x\n\r", taskContext.s27);
        printf("s28 = 0x%08x\n\r", taskContext.s28);
        printf("s29 = 0x%08x\n\r", taskContext.s29);
        printf("s30 = 0x%08x\n\r", taskContext.s30);
        printf("s31 = 0x%08x\n\r", taskContext.s31);
        printf("fpscr = 0x%08x\n\r", taskContext.fpscr);
    }

    printf("CONTROL = 0x%08x\n\r", taskContext.control);
    printf("MSP     = 0x%08x\n\r", taskContext.msp);
    printf("PSP     = 0x%08x\n\r", taskContext.psp);
    printf("sp      = 0x%08x\n\r", taskContext.sp);
#endif
}

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
#if 1
void CommonFault_Handler(void)
{
    __asm volatile
    (
        "cpsid i                       \n"     /* disable irq                 */
        // "ldr r3, =pxExceptionStack     \n"
        // "ldr r3, [r3]                  \n"     /* r3 := pxExceptionStack      */
        "ldr r0, =pTaskContext         \n"
        "ldr r0, [r0]                  \n"     /* r0 := pTaskContext          */
        "add r0, r0, #16               \n"     /* point to context.r4         */
        "stmia r0!, {r4-r11}           \n"     /* store r4-r11                */
        "mov r5, r12                   \n"     /* r5 := EXC_RETURN            */
        "add r0, r0, #20               \n"     /* point to context.control    */
        "mrs r1, control               \n"     /* move CONTROL to r1          */
        "str r1, [r0], #4              \n"     /* store CONTROL               */
        "str r5, [r0], #4              \n"     /* store EXC_RETURN            */
        "mrs r4, msp                   \n"     /* r4 := MSP                   */
        "str r4, [r0], #4              \n"     /* store MSP                   */
        "mrs r1, psp                   \n"     /* move PSP to r1              */
        "str r1, [r0]                  \n"     /* store PSP                   */
        "tst r5, #0x10                 \n"     /* FPU context?                */
        "itt eq                        \n"
        "addeq r0, r0, #68             \n"     /* point to contex.s16         */
        "vstmeq r0, {s16-s31}          \n"     /* store r16-r31               */
        "cmp r3, #0                    \n"     /* if (!pxExceptionStack)      */
        "it ne                         \n"
        "movne sp, r3                  \n"     /* update msp                  */
        "push {lr}                     \n"
        // "bl exception_init             \n"
        "pop {lr}                      \n"
        "tst r5, #4                    \n"     /* thread or handler mode?     */
        "ite eq                        \n"
        "moveq r0, r4                  \n"
        "mrsne r0, psp                 \n"
        "bx lr                         \n"
    );
}

void Hard_Fault_Handler(uint32_t stack[])
{
	/* Go to infinite loop when Hard Fault exception occurs */
	uint32_t fault = 0;
	// uint32_t i = 0;
	// uint32_t temp = 0; 
	ITM_SendChar('H');
	if ((SCB->HFSR & (1 << 30)) != 0) {
		ITM_SendChar('F');
		// printf("SCB->CFSR = 0x%08x\n\r", (unsigned int)SCB->CFSR );
		if ((SCB->CFSR & 0xFFFF0000) != 0) {
            ITM_SendChar('U');
        }
        if ((SCB->CFSR & 0x0000FF00) != 0 ) {
            ITM_SendChar('B');
        }
        if ((SCB->CFSR & 0x000000FF) != 0 ) {
            ITM_SendChar('M');
        }
	}
    // ITM_SendChar('0');
    // ITM_SendChar('x');
    fault = SCB->CFSR;
	exception_int_printf(fault);
    // for(i=0; i < 8; i++)
    // {
    //     temp = (fault >> ((7 - i)*4)) & 0x0000000f;
    //     if(temp > 10)
    //     {
    //             temp = temp - 10 + 'a';
    //     }
    //     else
    //     {
    //             temp = temp + '0';
    //     }
    //     ITM_SendChar(temp);
    // }

	stackDump(stack);

    while (1)
    {
    }
}

/**
	* @brief  This function handles Hard Fault exception.
	* @param  None
	* @retval None
	*/
void HardFault_Handler(void)
{
    __asm volatile
    (
        "mov r12, lr                   \n"
        "bl CommonFault_Handler        \n"
        "bl Hard_Fault_Handler         \n"
    );
}
#else
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
#endif

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void SysTick_Handler(void)
{
  extern void HAL_IncTick(void);
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
