/**
  ******************************************************************************
  * @file    USART/Printf/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "systick/sys_tick.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_Printf
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
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
/* 用于调试时查看出错现场 */
volatile uint32_t g_hardfault_stack[8];

void HardFault_C(uint32_t *stack_frame)
{
    g_hardfault_stack[0] = stack_frame[0];  /* R0  */
    g_hardfault_stack[1] = stack_frame[1];  /* R1  */
    g_hardfault_stack[2] = stack_frame[2];  /* R2  */
    g_hardfault_stack[3] = stack_frame[3];  /* R3  */
    g_hardfault_stack[4] = stack_frame[4];  /* R12 */
    g_hardfault_stack[5] = stack_frame[5];  /* LR  */
    g_hardfault_stack[6] = stack_frame[6];  /* PC  */
    g_hardfault_stack[7] = stack_frame[7];  /* PSR */

    /* 若连接了调试器，可在此处中断查看变量 */
    __breakpoint(0);

    NVIC_SystemReset();
}

__asm void HardFault_Handler(void)
{
    IMPORT  HardFault_C
    TST     LR, #4              /* 檢查 EXC_RETURN[2] */
    ITE     EQ
    MRSEQ   R0, MSP
    MRSNE   R0, PSP
    LDR     R1, =HardFault_C
    BX      R1
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    NVIC_SystemReset();
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    NVIC_SystemReset();
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    NVIC_SystemReset();
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
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
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    sys_tick_inc();
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
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

#ifdef USE_FULL_ASSERT
/**
  * @brief  参数检查失败时调用，随后系统复位
  * @param  file: 源文件名
  * @param  line: 出错行号
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    __breakpoint(0);
    NVIC_SystemReset();
}
#endif /* USE_FULL_ASSERT */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
