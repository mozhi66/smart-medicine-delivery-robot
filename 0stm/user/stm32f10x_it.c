/**
  ******************************************************************************
  * @file    stm32f10x_it.c
  * @brief   Main Interrupt Service Routines.
  *          智能药箱（Keil SPL 移植版）中断服务函数
  *
  * 本文件由 Keil 标准模板 stm32f10x_it.c 基础上补建：
  *   - SysTick_Handler   : 1ms 时基（my_lib/delay.c 的 ulTicks，uint32_t）
  *   - USART1_IRQHandler : PC 命令通道（USE_RX_DMA=0，RXNE 逐字节入 FIFO）
  *   - USART3_IRQHandler : HC-05 手机命令通道（RXNE 逐字节入 FIFO）
  *   - TIM3_IRQHandler   : 1ms 舵机插值节拍（SERVO_Update1ms）
  *
  * 注意：USE_RX_DMA=0，DMA1_Channel5 相关代码均未编译，
  *       故无 DMA1_Channel5_IRQHandler。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

/* 外部接口声明（避免在 it.c 中包含 app 头文件引起的宏污染） -------------------*/
extern __IO uint32_t ulTicks;          /* my_lib/delay.c 定义的 1ms 计数 */
extern void UART_RxBytePush(uint8_t b);    /* user/main.c：PC 命令 FIFO 入口 */
extern void HC05_RxBytePush(uint8_t b);    /* user/main.c：HC05 FIFO 入口 */
extern void SERVO_Update1ms(void);         /* app/servo.c：舵机插值 1ms 推进 */

/** @addtogroup STM32F10x_StdPeriph_Template
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
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

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
#ifndef RTE_CMSIS_RTOS_RTX
void SVC_Handler(void)
{
}
#endif

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
#ifndef RTE_CMSIS_RTOS_RTX
void PendSV_Handler(void)
{
}
#endif

/**
  * @brief  This function handles SysTick Handler.
  * @note   my_lib/delay.c 的 GetTick()/Delay() 依赖此 1ms 计数。
  *         ulTicks 在 delay.c 中定义为 __IO uint32_t，此处 extern 必须同型。
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    ulTicks++;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/******************************************************************************/

/**
  * @brief  USART1 中断：PC 命令通道（$MV/$SS/... 帧协议）
  * @note   USE_RX_DMA=0：RXNE 逐字节压入软件 FIFO，由主循环解析。
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t b = (uint8_t)USART_ReceiveData(USART1);
        UART_RxBytePush(b);
    }
}

/**
  * @brief  USART3 中断：HC-05 手机命令通道（单字符 F/B/L/R/G/H/S...）
  * @note   优先级 1（低于 USART1/TIM3 的 0），手机命令让位给运动节拍。
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t b = (uint8_t)USART_ReceiveData(USART3);
        HC05_RxBytePush(b);
    }
}

/**
  * @brief  TIM3 更新中断：1ms 一次，推进舵机插值
  * @note   TIM3 被机械臂占为 1ms 插值基准（PSC=71, ARR=999）。
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        SERVO_Update1ms();
    }
}

/**
  * @}
  */
