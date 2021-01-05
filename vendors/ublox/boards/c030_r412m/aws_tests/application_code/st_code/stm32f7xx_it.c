/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

extern void xPortSysTickHandler( void );

//extern TIM_HandleTypeDef TimHandle;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/*
void prvGetRegistersFromStack( uint32_t * pulFaultStackAddress )
{
    volatile uint32_t r0;
    volatile uint32_t r1;
    volatile uint32_t r2;
    volatile uint32_t r3;
    volatile uint32_t r12;
    volatile uint32_t lr;   // Link register.
    volatile uint32_t pc;   // Program counter.
    volatile uint32_t psr;  // Program status register.

    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];

    // Remove compiler warnings about the variables not being used.
    ( void ) r0;
    ( void ) r1;
    ( void ) r2;
    ( void ) r3;
    ( void ) r12;
    ( void ) lr;   // Link register.
    ( void ) pc;   // Program counter.
    ( void ) psr;  // Program status register.

    // When the following line is hit, the variables contain the register values.
    for( ; ; )
    {
    }
}
*/

/*
void HardFault_Handler( void )
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}
*/

/*-----------------------------------------------------------*/

/**
  * @brief This function handles Hard fault interrupt.
  */

void HardFault_Handler(void)
{
   //USER CODE BEGIN HardFault_IRQn 0

   //USER CODE END HardFault_IRQn 0
  while (1)
  {
     //USER CODE BEGIN W1_HardFault_IRQn 0
     //USER CODE END W1_HardFault_IRQn 0
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
/*void SVC_Handler(void)
{
	   USER CODE BEGIN HardFault_IRQn 0

	   USER CODE END HardFault_IRQn 0
	  while (1)
	  {
	     USER CODE BEGIN W1_HardFault_IRQn 0
	     USER CODE END W1_HardFault_IRQn 0
	  }
}*/

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
	  /* USER CODE BEGIN HardFault_IRQn 0 */

	  /* USER CODE END HardFault_IRQn 0 */
	  while (1)
	  {
	    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
	    /* USER CODE END W1_HardFault_IRQn 0 */
	  }
}

/**
  * @brief This function handles Pendable request for system service.
  */
/*void PendSV_Handler(void)
{
	   USER CODE BEGIN HardFault_IRQn 0

	   USER CODE END HardFault_IRQn 0
	  while (1)
	  {
	     USER CODE BEGIN W1_HardFault_IRQn 0
	     USER CODE END W1_HardFault_IRQn 0
	  }
}*/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_SYSTICK_IRQHandler();
}

/* When using the ST HAL, we must implement their systick callback. So, here,
 * we simply call the RTOS systick handler that resides in the core level abstraction.
 */

void HAL_SYSTICK_Callback( void )
{
	xPortSysTickHandler( );
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
/**
 * @brief This function handles TIM6 global interrupt, DAC1_CH1 and DAC1_CH2
 * underrun error interrupts.
 */
/*void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}*/

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
