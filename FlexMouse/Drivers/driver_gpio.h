/**
  ********************************************************************************************************************************
  * @file    driver_gpio.h 
  * @author  Pamela Lee
  * @brief   Header of Driver function/s for serial protocol with Usart1 hardware
  * @details Protocol Usart1, after decode a whole valid frame from serial port1,
  *          trigger the system control to execute the relative APP in the int stage the Rx data is in Usart1RxPipe.
  *          To Transmitt data : put data into Usart1TxPipe, and call this function USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _DRV_GPIO_H_
#define _DRV_GPIO_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include <stdint.h>

#include "typedef.h"

#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* User parameters -------------------------------------------------------------------------------------------------------------*/
#define VALUE_INDICATING_TRIGGERED_IRQ 0x55

#define SIZE_OF_GPIO_SEQ_MEM 32
#define ACCESS_MODE

/* Setup -----------------------------------------------------------------------------------------------------------------------*/
static Ring_Buf_Handle gpioSeqMem_u32;
static Ram_Buf_Handle gpioStructMem_u32;

/**
  ********************************************************************************************************************************
  * @brief   Uart1 Control (inside shared memory)
  * @details 
  * @param   gpioControlSeqMem_u32 
  * @param   gpioPortC_u8                     TODO: Not sure what this struct variable is for.
  * @param   errorCode_u8                     Error code of this GPIO module.
  ********************************************************************************************************************************
  */
typedef struct {
    Ring_Buf_Handle gpioControlSeqMem_u32;
    uint8_t gpioPortC_u8;
    uint8_t errorCode_u8;
} GPIO_Control;

/* Function Declarations -------------------------------------------------------------------------------------------------------*/
/**
  ********************************************************************************************************************************
  * @brief    
  * @details 
  ********************************************************************************************************************************
  */
void GPIOInit(void);

/**
  ********************************************************************************************************************************
  * @brief    This function handles EXTI line[9:5] interrupts.
  * @details 
  ********************************************************************************************************************************
  */
void EXTI9_5_IRQHandler(void);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DRV_GPIO_H_ */