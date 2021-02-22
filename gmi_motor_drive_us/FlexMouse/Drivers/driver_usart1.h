/**
********************************************************************************************************************************
* @file    driver_usart1.h 
* @author  Satya Akkina
* @brief   Header of Driver function/s for serial protocol with Usart1 hardware
* @details 
********************************************************************************************************************************
*/



/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _DRV_USART1_H_
#define _DRV_USART1_H_

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
  
  /* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "typedef.h"
  
#include "sequential_memory.h"
#include "structured_memory.h"
#include "scheduler.h"
  
void USART1_CharTransmitComplete_Callback(void);
void USART1_CharReception_Callback(void);
  
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DRV_USART1_H_ */