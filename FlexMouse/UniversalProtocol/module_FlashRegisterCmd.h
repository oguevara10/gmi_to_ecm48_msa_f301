/**
  ***************************************************************************************************
  * @file    module_FlashRegisterCmd.h 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    12-Jan-2021
  * @brief   Header of c++ function/s for Universal protocol 
  * @note    
  ***************************************************************************************************
  */
//^** Tips: APPs/Drivers adding process example step6  [refer to user manual ) 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODULE_FLASHREGISTERCMD_H
#define __MODULE_FLASHREGISTERCMD_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"

#ifdef __cplusplus
extern "C" {
#endif
 
uint16_t RegisterRead(uint16_t _registerNum);
void RegisterSend(uint16_t _registerNumber, uint16_t _Data);
#ifdef __cplusplus
}
#endif

#endif /* __MODULE_FLASHREGISTERCMD_H */

