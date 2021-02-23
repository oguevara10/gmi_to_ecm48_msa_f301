/**
  ***************************************************************************************************
  * @file    module_FlashUpdateCmd.h 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    10-DEC-2020
  * @brief   Header of c++ function/s for Universal protocol 
  * @note    
  ***************************************************************************************************
  */
//^** Tips: APPs/Drivers adding process example step6  [refer to user manual ) 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODULE_FLASHUPDATECMD_H
#define __MODULE_FLASHUPDATECMD_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"

#ifdef __cplusplus
extern "C" {
#endif
 
uint8_t flashBlkWriteStateMachine_Run(uint8_t _module_id_u8);
uint8_t flashTxBlk(uint16_t _flashBlkNum, unsigned char* _buf);
#ifdef __cplusplus
}
#endif

#endif /* __MODULE_FLASHUPDATECMD_H */

