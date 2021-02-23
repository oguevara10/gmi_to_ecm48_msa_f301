/**
  ***************************************************************************************************
  * @file    App_PeriodicReplyCmd.h 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    1-Jul-2020
  * @brief   Header of c++ function/s for APP example0 (simple template test APP header)
  * @note    
  ***************************************************************************************************
  */
//^** Tips: APPs/Drivers adding process example step6  [refer to user manual ) 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_PERIODICREPLYCMD_H
#define __APP_PERIODICREPLYCMD_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32F0xx.h"
//#include "stm32f0_discovery.h"
//#include "core_cm0.h"
#include "main.h"
#include "ShareMemory.h"
#include "RingBuffer.h"
#include "SysCon.h"

#ifdef __cplusplus
extern "C" {
#endif
 
//static ShareMemory* App_PeriodicReplyCmdShMem;  

  
//******************* App_PeriodicReplyCmd Control (inside shared memory) *******************************************************************************************************************************  
/*
typedef struct 
{
  RingBuffer*   InternalPipe;
  uint8_t  ErrorCode;                                                           //Error code
}App_PeriodicReplyCmdControl;
*/
//******************* end of App_Template Control (inside shared memory) ******************************************************************************************************************************* 
  
  




#ifdef __cplusplus
}
#endif

#endif /* __APP_PERIODICREPLYCMD_H */

