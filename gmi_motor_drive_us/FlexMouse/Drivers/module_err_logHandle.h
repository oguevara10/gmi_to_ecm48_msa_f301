/**
  ********************************************************************************************************************************
  * @file    module_err_logHandle.h 
  * @author  Pamela Lee
  * @version V1.0
  * @date    28-OCT-2020
  * @brief   Kernal module for error or log handling.
  * @details 
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _MODULE_ERR_LOGHANDLE_H_
#define _MODULE_ERR_LOGHANDLE_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "main.h"
#include "typedef.h"

#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
  
static Ram_Buf_Handle Err_logHandlingStructMem_u32;  
  

typedef struct {
  uint8_t* ErrorDat;
  uint8_t ErrorDatLen;
  uint8_t* LogDat;
  uint8_t LogDatLen;
} Err_logHandling_Control;


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MODULE_ERR_LOGHANDLE_H_ */