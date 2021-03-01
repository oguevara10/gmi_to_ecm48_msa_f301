/**
  ********************************************************************************************************************************
  * @file    module_ecm_icl.h 
  * @author  Myron Mychal
  * @brief   This is a template non-driver app.
  * @details This app does nothing.
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _MODULE_ECM_ICL_H_
#define _MODULE_ECM_ICL_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "main.h"
#include "typedef.h"

#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"
#include "scheduler.h"
#include "ring_buffer.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Setup -----------------------------------------------------------------------------------------------------------------------*/
static Ring_Buf_Handle appSeqMem0_u32;
static Ring_Buf_Handle appSeqMem1_u32;
static Ram_Buf_Handle appStructMem_u32;

/**
  ********************************************************************************************************************************
  * @brief   Module App control (inside shared memory)
  * @details 
  * @param   internalPipe_u32 
  * @param   errorCode_u8       Error code of this app module.
  ********************************************************************************************************************************
  */
//typedef struct {
 //   Ring_Buf_Handle internalPipe_u32;
 //   uint8_t errorCode_u8;
//} App_TemplateControl;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MODULE_APP_H_ */