/**
  ********************************************************************************************************************************
  * @file    module_AutoAck.h 
  * @author  Pamela Lee
  * @version V1.0
  * @date    21-OCT-2020
  * @brief   Main driver module for Auto Acknowledgement of Universal procotol.
  * @details 
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _MODULE_AUTOACK_H_
#define _MODULE_AUTOACK_H_

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
  
static Ram_Buf_Handle AutoAckStructMem_u32;  
  

typedef struct {
  uint8_t RxAckFrameID;
} AutoAck_Control;

void UniversalAckInit(void);
uint8_t AckDeRegistered(uint8_t FrameAckID);                     //Got the ack from the receiver and de-registered the record 
uint8_t AckDatSet(uint8_t _AckCmd, uint16_t _E_AckCmd, uint8_t _ProcessAckID);
uint8_t IsAckBufFull(void);
void Set_ValidHeader(void);
void AckFlushBuf(void);

void Set_ValidRx(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MODULE_AUTOACK_H_ */