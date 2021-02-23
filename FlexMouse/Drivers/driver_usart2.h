/**
  ********************************************************************************************************************************
  * @file    driver_usart2.h 
  * @author  Pamela Lee
  * @brief   Header of Driver function/s for serial protocol with Usart2 hardware
  * @details Protocol Usart2, after decode a whole valid frame from serial port2,
  *          trigger the system control to execute the relative APP in the int stage the Rx data is in usart2SeqMemRX_u32.
  *          To Transmitt data : put data into Usart1TxPipe, and call this function USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _DRV_USART2_H_
#define _DRV_USART2_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "typedef.h"

#include "sequential_memory.h"
#include "structured_memory.h"
#include "scheduler.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
  


/* User parameters -------------------------------------------------------------------------------------------------------------*/
#define BAUD_RATE_115200 115200
#define TX_RX_BUF_SIZE 70
  
#define Master 0x55
#define Slave  0xAA
#define UniHeaderlen 7;                                                //header size prepare incase Advanced frame 
static uint8_t frameID = 1;

#define RxSyncChr Master  //receive from Master  
#define TxSyncChr Slave   //this is a slave unit

/* Setup -----------------------------------------------------------------------------------------------------------------------*/
static Ring_Buf_Handle usart2InternalSeqMem_u32;
//static Ring_Buf_Handle usart2SeqMemRX_u32;
static Ring_Buf_Handle usart2SeqMemRXG1_2_u32;
static Ring_Buf_Handle usart2SeqMemRXG3_u32;
static Ring_Buf_Handle usart2SeqMemRXG4L_u32;
static Ring_Buf_Handle usart2SeqMemRXG4H_u32;
static Ring_Buf_Handle usart2SeqMemTX_u32;
static Ram_Buf_Handle usart2StructMem_u32;

/**
  ********************************************************************************************************************************
  * @brief   Uart2 Control (inside shared memory)
  * @details 
  * @param   seqMemRX_u32 
  * @param   seqMemTX_u32 
  * @param   errorCode_u8           Error code of this USART2 module.
  ********************************************************************************************************************************
  */
//******************* Uart2 Control (inside structured memory) *******************************************************************************************************************************  
typedef struct {
   // Ring_Buf_Handle seqMemRX_u32;
    Ring_Buf_Handle seqMemRXG1_2_u32;
    Ring_Buf_Handle seqMemRXG3_u32;
    Ring_Buf_Handle seqMemRXG4L_u32;
    Ring_Buf_Handle seqMemRXG4H_u32;
    Ring_Buf_Handle seqMemTX_u32;
    Ring_Buf_Handle seqMem_InternalPipe_u32;
    int16_t motorSpeed_s16;
    uint16_t motorStatus_u16;
    uint8_t errorCode_u8;
} Usart2_Control;
//******************* end of Uart2 Control (inside structured memory) ******************************************************************************************************************************* 
    
enum           //Universal Protocol state define
{ 
  protocolstart,                //1 of 3 Rx frame process
  headerValidate,               //2 of 3 Rx frame process
  frameCRC                     //3 of 3 Rx frame process
};

/* Function Declarations -------------------------------------------------------------------------------------------------------*/
/**
  ********************************************************************************************************************************
  * @brief   USART2 initialization function.
  * @details Configure peripheral clocks. Configure hardware TX/RX pins. Configure USART2 interrupt.
  *             PA2 -> USART2_TX, PA3 -> USART2_RX 
  ********************************************************************************************************************************
  */
void MX_USART2_UART_Init(void);

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details
  * @return  Returns
  ********************************************************************************************************************************
  */
uint8_t Usart2_IRQCallback(void);

void USART_CharReception_Callback(void);
void USART_TXEmpty_Callback(void);
void USART_CharTransmitComplete_Callback(void);
void protocolHeaderfetch(void);
uint8_t TxProcess(void);
void Error_Callback(void);
void assign_UART2_DrvMem(void);
void usart2_Init(void);
void USART2_IRQHandler(void);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DRV_USART2_H_ */