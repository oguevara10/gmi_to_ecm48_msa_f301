/**
********************************************************************************************************************************
* @file    module_usart1.c 
* @author  Pamela Lee
* @brief   Main driver module for USART1 Communication.
* @details This module initializes the USART1 port and attaches the pre-selected fixed memory allocation to the module.
To Transmitt data in the RUN_MODULE case: put data into seqMemTX_u32, and call this function:
*             USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
********************************************************************************************************************************
*/

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "module_usart1.h"

#include "driver_usart1.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
/* Uarts handle declaration */
extern void Delay(__IO uint32_t nTime);

extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];
extern Ram_Buf *usart1StructMem_u32;

//usart1_Control* usart1_Module_Control;
unsigned char* RxCMD ;
extern uint8_t usart1CaptureLen;
extern uint8_t UniProtocolState;
extern __IO uint8_t indexTx;

extern Usart1_Control *usart1Control;

enum {
  INIT_MODULE,
  RUN_MODULE,
  // additional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};


void assign_UART2_ModuleMem(){  
  usart1InternalSeqMem_u32 = SeqMem_CreateInstance(MODULE_USART1, TX_RX_BUF_SIZE, 
                              ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for this driver need to be bigger than 1 complete frame 
  usart1SeqMemRXG1_2_u32 = SeqMem_CreateInstance(MODULE_USART1, TX_RX_BUF_SIZE, 
                              ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for final packet receiver buffer 
  usart1SeqMemRXG3_u32 = SeqMem_CreateInstance(MODULE_USART1, TX_RX_BUF_SIZE, 
                              ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for final packet receiver buffer 
  usart1SeqMemRXG4L_u32 = SeqMem_CreateInstance(MODULE_USART1, TX_RX_BUF_SIZE +80, 
                              ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for final packet receiver buffer 
  usart1SeqMemRXG4H_u32 = SeqMem_CreateInstance(MODULE_USART1, TX_RX_BUF_SIZE +80, 
                              ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for final packet receiver buffer 
  usart1SeqMemTX_u32 = SeqMem_CreateInstance(MODULE_USART1, TX_RX_BUF_SIZE +7 , 
                              ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for Tx data 
  usart1StructMem_u32 =  StructMem_CreateInstance(MODULE_USART1, sizeof(Usart1_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a structured memory for this driver [should map it back to this driver local struct]
  
  usart1Control = (Usart1_Control*)(*usart1StructMem_u32).p_ramBuf_u8;
  
  /** assign all the new generated sequential-memory of USART1 to the structured-memory **/
  usart1Control->seqMemTX_u32 = usart1SeqMemTX_u32;
  usart1Control->seqMemTX_u32->is_OverwrittingAllowed_u8 = FALSE;
  usart1Control->seqMemRXG1_2_u32 = usart1SeqMemRXG1_2_u32;
  usart1Control->seqMemRXG3_u32 = usart1SeqMemRXG3_u32;
  usart1Control->seqMemRXG4L_u32 = usart1SeqMemRXG4L_u32;
  usart1Control->seqMemRXG4H_u32 = usart1SeqMemRXG4H_u32;
  usart1Control->seqMem_InternalPipe_u32 = usart1InternalSeqMem_u32;
  usart1Control->errorCode_u8 = 0;
}

uint8_t moduleUsart1_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8) {
  uint8_t return_state_u8 = INIT_MODULE;
  switch (next_state_u8) {
  case INIT_MODULE: {
    // Initialize UART
    
    usart1_Init();
    assign_UART2_ModuleMem();
    
    // Find the structured memory for the UART2 driver module, by searching for the UART2 onwer id.
    Ram_Buf_Handle this_ram_buf_u32;
    for (uint8_t struct_mem_index_u8 = 0; struct_mem_index_u8 < TOTAL_NUM_OF_STRUCT_MEM_INSTANCES;
    struct_mem_index_u8++) {
      this_ram_buf_u32 = &sharedMemArray[struct_mem_index_u8];
      if (RamBuf_GetOwner(this_ram_buf_u32) == drv_id_u8) {
        usart1StructMem_u32 = &sharedMemArray[struct_mem_index_u8];
      }
    }
    
    // Attach the structured memory to the process's master shared memory.
    uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
    if (table_index_u8 != INDEX_NOT_FOUND) {
      processInfoTable[table_index_u8].Sched_DrvData.irqState_u8 = DEFAULT_IRQ_STATE;
      processInfoTable[table_index_u8].Sched_DrvData.p_masterSharedMem_u32 =
        usart1StructMem_u32;
    }
    
    //Get structured memory for ADC1 data
    usart1Control = (Usart1_Control*)((*(processInfoTable[table_index_u8].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);
    usart1CaptureLen = UniHeaderlen;                                 //pam bug without this
    return_state_u8 = RUN_MODULE;
    break;
  }
  case RUN_MODULE: {
    // test Uart2 transmits data
    // if (USART_GetITStatus(USART1, USART_IT_TXE) == RESET) {
    //     (*(*usart1Control).seqMemTX_u32).Write(Txbuf, &Len);
    //     USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //  Enable when something to send
    // }
    
 
    //(*usart1Control).seqMem_InternalPipe_u32->systemInstanceIndex_u8)
    //if(RingBuf_GetUsedNumOfElements((Ring_Buf_Handle)((*(*usart1Control).seqMem_InternalPipe_u32).p_ringBuf_u8)) >= usart1CaptureLen )
    if(RingBuf_GetUsedNumOfElements((*usart1Control).seqMem_InternalPipe_u32) >= usart1CaptureLen )
    {
      protocolHeaderfetch();
    }
    uint8_t TxLen = UniHeaderlen;
    //if((RingBuf_GetUsedNumOfElements((Ring_Buf_Handle)((*(*usart1Control).seqMemTX_u32).p_ringBuf_u8)) >= TxLen) && !indexTx)
    if(((RingBuf_GetUsedNumOfElements((*usart1Control).seqMemTX_u32) >= TxLen) && !indexTx) && (LL_USART_IsActiveFlag_TXE(USART1)))
    {
      TxProcess();
    }  
    return_state_u8 = RUN_MODULE;
    break;
  }
  case KILL_MODULE: {
    // The USART1 driver module must only be executed once.
    // Setting processStatus_u8 to PROCESS_STATUS_KILLED prevents the scheduler main loop from calling this module again.
    uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
    if (table_index_u8 != INDEX_NOT_FOUND) {
      processInfoTable[table_index_u8].Sched_DrvData.processStatus_u8 = PROCESS_STATUS_KILLED;
    }
    return_state_u8 = KILL_MODULE;
    break;
  }
  default: {
    return_state_u8 = KILL_MODULE;
    break;
  }
  }
  return return_state_u8;
}