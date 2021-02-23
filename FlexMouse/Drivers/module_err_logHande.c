/**
  ********************************************************************************************************************************
  * @file    module_err_logHandle.c 
  * @author  Pamela Lee
  * @version V1.0
  * @date    28-OCT-2020
  * @brief   Kernal module for error or log handling.
  * @details All module/s can report their Error or log by either software interrupt(in urgent message) or by structured_memory(for normal data log), 
  *          this module will response according to the type of the message, which may either store it in the EEprom as error/data log or just 
  *          increase as the relatived counter/s.
  *     parameters:-
  *          ErrorModule ID(SENDER_MODULE_ID):  Module ID for error occur                    
  *          ErrorMesage Type(irqType_u8):      0xE0 = AutoAck-TimeOut, 
  *                                             0xE1 = Heap-Memory allocation Error, 
  *                                             0xE2 = Module execution exceed time limit error,
  *                                             0xE3 = IRQ response execution exceed time limit error,
  *                                             0xE4 = Flash Error 
  *                                             0xEF = for debug tetsing 
  *          Further Detials(irqDatXX):         All the details will send through the various irqDatxx as data, may need to decode in different way (see the particular routine for details)
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "module_err_logHandle.h"

//#include "driver_usart1.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
/* Auto acknowledgement handle declaration */
extern ProcessInfo processInfoTable[];

Err_logHandling_Control* err_logHandling_Control;

#define Err_logTimePeriod 5000                        //Err_log Handling waiting period
uint64_t Err_log_WaitTime; 
uint64_t testDat = 0;                           

enum {
  INIT_MODULE,
  RUN_MODULE,
  // additional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

uint8_t module_err_log_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8) {
  uint8_t return_state_u8 = INIT_MODULE;
  switch (next_state_u8) {
    case INIT_MODULE: {
      Err_logHandlingStructMem_u32 = StructMem_CreateInstance(MODULE_ERR_LOGHANDLE, sizeof(Err_logHandling_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a structured memory
      err_logHandling_Control = (Err_logHandling_Control*)(*Err_logHandlingStructMem_u32).p_ramBuf_u8;
//      (*err_logHandling_Control)
      Err_log_WaitTime = getSysCount() + Err_logTimePeriod;                        //store time tick value  
      return_state_u8 = RUN_MODULE;
      break;
    }
    case RUN_MODULE: {                                                  //        
      if (getSysCount() >= Err_log_WaitTime) 
      {
       
        Err_log_WaitTime = getSysCount() + Err_logTimePeriod;                        //store time tick value  
      }
      return_state_u8 = RUN_MODULE;
      break;
    }
    case IRQ_MODULE: {
        testDat++;
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


