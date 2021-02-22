/**
  ********************************************************************************************************************************
  * @file    module_app.c 
  * @author  Pamela Lee
  * @brief   This is a template non-driver app.
  * @details This app does nothing.
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "driver_usart2.h"


/* Content ---------------------------------------------------------------------------------------------------------------------*/
extern ProcessInfo processInfoTable[];

#define DemandPollPeriod 2000                                                       //time period for checking and sending 0-10V and speed data to motor board
uint64_t tt_DemandTime;

Usart2_Control* usart2Control_AppLocal;
uint8_t testCounter = 0;
uint64_t errorCounter = 0;



uint8_t tmpry4Test = true; /** this is only for testing the error/log data exchange !!!!!!!!!!!!! please delete these line for production version **/
#define tmpDelayPeriod 100                                                       //time period for checking and sending 0-10V and speed data to motor board
uint64_t tt_DemandtmpDelayTime;


enum AppStates {
    INIT_APP,
    RUN_APP,
    // additional states to be added here as necessary.
    IRQ_APP = DEFAULT_IRQ_STATE,
    STOP_APP = KILL_APP
};

/** pam procedure #10 of Module insertion  :  add the module execution function **/
uint8_t p_moduleApp_u32(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8,
                        uint8_t irq_id_u8) {
    uint8_t return_state_u8 = 0;
    switch (next_State_u8) {
        case INIT_APP: {
            uint8_t Usart2index  = getProcessInfoIndex(MODULE_USART2); //return Process index from processInfo array
            usart2Control_AppLocal = (Usart2_Control*)((*(processInfoTable[Usart2index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);    //Get structured memory for USART2
            tt_DemandTime = getSysCount() + DemandPollPeriod;                          //store time tick value  
            return_state_u8 = RUN_APP;
            break;
        }
        case RUN_APP: {
            if (getSysCount() >= tt_DemandTime) {
                unsigned char speedTx[] = {0x55, 0x01, 0x00, 0xFF, 0x00, (unsigned char)testCounter, (unsigned char)module_id_u8, 0xCC};
                unsigned int speedLen = sizeof(speedTx);
        //        RingBuf_WriteBlock((*usart2Control_AppLocal).seqMemTX_u32, speedTx, &speedLen);             
                tt_DemandTime = getSysCount() + DemandPollPeriod;                          //update next time tick value 
            }      
            
            
            /** this is only for testing the error/log data exchange !!!!!!!!!!!!! please delete these line for production version **/
            tt_DemandtmpDelayTime = getSysCount() + tmpDelayPeriod + errorCounter;                          //update next time tick value 
            if(errorCounter >= 4){ // && (tmpry4Test == true)) {                            //test the error reporting by software interrupt
              setupSoftwareIRQ(module_id_u8, MODULE_ERR_LOGHANDLE, 0xEF, 0x01, 0x00, NULL);
              tmpry4Test = false;
              while(getSysCount() < tt_DemandtmpDelayTime){}
              
            }
           /** this is only for testing the error/log data exchange !!!!!!!!!!!!! please delete these line for production version end **/          
            return_state_u8 = RUN_APP;
            break;
        }
        case IRQ_APP: {
            errorCounter++;
            return_state_u8 = RUN_APP;
            break;
        }
        case STOP_APP: {
            return_state_u8 = INIT_APP;
            break;
        }
        default: {
            return_state_u8 = STOP_APP;
            break;
        }
    }
    return return_state_u8;
}
/** pam procedure #10 of Module insertion  :  add the module execution function end **/