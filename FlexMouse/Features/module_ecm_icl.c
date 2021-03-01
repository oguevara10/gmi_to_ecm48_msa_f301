/**
  ********************************************************************************************************************************
  * @file    module_ecm_icl.c 
  * @author  Myron Mychal
  * @brief   This is based off the module_ap.c template
  * @details This module controls the behavior of the inrush current limiting (ICL) circuit in the ECM 48 frame
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
//#include "driver_usart1.h"
#include "module_ecm_icl.h"


/* Content ---------------------------------------------------------------------------------------------------------------------*/
extern ProcessInfo processInfoTable[];

#define DemandPollPeriod 2000                                                       //time period for checking and sending 0-10V and speed data to motor board
uint64_t tt_DemandECMICLTime;

//Usart1_Control* usart1Control_AppLocal;
uint8_t testECMICLCounter = 0;
uint64_t errorECMICLCounter = 0;



uint8_t tmpry4ECMICLTest = true; /** this is only for testing the error/log data exchange !!!!!!!!!!!!! please delete these line for production version **/
#define tmpDelayPeriod 100                                                       //time period for checking and sending 0-10V and speed data to motor board
uint64_t tt_DemandtmpECMICLDelayTime;


enum AppStates {
    INIT_ECM_ICL,
    RUN_ECM_ICL,
    // additional states to be added here as necessary.
    IRQ_APP = DEFAULT_IRQ_STATE,
    STOP_APP = KILL_APP
};

/** pam procedure #10 of Module insertion  :  add the module execution function **/
uint8_t p_moduleECM_ICL_u32(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8,
                        uint8_t irq_id_u8) {
    uint8_t return_state_u8 = 0;
    switch (next_State_u8) {
        case INIT_ECM_ICL: {
            //uint8_t Usart1index  = getProcessInfoIndex(MODULE_USART1); //return Process index from processInfo array
            //usart1Control_AppLocal = (Usart1_Control*)((*(processInfoTable[Usart1index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);    //Get structured memory for USART1
            tt_DemandECMICLTime = getSysCount() + DemandPollPeriod;                          //store time tick value  
            return_state_u8 = RUN_ECM_ICL;
            break;
        }
        case RUN_ECM_ICL: {
            if (getSysCount() >= tt_DemandECMICLTime) {
        //        unsigned char speedTx[] = {0x55, 0x01, 0x00, 0xFF, 0x00, (unsigned char)testCounter, (unsigned char)module_id_u8, 0xCC};
        //        unsigned int speedLen = sizeof(speedTx);
        //        RingBuf_WriteBlock((*usart1Control_AppLocal).seqMemTX_u32, speedTx, &speedLen);             
                tt_DemandECMICLTime = getSysCount() + DemandPollPeriod;                          //update next time tick value 
            }      
            
            
            /** this is only for testing the error/log data exchange !!!!!!!!!!!!! please delete these line for production version **/
            tt_DemandtmpECMICLDelayTime = getSysCount() + tmpDelayPeriod + errorECMICLCounter;                          //update next time tick value 
            if(errorECMICLCounter >= 4){ // && (tmpry4Test == true)) {                            //test the error reporting by software interrupt
              setupSoftwareIRQ(module_id_u8, MODULE_ERR_LOGHANDLE, 0xEF, 0x01, 0x00, NULL);
              tmpry4ECMICLTest = false;
              while(getSysCount() < tt_DemandtmpECMICLDelayTime){}
              
            }
           /** this is only for testing the error/log data exchange !!!!!!!!!!!!! please delete these line for production version end **/          
            return_state_u8 = RUN_ECM_ICL;
            break;
        }
        case IRQ_APP: {
            errorECMICLCounter++;
            return_state_u8 = RUN_ECM_ICL;
            break;
        }
        case STOP_APP: {
            return_state_u8 = INIT_ECM_ICL;
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