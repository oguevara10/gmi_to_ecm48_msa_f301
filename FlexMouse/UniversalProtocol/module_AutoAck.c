/**
  ********************************************************************************************************************************
  * @file    module_AutoAck.c 
  * @author  Pamela Lee
  * @version V1.0
  * @date    21-OCT-2020
  * @brief   Main driver module for Auto Acknowledgement of Universal procotol.
  * @details 
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "module_AutoAck.h"
#include "mc_api.h"
#include "ab_module_Mc_StateMachine.h"

//#include "driver_usart1.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
/* Auto acknowledgement handle declaration */
extern ProcessInfo processInfoTable[];

AutoAck_Control *autoAckControl;
Module_StateMachineControl*  module_StateMachineControl_AutoAck;

#define AckTimeOut 5000                        //Ack waiting timeout period
uint64_t Ack_WaitTime; 

uint8_t FrameAckID = 0;
bool ValidRx = false;
#define shifter 1

//******************************************************************************************
static void LinkLost(void);

//*****************************************************************************************
typedef  struct 
{
    uint8_t       AckCmd;
    uint16_t      E_AckCmd;
    uint8_t       ProcessAckID;                                          //Ack process ID
    uint8_t       FrameAckID;                                            //
}UniversalAckInfo;
#define AckBufSize 16
UniversalAckInfo buf[AckBufSize];//= { {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} };
uint8_t Uni_AckHead = 0;
uint8_t Uni_AckTail = 0;
uint8_t Uni_NextFrameID = 0;                    //Unversal Ack frame ID (which the number continue count up and roll over) 
//********************************************************************************************************************************************************

enum {
  INIT_AUTOACK_MODULE,
  RUN_AUTOACK_MODULE,
  // additional states to be added here as necessary.
  IRQ_AUTOACK_MODULE = DEFAULT_IRQ_STATE,
  KILL_AUTOACK_MODULE = KILL_APP
};




uint8_t moduleAutoAck_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8) {
  uint8_t return_state_u8 = INIT_AUTOACK_MODULE;
  switch (next_state_u8) {
    case INIT_AUTOACK_MODULE: {
      // Initialize Auto Ackledgement ring 
      UniversalAckInit();
      
      AutoAckStructMem_u32 = StructMem_CreateInstance(MODULE_AUTOACK, sizeof(AutoAck_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a structured memory
      autoAckControl = (AutoAck_Control*)(*AutoAckStructMem_u32).p_ramBuf_u8;
      (*autoAckControl).RxAckFrameID = 0;
      ValidRx = false;
      Ack_WaitTime = getSysCount() + AckTimeOut;                        //store time tick value  
      uint8_t Mc_StateMachineindex  = getProcessInfoIndex(MODULE_MC_STATEMACHINE);              //return Process index from processInfo array with the MC_statemachine module
      module_StateMachineControl_AutoAck = (Module_StateMachineControl*) ((*(processInfoTable[Mc_StateMachineindex].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
      
      return_state_u8 = RUN_AUTOACK_MODULE;
      break;
    }
    case RUN_AUTOACK_MODULE: {                                                  //        
      if (getSysCount() >= Ack_WaitTime) 
      {
        AckFlushBuf();                                             //make sure the AckHead is pointing to the first Leading record
        if(Uni_AckHead != Uni_AckTail)                                  //if buffer is not empty
        { // timeout the AckHead record
          /**prepare software interrupt for the Ack timeout module**/
          setupSoftwareIRQ(drv_id_u8, buf[Uni_AckHead].ProcessAckID, 0xE0, buf[Uni_AckHead].FrameAckID, buf[Uni_AckHead].AckCmd, NULL);
          
          
 //         uint8_t SoftwareIrqBitPtIndx = buf[Uni_AckHead].ProcessAckID / 64;     // get the interrupt pointer group of software IRQ point index
 //         uint64_t IrqBitTempry = shifter;
 //         SoftwareIrqBitPt[SoftwareIrqBitPtIndx] = IrqBitTempry << (buf[Uni_AckHead].ProcessAckID - (SoftwareIrqBitPtIndx * 64)); //set software interrupt trigger bit
 //         IrqTrigProcessID = drv_id_u8;                                     /**set current module ID to let the IRQ response module know who triggered this interrupt  **/          
         
 //         //find out the ISR module and enter all the parameter for it to respone the interrupt, "this ACK time out error"
 //         uint8_t table_index_u8 = getProcessInfoIndex(buf[Uni_AckHead].ProcessAckID);  
 //         if (table_index_u8 != INDEX_NOT_FOUND) {
 //           processInfoTable[table_index_u8].Sched_DrvData.irqType_u8 = 0xE0;                           /**inform the interrupt response module this is an error message**/
 //           processInfoTable[table_index_u8].Sched_DrvData.irqDat_u8 = buf[Uni_AckHead].FrameAckID;
 
 //           processInfoTable[table_index_u8].Sched_DrvData.irqDat1_len_u8 = buf[Uni_AckHead].AckCmd;     /**The time out command it send **/       
 //           if(buf[Uni_AckHead].FrameAckID == 0x20)
 //           {
 //           processInfoTable[table_index_u8].Sched_DrvData.irqDatPt_u8 = NULL;                          /**usually set as NULL pointer for no extended data (else irqDat_u8 = data length of data pointer)**/
 //           }
 //         }
          AckDeRegistered(buf[Uni_AckHead].FrameAckID);                 //delete and free the first record from buffer          
        }    
        Ack_WaitTime = getSysCount() + AckTimeOut;                      //store time tick value  
        
        if (ValidRx != false)
        {
          ValidRx = false;         
        }
        else
        {
          LinkLost();
        }
      }
      if((*autoAckControl).RxAckFrameID){
        AckDeRegistered((*autoAckControl).RxAckFrameID);                //got auto-ack receiver acknowledment, then de-registered this to fulfil the whole Auto-ack process
        (*autoAckControl).RxAckFrameID = 0;                             //clear the de-resister request
        
      }
      return_state_u8 = RUN_AUTOACK_MODULE;
      break;
    }
    case KILL_AUTOACK_MODULE: {
      // The USART1 driver module must only be executed once.
      // Setting processStatus_u8 to PROCESS_STATUS_KILLED prevents the scheduler main loop from calling this module again.
      uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
      if (table_index_u8 != INDEX_NOT_FOUND) {
        processInfoTable[table_index_u8].Sched_DrvData.processStatus_u8 = PROCESS_STATUS_KILLED;
      }
      return_state_u8 = KILL_AUTOACK_MODULE;
      break;
    }
    default: {
      return_state_u8 = KILL_AUTOACK_MODULE;
      break;
    }
  }
  return return_state_u8;
}

void UniversalAckInit(void)
{
  for(uint8_t index = 0; index < AckBufSize ; index++)
  {
    buf[index].AckCmd = 0;
    buf[index].E_AckCmd = 0;
    buf[index].FrameAckID = 0;
    buf[index].ProcessAckID = 0;
  }  
  Uni_AckHead = 0;
  Uni_AckTail = 0;
  Uni_NextFrameID = 0;
}

uint8_t AckDeRegistered(uint8_t _FrameAckID)                     //Got the ack from the receiver and de-registered the record 
{
  uint8_t _success_u8 = false;
  for(uint8_t index = 0; index < AckBufSize ; index++)
  {
     if(_FrameAckID == buf[index].FrameAckID)                   //search the received FrameAckID in buffer
     {  //clear all this record when found
        buf[index].AckCmd = 0;
        buf[index].E_AckCmd = 0;
        buf[index].FrameAckID = 0;
        buf[index].ProcessAckID = 0;   
        _success_u8 = true;
        AckFlushBuf();
     }
  }  
  return(_success_u8);
}

void AckFlushBuf(void)                                          //FLash any empty record in buffer, starting from the buffer[Uni_AckHead]      
{
  while((Uni_AckHead != Uni_AckTail) && (buf[Uni_AckTail].FrameAckID == 0) ) //push back all the trailing empty record, and move the tail back to the last trailing record
  { //Move head to the closest in buffer
    if(Uni_AckTail != 0 ){
      Uni_AckTail--;
    }
    else
    {
      Uni_AckTail = (AckBufSize - 1);
    }
  } 
  //push back all the leading empty record, and move the head back to the first leading record
  while((Uni_AckHead != Uni_AckTail) && (buf[Uni_AckHead].FrameAckID == 0) )
  { //Move head to the closest in buffer
    if(++Uni_AckHead == AckBufSize)  Uni_AckHead = 0;
  }  
}


uint8_t AckDatSet(uint8_t _AckCmd, uint16_t _E_AckCmd, uint8_t _ProcessAckID)
{
  if(!IsAckBufFull())
  {
    if(Uni_AckHead == Uni_AckTail){                                     //if this is the first data in buffer, reload the timeout value 
      Ack_WaitTime = getSysCount() + AckTimeOut;                        //store time tick value  
    }
    if(++Uni_AckTail == AckBufSize) {
      Uni_AckTail = 0;
    }
    buf[Uni_AckTail].AckCmd = _AckCmd;
    buf[Uni_AckTail].E_AckCmd = _E_AckCmd;
    buf[Uni_AckTail].ProcessAckID = _ProcessAckID;
    if(++Uni_NextFrameID == 0)
    {
      Uni_NextFrameID = 1;
    }
    buf[Uni_AckTail].FrameAckID = Uni_NextFrameID;
    return buf[Uni_AckTail].FrameAckID;  // Return current FrameID to construct the frame 
  }
  return 0;
}

uint8_t IsAckBufFull(void) 
{
  int16_t result = (int16_t)Uni_AckHead - (int16_t)Uni_AckTail;
  if( (result == -(AckBufSize-1)) || (result == 1)) 
  { 
    return (true);
  }  
  return (false);
}

/* 
RPa: do the necessary sequence when Communication link is lost
*/
static void LinkLost(void)
{
  // Add the necessary process when link is lost between app-side and motor-side
  //(*module_StateMachineControl_AutoAck).command_Speed = 0;
}

/*
RPa: Setting the Valid received packet flag which is a private variable
*/
void Set_ValidRx(void)
{
  ValidRx = true;
}
