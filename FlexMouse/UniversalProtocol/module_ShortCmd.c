/**
  ***************************************************************************************************
  * @file    module_ShortCmd.c 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    1-Jul-2020
  * @brief   Decode and perform group 1 and 2 CMD
  * @note    This App decode Group1 and 2 CMD in Universal protocol
  ***************************************************************************************************
  */
#include "pmsm_motor_parameters.h"
#include "module_ShortCmd.h"
#include "mc_api.h"
#include "driver_usart1.h"
#include "ab_module_Mc_StateMachine.h"


extern uint64_t getSysCount(void);
/* SysCon handle declaration */
extern ProcessInfo processInfoTable[];

Usart1_Control* usart1Control_ShortCmd;
Module_StateMachineControl*  module_StateMachineControl_ShortCmd;

enum AppStates {
    INIT_APP,
    RUN_APP,
    // additional states to be added here as necessary.
    IRQ_APP = DEFAULT_IRQ_STATE,
    STOP_APP = KILL_APP
};

//uint16_t adcFilterVal = 0;
unsigned char* protocolBuf_ShortCmd ;
//^**Tips: APPs/Drivers adding process example step7 (Add the Additional funtion itself)
uint8_t moduleShortCmd_u32(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8,
                        uint8_t irq_id_u8)                 
{ 
  uint8_t     returnStage = 0;  
  
  switch (next_State_u8)
    {
      case INIT_APP:                                                              //initial stage
        {
          /*Attach Uart2 shared memory into this App*/
          uint8_t Usart1index  = getProcessInfoIndex(MODULE_USART1);              //return Process index from processInfo array with the Uart2 driver
          usart1Control_ShortCmd = (Usart1_Control*) ((*(processInfoTable[Usart1index].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
          uint8_t Mc_StateMachineindex  = getProcessInfoIndex(MODULE_MC_STATEMACHINE);              //return Process index from processInfo array with the MC_statemachine module
          module_StateMachineControl_ShortCmd = (Module_StateMachineControl*) ((*(processInfoTable[Mc_StateMachineindex].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
          
          returnStage = RUN_APP ;
          break;
        }       
      case RUN_APP:
        { 
          unsigned int DataLen2 = (unsigned int)UniHeaderlen;
          if(RingBuf_GetUsedNumOfElements((*usart1Control_ShortCmd).seqMemRXG1_2_u32) >= DataLen2 )
          {        
            if((protocolBuf_ShortCmd = (unsigned char*) realloc(protocolBuf_ShortCmd,DataLen2)) == NULL) reallocError++;     
            RingBuf_Observe((*usart1Control_ShortCmd).seqMemRXG1_2_u32, protocolBuf_ShortCmd, 0, &DataLen2);  
            //calculate the total number of frame
            DataLen2 = ((unsigned int)protocolBuf_ShortCmd[1] & 0x3F) + (unsigned int)UniHeaderlen;
            if((protocolBuf_ShortCmd = (unsigned char*) realloc(protocolBuf_ShortCmd,DataLen2)) == NULL) reallocError++;     //allocate the right frame size of memory for buffer
            RingBuf_ReadBlock((*usart1Control_ShortCmd).seqMemRXG1_2_u32, protocolBuf_ShortCmd, &DataLen2); //extract the whole frame
            //decode and perform the CMD function
            switch(protocolBuf_ShortCmd[2])
            {
              case 0x00:
                {
                  MC_StartMotor1();
                  break;
                }
              case 0x01:
                {
                  MC_StopMotor1();                  
                  break;
                }
              case 0x02:
                {
                  MC_StopRampMotor1();
                  break;
                }
              case 0x03:
                {
                  MC_AcknowledgeFaultMotor1();
                  break;
                }
              case 0x21:
                {
                  int32_t speed_target = protocolBuf_ShortCmd[6];
                  speed_target += (int16_t) protocolBuf_ShortCmd[5] << 8;
                  (*module_StateMachineControl_ShortCmd).command_Speed = speed_target;
                  
                  uint8_t dir_target = (uint8_t) protocolBuf_ShortCmd[7];
                  
                  if (dir_target == 6)(*module_StateMachineControl_ShortCmd).motorDir = 1;
                  if (dir_target == 9) (*module_StateMachineControl_ShortCmd).motorDir = -1;
                  
                  break;
                }
              default:
                break;
            }

          }
          if((protocolBuf_ShortCmd = (unsigned char*) realloc(protocolBuf_ShortCmd,1)) == NULL) reallocError++;;
          returnStage = RUN_APP;
          break;
        }
      case IRQ_APP:
        {
          //if more than 1 driver interrupt attached to this APP
//           uint8_t index = getProcessInfoIndex(interruptIdentfer);         //return Process index from processInfo array of the driver interrupt call, APP can response respectively
          returnStage = RUN_APP;
          break;
        }               
      case STOP_APP:
        {
          returnStage = INIT_APP;
          break;
        }
      default:
        returnStage = STOP_APP;   
    }
  return returnStage;
}

