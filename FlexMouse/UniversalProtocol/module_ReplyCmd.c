/**
  ***************************************************************************************************
  * @file    module_ReplyCmd.c 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    19-OCT-2020
  * @brief   Decode and perform group 3 CMD
  * @note    This App decode Group3 CMD in Universal protocol
  ***************************************************************************************************
  */
#include "pmsm_motor_parameters.h"
#include "module_ReplyCmd.h"
#include "mc_api.h"
#include "driver_usart1.h"
#include "mc_config.h"

extern ProcessInfo processInfoTable[];
extern MCT_Handle_t MCT[NBR_OF_MOTORS];
static MCT_Handle_t *pMCT = &MCT[M1]; 

Usart1_Control* usart1Control_ReplyCmd;

/************** this enum and array made for the periodic resent of motor data back to comBoard ***************************/
typedef enum                                                            //data request cmd list
{                                                                       //please assign according to the universal protocol document
  BusVolt = 0x40,               //item0
  MotFault,                     //item1
  MeaSpeed = 0x60,               //item2
  MotDir = 0x42,               //item3
  MotEE = 0x4F,                 //item4
  MotThermMech = 0x6F,           //item5
  MotTorque = 0x61,                     //item6
  HeartBeat = 0x4E,                    //item7
}ReplyCMD;
         //         item       0  1  2    3    4    5   6   7         -- as the valuable will match in the same way for the enum command list above
uint64_t tt_PerioidTime[]   = {0, 0, 0,   0,   0,   0,  0,  0};                                //please declare total numbers of items in enum, for example three zero with 3 command in enum
uint16_t PerioidTimeValue[] = {0, 0, 0,   0, 1000, 0,   0,  2000};      ///please declare total numbers of items in enum, for example three zero with 3 
                                                 ///command in enum,you can put the default reply period in the relative item, then it will report data automatically.
                                                 /// for example item2 is Measured-speed will send back every 1000mS
/**************************************************************************************************************************/

enum AppStates {
    INIT_APP,
    RUN_APP,
    CMDreply,
    // additional states to be added here as necessary.
    IRQ_APP = DEFAULT_IRQ_STATE,
    STOP_APP = KILL_APP
};

//uint16_t adcFilterVal = 0;
unsigned char* protocolBuf_ReplyCmd ;
//^**Tips: APPs/Drivers adding process example step7 (Add the Additional funtion itself)
uint8_t moduleReplyCmd_u32(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8,
                        uint8_t irq_id_u8)                
{ 
  uint8_t     returnStage = 0;  
  switch (next_State_u8)
    {
      case INIT_APP:                                                              //initial stage
        {     
          /*Attach Uart2 shared memory into this App*/
          uint8_t Usart1index  = getProcessInfoIndex(MODULE_USART1);              //return Process index from processInfo array with the Uart2 driver
          usart1Control_ReplyCmd = (Usart1_Control*) ((*(processInfoTable[Usart1index].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);

          returnStage = RUN_APP ;
          break;
        }       
      case RUN_APP:
        { 
          unsigned int DataLen2 = (unsigned int)UniHeaderlen;
          if(RingBuf_GetUsedNumOfElements((*usart1Control_ReplyCmd).seqMemRXG3_u32) >= DataLen2 )
          {        
            if((protocolBuf_ReplyCmd = (unsigned char*) realloc(protocolBuf_ReplyCmd,DataLen2)) == NULL) reallocError++;     
            RingBuf_Observe((*usart1Control_ReplyCmd).seqMemRXG3_u32, protocolBuf_ReplyCmd, 0, &DataLen2);  
            //calculate the total number of frame
            DataLen2 = ((unsigned int)protocolBuf_ReplyCmd[1] & 0x3F) + (unsigned int)UniHeaderlen;
            if((protocolBuf_ReplyCmd = (unsigned char*) realloc(protocolBuf_ReplyCmd,DataLen2)) == NULL) reallocError++;     //allocate the right frame size of memory for buffer
            RingBuf_ReadBlock((*usart1Control_ReplyCmd).seqMemRXG3_u32, protocolBuf_ReplyCmd, &DataLen2); //extract the whole frame
            //decode and perform the CMD function
            switch((ReplyCMD)protocolBuf_ReplyCmd[2])
            {
              case BusVolt: // point to tt_PerioidTime[0] and PerioidTimeValue[0]
                { //Bus voltage request
                  PerioidTimeValue[0] = (uint16_t)protocolBuf_ReplyCmd[5] << 8;
                  PerioidTimeValue[0] += protocolBuf_ReplyCmd[6];
                  if(PerioidTimeValue[0] > 1)
                  {     // if not one off cmd will start to remember the next wakeup time
                    tt_PerioidTime[0] = getSysCount() + PerioidTimeValue[0];                          //store time tick value
                  }   
                  break;
                }
              case MotFault: // point to tt_PerioidTime[1] and PerioidTimeValue[1]
                { //fault status request
                  PerioidTimeValue[1] = (uint16_t)protocolBuf_ReplyCmd[5] << 8;
                  PerioidTimeValue[1] += protocolBuf_ReplyCmd[6];
                  if(PerioidTimeValue[1] > 1)
                  {     // if not one off cmd will start to remember the next wakeup time
                    tt_PerioidTime[1] = getSysCount() + PerioidTimeValue[1];                          //store time tick value
                  }                
                  break;
                }
              case MeaSpeed: // point to tt_PerioidTime[2] and PerioidTimeValue[2]
                { //measured speed request
                  PerioidTimeValue[2] = (uint16_t)protocolBuf_ReplyCmd[5] << 8;
                  PerioidTimeValue[2] += protocolBuf_ReplyCmd[6];
                  if(PerioidTimeValue[2] > 1)
                  {     // if not one off cmd will start to remember the next wakeup time
                    tt_PerioidTime[2] = getSysCount() + PerioidTimeValue[2];                          //store time tick value
                  }
                  break;
                }
             case MotDir: // point to tt_PerioidTime[3] and PerioidTimeValue[3]
                { //actual direction request
                  PerioidTimeValue[3] = (uint16_t)protocolBuf_ReplyCmd[5] << 8;
                  PerioidTimeValue[3] += protocolBuf_ReplyCmd[6];
                  if(PerioidTimeValue[3] > 1)
                  {     // if not one off cmd will start to remember the next wakeup time
                    tt_PerioidTime[3] = getSysCount() + PerioidTimeValue[3];                          //store time tick value
                  }
                  break;                  
                }                  
             case MotEE: // point to tt_PerioidTime[4] and PerioidTimeValue[4]
                { //measured power request
                  PerioidTimeValue[4] = (uint16_t)protocolBuf_ReplyCmd[5] << 8;
                  PerioidTimeValue[4] += protocolBuf_ReplyCmd[6];
                  if(PerioidTimeValue[4] > 1)
                  {     // if not one off cmd will start to remember the next wakeup time
                    tt_PerioidTime[4] = getSysCount() + PerioidTimeValue[4];                          //store time tick value
                  }
                  break;
                }
             case MotThermMech: // point to tt_PerioidTime[5] and PerioidTimeValue[5]
                { //estimated temperature request
                  PerioidTimeValue[5] = (uint16_t)protocolBuf_ReplyCmd[5] << 8;
                  PerioidTimeValue[5] += protocolBuf_ReplyCmd[6];
                  if(PerioidTimeValue[5] > 1)
                  {     // if not one off cmd will start to remember the next wakeup time
                    tt_PerioidTime[5] = getSysCount() + PerioidTimeValue[5];                          //store time tick value
                  }
                  break;  
                }  
             case MotTorque: // point to tt_PerioidTime[6] and PerioidTimeValue[6]
                { //estimated temperature request
                  PerioidTimeValue[6] = (uint16_t)protocolBuf_ReplyCmd[5] << 8;
                  PerioidTimeValue[6] += protocolBuf_ReplyCmd[6];
                  if(PerioidTimeValue[6] > 1)
                  {     // if not one off cmd will start to remember the next wakeup time
                    tt_PerioidTime[6] = getSysCount() + PerioidTimeValue[6];                          //store time tick value
                  }
                  break;   
                }
             case HeartBeat: // point to tt_PerioidTime[6] and PerioidTimeValue[6]
                { //estimated temperature request
                  PerioidTimeValue[7] = (uint16_t)protocolBuf_ReplyCmd[5] << 8;
                  PerioidTimeValue[7] += protocolBuf_ReplyCmd[6];
                  if(PerioidTimeValue[7] > 1)
                  {     // if not one off cmd will start to remember the next wakeup time
                    tt_PerioidTime[7] = getSysCount() + PerioidTimeValue[7];                          //store time tick value
                  }
                  break;   
                }                
              default:
                break;
            }
          }
          if((protocolBuf_ReplyCmd = (unsigned char*) realloc(protocolBuf_ReplyCmd,1)) == NULL) reallocError++;  
          returnStage = CMDreply;
          break;
        }
      case CMDreply:
        {
          uint8_t CMDindex;
          for(CMDindex = 0; CMDindex < (sizeof(PerioidTimeValue)/sizeof(PerioidTimeValue[0])); CMDindex++)
          {
            if((PerioidTimeValue[CMDindex] == 1) || ((getSysCount() >= tt_PerioidTime[CMDindex]) && ( PerioidTimeValue[CMDindex] != 0)))
            {
              unsigned int TxLen;
              switch(CMDindex)
              {
                case 0:
                  { //Bus voltage request
                    uint16_t busVoltage = VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS);
                    unsigned char busVoltageTx[] = {0x55, 0x02, 0x40, 0x00, 0x00, 0xff, 0xff, 0xCC, 0xCC};
                    TxLen = sizeof(busVoltageTx);
                    busVoltageTx[5] = (unsigned char) ((busVoltage & 0xff00) >> 8);
                    busVoltageTx[6] = (unsigned char) busVoltage & 0xff;
                    RingBuf_WriteBlock((*usart1Control_ReplyCmd).seqMemTX_u32, busVoltageTx, &TxLen); 
                    break;
                  }
                case 1:
                  { //fault status request
                    int16_t faultStatus = MC_GetOccurredFaultsMotor1();
                    unsigned char faultStatusTx[] = {0x55, 0x02, 0x41, 0x00, 0x00, 0xff, 0xff, 0xCC, 0xCC};
                    TxLen = sizeof(faultStatusTx);
                    faultStatusTx[5] = (unsigned char) ((faultStatus & 0xff00) >> 8);
                    faultStatusTx[6] = (unsigned char) faultStatus & 0xff;
                    RingBuf_WriteBlock((*usart1Control_ReplyCmd).seqMemTX_u32, faultStatusTx, &TxLen); 
                    break;
                  }
                case 2:
                  { //measured speed request
                    int16_t Speed = MC_GetMecSpeedAverageMotor1() * 6;
                    unsigned char speedTx[] = {0x55, 0x02, 0x60, 0x00, 0x00, 0xff, 0xff, 0xCC, 0xCC};
                    TxLen = sizeof(speedTx);
                    speedTx[5] = (unsigned char) ((Speed & 0xff00) >> 8);
                    speedTx[6] = (unsigned char) Speed & 0xff;   
                    RingBuf_WriteBlock((*usart1Control_ReplyCmd).seqMemTX_u32, speedTx, &TxLen); 
                    break;
                  }
                case 3:
                  { //actual direction request
                    int16_t Direction = MC_GetImposedDirectionMotor1();
                    unsigned char directionTx[] = {0x55, 0x02, 0x42, 0x00, 0x00, 0xff, 0xff, 0xCC, 0xCC};
                    TxLen = sizeof(directionTx);
                    directionTx[5] = (unsigned char) ((Direction & 0xff00) >> 8);
                    directionTx[6] = (unsigned char) Direction & 0xff;   
                    RingBuf_WriteBlock((*usart1Control_ReplyCmd).seqMemTX_u32, directionTx, &TxLen); 
                    break;
                  }                  
                case 4:
                  { //EE information request                   
                    int16_t Voltage = MC_GetPhaseVoltageAmplitudeMotor1();//RPa: do the necessary conversion either on the motor-side or the app-side
                    int16_t Current = MC_GetPhaseCurrentAmplitudeMotor1();//RPa: do the necessary conversion either on the motor-side or the app-side
                    int16_t Power = MPM_GetAvrgElMotorPowerW(&PQD_MotorPowMeasM1._super);
                    unsigned char EETx[] = {0x55, 0x06, 0x4F, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xCC, 0xCC};
                    TxLen = sizeof(EETx);
                    EETx[5] = (unsigned char) ((Voltage & 0xff00) >> 8);
                    EETx[6] = (unsigned char) Voltage & 0xff;  
                    EETx[7] = (unsigned char) ((Current & 0xff00) >> 8);
                    EETx[8] = (unsigned char) Current & 0xff;
                    EETx[9] = (unsigned char) ((Power & 0xff00) >> 8);
                    EETx[10] = (unsigned char) Power & 0xff;
                    RingBuf_WriteBlock((*usart1Control_ReplyCmd).seqMemTX_u32, EETx, &TxLen); 
                    break;
                  } 
                 case 5:
                  { //Thermo-mechanical information request
                    int16_t Torque = PQD_MotorPowMeasM1.pFOCVars->Iqd.q;
                    int16_t Temperature = NTC_GetAvTemp_C(pMCT->pTemperatureSensor);// RPa: update this call                  
                    unsigned char ThMETx[] = {0x55, 0x04, 0x6F, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xCC, 0xCC};
                    TxLen = sizeof(ThMETx);
                    ThMETx[5] = (unsigned char) ((Torque & 0xff00) >> 8);
                    ThMETx[6] = (unsigned char) Torque & 0xff;   
                    ThMETx[7] = (unsigned char) ((Temperature & 0xff00) >> 8);
                    ThMETx[8] = (unsigned char) Temperature & 0xff;  
                    RingBuf_WriteBlock((*usart1Control_ReplyCmd).seqMemTX_u32, ThMETx, &TxLen); 
                    break;
                  }  
                 case 6:
                  { //Motor Torque request
                    int16_t Torque = PQD_MotorPowMeasM1.pFOCVars->Iqd.q;         
                    unsigned char TorqueTx[] = {0x55, 0x04, 0x61, 0x00, 0x00, 0xff, 0xff, 0xCC, 0xCC};
                    TxLen = sizeof(TorqueTx);
                    TorqueTx[5] = (unsigned char) ((Torque & 0xff00) >> 8);
                    TorqueTx[6] = (unsigned char) Torque & 0xff;  
                    RingBuf_WriteBlock((*usart1Control_ReplyCmd).seqMemTX_u32, TorqueTx, &TxLen); 
                    break;
                  } 
                 case 7:
                  { //HeartBeat Request     
                    unsigned char HeartBeatTx[] = {0x55, 0x04, 0x4E, 0x5A, 0xA5, 0xff, 0xff, 0xCC, 0xCC};// Just send 0x5AA5 to App-side and get a response
                    TxLen = sizeof(HeartBeatTx);
                    RingBuf_WriteBlock((*usart1Control_ReplyCmd).seqMemTX_u32, HeartBeatTx, &TxLen); 
                    break;
                  }                   
                default:
                    break;
              }  
              if(PerioidTimeValue[CMDindex] == 1) 
              {
                PerioidTimeValue[CMDindex] = 0;
              }
              else
              {
                tt_PerioidTime[CMDindex] = getSysCount() + PerioidTimeValue[CMDindex];                          //store  next time tick compare value
              }
            }
          }
          returnStage = RUN_APP ;
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

