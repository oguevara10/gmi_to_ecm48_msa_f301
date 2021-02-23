/**
  ***************************************************************************************************
  * @file    App_PeriodicReplyCmd.c 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    1-Jul-2020
  * @brief   Main function/s (simple is a template APP)
  * @note    this example do nothing
  ***************************************************************************************************
  */

#include "App_PeriodicReplyCmd.h"
#include "D_Usart1.h"

extern uint64_t getSysCount(void);
/* SysCon handle declaration */
extern ShareMemory ShMem[totalshMem];
extern RingBuffer RB[totalpipe];
extern ProcessInfo processInfo[];
extern ShareMemory* Usart1ShMem;

Usart1Control* usart1Control_PeriodicReplyCmd;


/* SysCon handle declaration */
//extern ShareMemory ShMem[totalshMem];
//extern RingBuffer RB[totalpipe];
extern ProcessInfo processInfo[];

enum                                                                            //Default APPs/Driver stage template
{ 
  AppInit,
  AppStart,
  //any other stage in here !!!

  //above 200 will be all interrupt for this APP
  AppInterrupt = 200,
  killApp = 255
};

unsigned char* protocolBuf_PeriodicReplyCmd ;
//^**Tips: APPs/Drivers adding process example step7 (Add the Additional funtion itself)
uint8_t _App_PeriodicReplyCmd(uint8_t appID, uint8_t previousStage, uint8_t nextStage, uint8_t interruptIdentfer)                  
{ 
  uint8_t     returnStage = 0;  
  
  switch (nextStage)
    {
      case AppInit:                                                              //initial stage
        {
          uint8_t index = getProcessInfoIndex(appID);                            //return Process index from processInfo array with the appID
          if(index !=255)
          {
            processInfo[index].DrvDat.interruptStage = 200;                     //interrupt stage default at stage/state 200
          }   
          //Attach Uart2 shared memory into this App//
          uint8_t Usart1index  = getProcessInfoIndex(DM_Usart1);              //return Process index from processInfo array with the Uart2 driver
          usart1Control_PeriodicReplyCmd = (Usart1Control*) (*(processInfo[Usart1index].DrvDat.MasterShMemPtr)).ShMemBuf;         
          returnStage = AppStart ;
          break;
        }       
      case AppStart:
        { 
          unsigned int DataLength2 = (unsigned int)UniHeaderlen;
          if(RBUsedSpace((*usart1Control_PeriodicReplyCmd).RxG4Pipe->SystemIndex) >= DataLength2 )
          {        
            protocolBuf_PeriodicReplyCmd = (unsigned char*) realloc(protocolBuf_PeriodicReplyCmd,DataLength2);     
            RBPeek((*usart1Control_PeriodicReplyCmd).RxG4Pipe->SystemIndex, protocolBuf_PeriodicReplyCmd, 0, &DataLength2);  
            //calculate the total number of frame
            DataLength2 = ((unsigned int)protocolBuf_PeriodicReplyCmd[1] & 0x3F) + (unsigned int)UniHeaderlen;
            protocolBuf_PeriodicReplyCmd = (unsigned char*) realloc(protocolBuf_PeriodicReplyCmd,DataLength2);                  //allocate the right frame size of memory for buffer
            RBRead((*usart1Control_PeriodicReplyCmd).RxG3Pipe->SystemIndex, protocolBuf_PeriodicReplyCmd, &DataLength2);        //extract the whole frame
             //decode and perform the CMD function
            switch(protocolBuf_PeriodicReplyCmd[2])
            {
              case 0xC0:
                { 
                  /*
                  uint16_t busVoltage = VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS);
                  unsigned char busVoltageTx[] = {0x55, 0x02, 0x40, 0x00, 0x00, 0xff, 0xff, 0xCC, 0xCC};
                  unsigned int busVoltageLen = sizeof(busVoltageTx);

                  busVoltageTx[5] = (unsigned char) ((busVoltage & 0xff00) >> 8);
                  busVoltageTx[6] = (unsigned char) busVoltage & 0xff;
                  RBWriteBlk((*usart1Control_ReplyCmd).TxPipe->SystemIndex, busVoltageTx, &busVoltageLen);  */
                  break;
                }
              default:
                break;
            }  
          }
          protocolBuf_PeriodicReplyCmd = (unsigned char*) realloc(protocolBuf_PeriodicReplyCmd,1);     
          returnStage = AppStart ;            
          break;
        }
      case AppInterrupt:
        {
          //if more than 1 driver interrupt attached to this APP
//           uint8_t index = getProcessInfoIndex(interruptIdentfer);         //return Process index from processInfo array of the driver interrupt call, APP can response respectively
          returnStage = AppStart;
          break;
        }               
      case killApp:
        {
          returnStage = AppInit;
          break;
        }
      default:
        returnStage = killApp;   
    }
  return returnStage;
}

