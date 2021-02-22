/**
  ***************************************************************************************************
  * @file    module_FlashUpdateCmd.c 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    10-DEC-2020
  * @brief   Decode and perform group 4 CMD
  * @note    This App decode Group4 CMD in Universal protocol for all flash/setting update
  *          Flash Block mode: User should update the whole page of setting in a single/multi Page. 
  *                            when transfer a page of setting will be divided into 32bytes per block
  *                            always end with block0 (when receive block0 will finialize the whole update) 
  *                            and can start with any other blockx, 
  *                            @caution transfer must alsway contain the last block of the page for CRC  
  ***************************************************************************************************
  */
#include "pmsm_motor_parameters.h"
#include "module_FlashUpdateCmd.h"
#include "zz_module_flash.h"
//#include "mc_api.h"
#include "driver_usart2.h"
//#include "mc_config.h"
#include "ab_module_Mc_StateMachine.h"
#include "mc_tasks.h"
#include "mc_type.h"

extern ProcessInfo processInfoTable[];


Usart2_Control* usart2Control_FlashUpdateCmd;

typedef enum                                                            //data request cmd list
{   //match universal protocol (group4)                                 //please assign according to the universal protocol document
  FlashBlkWr = 0x78,
  FlashBlkRd,
  FlashBlkControl
}FlashUpdateCMD;

typedef enum
{
    ACK,                        //(this block good) Acknowledge
    resend,                     //bad block request to resend
    clrBlkTransfer,             //clear block transfer
    blkOverRun,                 //block over-run
    blkFlashUpdateSuccess,      //blk flash update success and request to reboot
    blkFlashUpdateFail,         //blk flash update fail and request to reboot
    rebootAck,                  //premission of reboot acknowledge from master 
    reqFlashFormat,             //flash data fromat request, number of block per sector, number of sector
    flashFormatResponse         //response of total number of blocks and block  per page  
}FlashControlCMD;

/****************flash write local variable ********************************/
#define flashBlockSize 32
//#define FLASH_PAGE_SIZE 0x800
#define FLASH_BLOCK_SETTING_PAGE ADDR_FLASH_PAGE_30
#define MIRROR_FLASH_BLOCK_SETTING_PAGE ADDR_FLASH_PAGE_31

typedef enum
{
  idle,
  copyCurrentSetting,           //copy current valid setting to buffer, and erase the actural flash setting sector
  burnPayload2Flash,            //burn data into flash sector and send back Ack for next block of data
  wait4NextBlock,               //wait for next block, Check block CRC if new block arrive
  finalFullCRC,                 //whole flash sector validation and prepare to restart
  
  finalValidFail,               //copy back the last valid flash setting in buffer and prepare to restart  
  flashProgrammingFail,         //any flash programming error will come here 
  AppSideRebootAck,             //send message to app-side and wait for reboot acknowledgement
  SystemReboot                  //Warm boot the system
}flashBlkWrState;

/** Module control struct 
    uint16_t flashBlockPt;              //store the last received frame flash block page number;
    flashBlkWrState flashWrState;       //Flash Block mode statemachine state-pointer 
    uint16_t error;                     //Error code: 0x01=> FlashPageCRCFail
                                        //            0x02=> FlashPageEraseError
                                        //            0x03=> FlashPageDataProgrammingError
**/
typedef struct
{
  uint16_t flashBlockPt;        //store the last received frame flash block page number;
  flashBlkWrState flashWrState; 
  uint16_t error;
}flashBlockWr_control;
 
uint64_t tt_FlashFrameTimeOut;                                //
#define FlashFrameTimeOutValue 2000000                          //Time out value between the last block Flash frame to the next frame
/****************flash write local variable end*****************************/

typedef enum  
{
    INIT_APP,
    RUN_APP,
    CMDreply,
    // additional states to be added here as necessary.
    IRQ_APP = DEFAULT_IRQ_STATE,
    STOP_APP = KILL_APP
}AppStates;

Module_StateMachineControl*  module_StateMachineControl_FlashUpdateCmd;
flashBlockWr_control flashBlockWrDat;
unsigned char* protocolBuf_FlashUpdateCmd ;
//uint16_t _uwCRCValue;  /**@pam testing **/
uint16_t blkCalculatedCRC; /**@pam testing **/
AppStates   returnStage = INIT_APP;  
uint8_t moduleFlashUpdateCmd_u32(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8, uint8_t irq_id_u8)                
{ 
  switch (next_State_u8)
    {
      case INIT_APP:                                                              //initial stage
        {     
          /*Attach Uart2 shared memory into this App*/
          uint8_t Usart2index  = getProcessInfoIndex(MODULE_USART2);              //return Process index from processInfo array with the Uart2 driver
          usart2Control_FlashUpdateCmd = (Usart2_Control*) ((*(processInfoTable[Usart2index].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
          uint8_t Mc_StateMachineindex  = getProcessInfoIndex(MODULE_MC_STATEMACHINE);              //return Process index from processInfo array with the MC_statemachine module
          module_StateMachineControl_FlashUpdateCmd = (Module_StateMachineControl*) ((*(processInfoTable[Mc_StateMachineindex].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);

          flashBlockWrDat.flashWrState = idle;
          returnStage = RUN_APP ;
          break;
        }       
      case RUN_APP:
        { 
          unsigned int DataLen2 = (unsigned int)UniHeaderlen;
          if(RingBuf_GetUsedNumOfElements((*usart2Control_FlashUpdateCmd).seqMemRXG4H_u32) >= DataLen2 )
          {  
           
            
            /***-------------------------- pre-determine what state is in for Rx data is valid or not valid between blockmode transfer ------------------------**/
            switch(flashBlockWrDat.flashWrState)
            {
              case idle:
              case wait4NextBlock: 
              case AppSideRebootAck: 
                {   //Error if new frame happen in between the stateMachine process!!!!!
                    if((protocolBuf_FlashUpdateCmd = (unsigned char*) realloc(protocolBuf_FlashUpdateCmd,DataLen2)) == NULL) reallocError++;     
                    RingBuf_Observe((*usart2Control_FlashUpdateCmd).seqMemRXG4H_u32, protocolBuf_FlashUpdateCmd, 0, &DataLen2);  
                    //calculate the total number of frame
                    DataLen2 = ((unsigned int)protocolBuf_FlashUpdateCmd[1] & 0x3F) + (unsigned int)UniHeaderlen;
                    if((protocolBuf_FlashUpdateCmd = (unsigned char*) realloc(protocolBuf_FlashUpdateCmd,DataLen2)) == NULL) reallocError++;     //allocate the right frame size of memory for buffer
                    RingBuf_ReadBlock((*usart2Control_FlashUpdateCmd).seqMemRXG4H_u32, protocolBuf_FlashUpdateCmd, &DataLen2); //extract the whole frame
                    break;
                }
              
              default:
                {
                    RingBuf_ClearContents((*usart2Control_FlashUpdateCmd).seqMemRXG4H_u32); //clear all content in buffer   
                    //send error message to sender
                    unsigned char FlashBlkErrorOverun[] = {0x55, 0x01, FlashBlkControl, 0x00, 0x00, 0x04, 0xCC, 0xCC}; //block over-run error
                    unsigned int TxLen = sizeof(FlashBlkErrorOverun);
                    RingBuf_WriteBlock((*usart2Control_FlashUpdateCmd).seqMemTX_u32, FlashBlkErrorOverun, &TxLen); 
                    break;
                }
            }
            /**-------------------------------------------- decode and perform the CMD function ------------------------------------------------**/
            switch((FlashUpdateCMD)protocolBuf_FlashUpdateCmd[2])
            {
              case FlashBlkWr: 
                { //Flash block write
                  if((flashBlockWrDat.flashWrState == idle) || ( flashBlockWrDat.flashWrState == wait4NextBlock)) { //not start                    
                    uint16_t BlkCRCValue = (((uint16_t)protocolBuf_FlashUpdateCmd[flashBlockSize+7]) << 8) + ((uint16_t)protocolBuf_FlashUpdateCmd[flashBlockSize+8]) ;
                    blkCalculatedCRC = Calculate_CRC( flashBlockSize , (unsigned char*)&protocolBuf_FlashUpdateCmd[7]);                                       //Get calculated CRC of this block
                    if(blkCalculatedCRC != BlkCRCValue)
                    { //CRC not match   means  error in receiving block, call for resend this block
                      unsigned char FlashBlkErrorResend[] = {0x55, 0x03, FlashBlkControl, 0x00, 0x00, 0x01, protocolBuf_FlashUpdateCmd[5], protocolBuf_FlashUpdateCmd[6], 0xCC, 0xCC}; //request resend the same block again
                      unsigned int TxLen = sizeof(FlashBlkErrorResend);
                      RingBuf_WriteBlock((*usart2Control_FlashUpdateCmd).seqMemTX_u32, FlashBlkErrorResend, &TxLen); 
                    } 
                    else{
                      flashBlockWrDat.flashBlockPt = (((uint16_t) protocolBuf_FlashUpdateCmd[5]) << 8) + ((uint16_t)protocolBuf_FlashUpdateCmd[6]); //store the current block number
                      flashBlockWrDat.flashWrState = copyCurrentSetting;                                      //prepare flash for new settings
                      (*module_StateMachineControl_FlashUpdateCmd).command_Speed = 0;                   //stop motor
                      flashBlkWriteStateMachine_Run(module_id_u8);                                      //start up block flash update stateMachine
                    } 
                  }
//                else {   //shouldn't run to this line, if so something wrong!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! }
                  break;
                }
              case FlashBlkRd: 
                { //Flash block read
                  uint16_t flashBlkNum = ((uint16_t )protocolBuf_FlashUpdateCmd[5]) << 8 ;
                  unsigned char* rdFlashBuf = (unsigned char*)FLASH_BLOCK_SETTING_PAGE;
                  flashTxBlk(flashBlkNum + protocolBuf_FlashUpdateCmd[6], rdFlashBuf);
                                  
                  break;
                }
              case FlashBlkControl: 
                { 
                  switch((FlashControlCMD)protocolBuf_FlashUpdateCmd[5])
                  {
                    case rebootAck:                   //premission of reboot acknowledge from master 
                      { //Flash block Ack/Nack resend
                        if(flashBlockWrDat.flashWrState == AppSideRebootAck) {
                          flashBlockWrDat.flashWrState = SystemReboot; 
                          flashBlkWriteStateMachine_Run(module_id_u8);
                        }
                        break;
                      }
                    case reqFlashFormat:              //flash data format request, number of block per sector, number of sector
                      { //Response for the flash data format 
                        unsigned char blkPerSector = FLASH_PAGE_SIZE/flashBlockSize;
                        unsigned char FlashBlkFormat[] = {0x55, 0x03, FlashBlkControl, 0x00, 0x00, 0x09, blkPerSector, 0x01, 0xcc, 0xcc};
                        unsigned int TxLen = sizeof(FlashBlkFormat);
                        RingBuf_WriteBlock((*usart2Control_FlashUpdateCmd).seqMemTX_u32, FlashBlkFormat, &TxLen); 
                        break;
                      }     
                  }
                  break;
                }   
              default:
                break;
            }
          }
          else
          {
            if(flashBlockWrDat.flashWrState != idle) flashBlkWriteStateMachine_Run(module_id_u8);  //still in block flash stateMachine 
          }
      //    if((protocolBuf_FlashUpdateCmd = (unsigned char*) realloc(protocolBuf_FlashUpdateCmd,1)) == NULL) reallocError++;  
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


/** @section below -------------------------------------------------- All Local stateMachine functions (below this line)------------------------------------------------------------------
  * @author  Pamela Lee
  * @date    12 Jan 2021
  * @version 1.0
  * @brief   Contain all the stateMachine functions for each of the Flash setting and register mode
  *---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
**/
uint8_t flashBlkWriteStateMachine_Run(uint8_t _module_id_u8)                    
{
  switch((flashBlkWrState)flashBlockWrDat.flashWrState)
  {
    case copyCurrentSetting:
      { //copy current valid setting to buffer, and erase the actural flash setting sector
        uint16_t tmprydat = ((((((uint16_t)protocolBuf_FlashUpdateCmd[5]) << 8) + protocolBuf_FlashUpdateCmd[6])+1) * flashBlockSize);
        if ( tmprydat == FLASH_PAGE_SIZE)  //block flash will only erase the sector when receive the last block(so always start with the last block(in the 32byte per block of 2ksector size will be block63)
        {
          /*********************** backup the last known setting into the Mirror flash page *********************************/
          if(!flashPageErase(_module_id_u8, MIRROR_FLASH_BLOCK_SETTING_PAGE))
          {  //Sector erase error
              flashBlockWrDat.error = 0x02;                         //Error code: 0x02=> FlashPageEraseError
              flashBlockWrDat.flashWrState = flashProgrammingFail;  //if error the Flash driver should already report it
          }  
          flashPageCopy(_module_id_u8, FLASH_BLOCK_SETTING_PAGE, MIRROR_FLASH_BLOCK_SETTING_PAGE);
          /** after backup the last known setting then erase the new setting page area **/
          if(!flashPageErase(_module_id_u8, FLASH_BLOCK_SETTING_PAGE))
          {  //Sector erase error
              flashBlockWrDat.error = 0x02;                         //Error code: 0x02=> FlashPageEraseError
              flashBlockWrDat.flashWrState = flashProgrammingFail;  //if error the Flash driver should already report it
          }            
        }
        flashBlockWrDat.flashWrState = burnPayload2Flash;       //seccuss proceed to burn data into flash state
        break;
      }
    case burnPayload2Flash:
      { //burn data into flash sector and send back Ack for next block of data, and free up protocolBuf_FlashUpdateCmd for next block
        uint32_t block_address = (FLASH_BLOCK_SETTING_PAGE+(flashBlockSize * ((((uint16_t)protocolBuf_FlashUpdateCmd[5]) << 8) + protocolBuf_FlashUpdateCmd[6]))) ;
        if(!flashBlockProgram(_module_id_u8, block_address,  (uint8_t*)&protocolBuf_FlashUpdateCmd[7], flashBlockSize))
        {   //Block programming flash error
            flashBlockWrDat.error = 0x03;                         //Error code: 0x03=> FlashPageDataProgrammingError
            flashBlockWrDat.flashWrState = flashProgrammingFail;  //if error the Flash driver should already report it
        }
        else
        {
            unsigned char FlashBlkErrorResend[] = {0x55, 0x03, FlashBlkControl, 0x00, 0x00, 0x00, protocolBuf_FlashUpdateCmd[5], protocolBuf_FlashUpdateCmd[6], 0xCC, 0xCC}; //Ack the last block number to sender for requesting next block 
            unsigned int TxLen = sizeof(FlashBlkErrorResend);
            RingBuf_WriteBlock((*usart2Control_FlashUpdateCmd).seqMemTX_u32, FlashBlkErrorResend, &TxLen); 
            flashBlockWrDat.flashWrState = wait4NextBlock;     //all done with this block proceed to wait for next block
            tt_FlashFrameTimeOut = getSysCount() + FlashFrameTimeOutValue;                          //setup timeout period  **/
        }
        break;
      }
    case wait4NextBlock:
      { //wait for next block or the last block is block zero(final block) , Check block CRC if new block arrive
        if (!(protocolBuf_FlashUpdateCmd[5] || protocolBuf_FlashUpdateCmd[6])) /** check this is the final block**/
        {       //final block
          flashBlockWrDat.flashWrState = finalFullCRC;
        }
        //loop here and waiting for next block, if timeout then fault it
        if (getSysCount() >= tt_FlashFrameTimeOut)   // for timeout check**/  
        {  //Flash frame waiting time out
          flashBlockWrDat.flashWrState = flashProgrammingFail;          
        }
        //before timeout happen, just do nothing for the next flashBlockWrDat.flashWrState will remain the same to loop here
        break;
      }
    case finalFullCRC:
      { //send flash block setting update successfully to app-side, whole flash sector validation and prepare to restart
        if(isFlashCRCValid(FLASH_BLOCK_SETTING_PAGE, 1))
        {   //send update successfully message to App-side
            unsigned char FlashBlkSuccessRebootReq[] = {0x55, 0x01, FlashBlkControl, 0x00, 0x00, 0x05, 0xCC, 0xCC}; //setting update seccuss, request for reboot
            unsigned int TxLen = sizeof(FlashBlkSuccessRebootReq);
            RingBuf_WriteBlock((*usart2Control_FlashUpdateCmd).seqMemTX_u32, FlashBlkSuccessRebootReq, &TxLen); 
            flashBlockWrDat.flashWrState = AppSideRebootAck; //successfully update whole page and proceed to wait APP-Side to validate the reboot 
        }
        else
        {
          flashBlockWrDat.error = 0x01;                        //Error code: 0x01=> FlashPageCRCFail
          flashBlockWrDat.flashWrState = flashProgrammingFail; //fail the block CRC       
        }
        break;
      }
    case finalValidFail:  
      { //copy back the last valid flash setting in buffer and prepare to restart    
        if(isFlashCRCValid(MIRROR_FLASH_BLOCK_SETTING_PAGE, 1))         //check the mirror/buffer is still valid or not 
        {
          if(flashPageErase(_module_id_u8, FLASH_BLOCK_SETTING_PAGE))           //Erase main setting flash page again
          {  //copy Mirror page back to main flash page
            flashPageCopy(_module_id_u8, MIRROR_FLASH_BLOCK_SETTING_PAGE, FLASH_BLOCK_SETTING_PAGE);
            flashBlockWrDat.flashWrState = AppSideRebootAck;
          } 
        }
        flashBlockWrDat.flashWrState = flashProgrammingFail;  //if error the Flash driver should already report it
        break;
      }
    case flashProgrammingFail:
      { //any flash programming error that cannot recovered will come here 
        /** Make sure the motor has stopped before disable the motor **/
        (*module_StateMachineControl_FlashUpdateCmd).motorEnable = FALSE;       //disable the motor if flash setting completely fail 
        
        break;
      }
    case AppSideRebootAck:
      {
        //this state just for loop aand wait for acknowledge!!
        break;
      }
    case SystemReboot:
      {
        /**@Caution System will be warm boot after this line, make sure store everything necessary before this line **/
        __NVIC_SystemReset();     
        break;
      }
    default:
      {
        flashBlockWrDat.flashWrState = idle;
        break;  
      }
  }  
  return 0;
}

uint8_t flashTxBlk(uint16_t _flashBlkNum, unsigned char* _buf)
{
  uint16_t blkStartAddr = _flashBlkNum*flashBlockSize;
  if ( _flashBlkNum > 0x8000) //if flashBlkNum > 0x8000 mean this is an absolute block number 
  {
    blkStartAddr = 0;
    _flashBlkNum &= 0xfff;
  }
  uint16_t uwCRCValue = Calculate_CRC(32, _buf + blkStartAddr);          //only flash data CRC, no block pointer.
  unsigned char blkSettingTx[] = {0x55, 0x24, FlashBlkRd, 0x00, 0x00, (_flashBlkNum & 0xff00) >> 8, _flashBlkNum & 0xff,\
                                  _buf[ 0 + blkStartAddr],  _buf[1 + blkStartAddr],  _buf[2 + blkStartAddr],  _buf[3 + blkStartAddr],  _buf[4 + blkStartAddr],  _buf[5 + blkStartAddr],  _buf[6 + blkStartAddr],  _buf[7 + blkStartAddr],  _buf[8 + blkStartAddr],  _buf[9 + blkStartAddr],\
                                  _buf[10 + blkStartAddr], _buf[11 + blkStartAddr], _buf[12 + blkStartAddr], _buf[13 + blkStartAddr], _buf[14 + blkStartAddr], _buf[15 + blkStartAddr], _buf[16 + blkStartAddr], _buf[17 + blkStartAddr], _buf[18 + blkStartAddr], _buf[19 + blkStartAddr],\
                                  _buf[20 + blkStartAddr], _buf[21 + blkStartAddr], _buf[22 + blkStartAddr], _buf[23 + blkStartAddr], _buf[24 + blkStartAddr], _buf[25 + blkStartAddr], _buf[26 + blkStartAddr], _buf[27 + blkStartAddr], _buf[28 + blkStartAddr], _buf[29 + blkStartAddr],\
                                  _buf[30 + blkStartAddr], _buf[31 + blkStartAddr],\
                                  (uwCRCValue & 0xff00) >> 8, uwCRCValue & 0xff, 0xCC, 0xCC};
  
  unsigned int blkSettingLen = sizeof(blkSettingTx);
  RingBuf_WriteBlock((*usart2Control_FlashUpdateCmd).seqMemTX_u32, blkSettingTx, &blkSettingLen);
  
  return 0;
}
