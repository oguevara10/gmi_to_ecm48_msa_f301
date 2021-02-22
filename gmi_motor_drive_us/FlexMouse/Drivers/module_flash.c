/**
********************************************************************************************************************************
* @file    module_flash.c 
* @author  Pamela Lee
* @brief   Main driver module for flash.
* @details This module initializes the flash
*          The ST motor libraries parameters will be mapped from the top of FLASH_USER_START_ADDR in 16bit data,
*          the data can be updated by using the function of uint8_t FlashDatSet(uint16_t _offset, uint16_t _flashDat), then 
*          data and offset will store in internal buffer as temporary data, user can either store all the temporary data into flash 
*          by flashPageUpdate(), or if the internal buffer is full will also update the temporary into flash.
********************************************************************************************************************************
*/

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "zz_module_flash.h"
#include "user_interface.h"
#include "motor_control_protocol.h"
#include "ui_task.h"


/* Content ---------------------------------------------------------------------------------------------------------------------*/

//extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];
extern VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1;
uint32_t Address = 0;                                     //, PageError = 0;
__IO uint8_t readData = 0 , MemoryProgramStatus = 0;

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;


//*****************************************************************************************
typedef  struct 
{
    uint16_t      offset;
    uint16_t      flashDat;
}FlashBufInfo;
#define FlashBufSize 20
FlashBufInfo flashBuf[FlashBufSize];
uint8_t Flash_BufHead = 0;
uint8_t Flash_BufTail = 0;
//********************************************************************************************************************************************************

enum {
  INIT_MODULE,
  RUN_MODULE,
  // additional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

uint8_t moduleFlash_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8) {
  uint8_t return_state_u8 = INIT_MODULE;
  switch (next_state_u8) {
    case INIT_MODULE: {
      FlashBufInit();
      /** for pam testing the buffer system **/
     /*
      FlashDatSet(0, 0xaaaa);
      FlashDatSet(0x1ff, 0x5555);
      FlashDatSet(0x122, 0x5555);   //going to delete before write inflash
      FlashDatSet(0x3fe, 0xaaaa);
      FlashDatSet(0x200, 0x5555);
      FlashDatSet(0x3fe, 0xaa55);   //going to replace the former same address
      FlashDatSet(0x0ff, 0xaaaa);
      FlashDatSet(0x320, 0xaaaa);
      FlashDatSet(0x200, 0xaaaa);
      
      flashPageErase(drv_id_u8, FLASH_USER_START_ADDR, 2);       //elase all two page for update 
      
      flashPageUpdate(drv_id_u8, (FLASH_USER_START_ADDR + FLASH_PAGE_SIZE), FLASH_USER_START_ADDR, 1); //copy the lower page to upper page with ram data in buffer
      
      flashPageCopy(drv_id_u8, FLASH_USER_START_ADDR, (FLASH_USER_START_ADDR + FLASH_PAGE_SIZE), 1); //copy the whole page from one to other
      */
    //  flashPageErase(drv_id_u8, ADDR_FLASH_PAGE_31);
   //   flashPageCopy(drv_id_u8, ADDR_FLASH_PAGE_30, ADDR_FLASH_PAGE_31);
      
      return_state_u8 = RUN_MODULE;
      break;
    }
    case RUN_MODULE: {
      /** Program the user Flash area word by word(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) **/
          
 
      


      return_state_u8 = RUN_MODULE;
      break;
    }
    case KILL_MODULE: {
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

/** =========================== private functions =========================================== **/
/**
  * @brief  Erase page/s of flash data
  * @param      drv_id_u8       The function caller module ID in case error occur within this function
  *             pageAddress     The starting address of the flash page
  * @retval successful
  */
uint8_t flashPageErase(uint8_t drv_id_u8, uint32_t pageAddress)                 //pam tested
{
    uint8_t returnValue = true;
    uint32_t PageError = 0;
    /** Unlock the Flash to enable the flash control register access **/
    HAL_FLASH_Unlock();
    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = pageAddress;
    EraseInitStruct.NbPages     = (uint32_t)NumOfPage;
    /**                                                Flash erase                                             **/
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)   //22ms
    {  // Error occurred while page erase.                                                                           
       setupSoftwareIRQ(drv_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, sizeof(uint32_t), (uint8_t*)(&PageError));     //report the error address 
       returnValue = false;
    }
    /** Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) **/
    HAL_FLASH_Lock();    
    return returnValue;
}

/**
  * @brief  Copy page/s of flash data to another sector with all the temporary storage in internal buffer
  * @param      drv_id_u8               The function caller module ID in case error occur within this function
  *             _FrompageAddress        The starting address of the source flash page/s
  *             _TopageAddress          The starting address of the sink flash page/s
  * @retval successful
  */
uint8_t flashPageUpdate(uint8_t drv_id_u8, uint32_t _FrompageAddress, uint32_t _TopageAddress)
{
    /** Unlock the Flash to enable the flash control register access **/
    HAL_FLASH_Unlock();
    uint16_t indx = 0;
    uint16_t currentDat;
    uint16_t returnBuf = 0;
    uint8_t returnValue = TRUE;
    for(  ;  indx < ((FLASH_PAGE_SIZE * NumOfPage) - 2) ; indx +=2)
    {
      if(FlashBufDeRegistered(indx, &returnBuf))  //check this offset address is changed 
      {
        currentDat = returnBuf;         
      }
      else
      {
        currentDat = FlashRead((unsigned char*) _FrompageAddress, indx); //no updated data then just read the old data
      } 
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (_TopageAddress + indx), (uint64_t)currentDat) == HAL_OK) //write 16bit data
      {
        Address += 2;
      }
     else
      {
        // Error occurred while writing data in Flash memory.                                                                 
       setupSoftwareIRQ(drv_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, 0x00, NULL);     //report the error address 
      }  
    }
    // put CRC to the last 2byte
    uint16_t uwCRCValue = Calculate_CRC((uint16_t)((FLASH_PAGE_SIZE * NumOfPage) - 2) , (unsigned char*)_TopageAddress);    
    //put calculated CRC back to the last word of the page
    if (!HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (_TopageAddress + indx), (uint64_t)uwCRCValue) == HAL_OK) //write 16bit data
    {
      // Error occurred while writing data in Flash memory.                                                                 
     setupSoftwareIRQ(drv_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, 0x00, NULL);     //report the error address
     returnValue = FALSE;
    }  
    /** Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) **/
    HAL_FLASH_Lock();
  return returnValue;
}

/**
  * @brief  Program a block of datae into flash with input buffer
  * @param      drv_id_u8               The function caller module ID in case error occur within this function
  *             _TopageAddress          The starting address of the sink flash page/s
  *             _buf                    Buffer of data to burn into flash
  *             _length                 The starting address of the source flash page/s
  * @retval successful
  */
uint8_t flashBlockProgram(uint8_t drv_id_u8, uint32_t _TopageAddress, uint8_t* _buf, uint32_t _length)                  //Pam Tested
{
    uint8_t returnValue = TRUE;
    HAL_FLASH_Unlock();

    for(uint16_t index = 0  ;  index < _length ; index +=2)
    {
      if (!HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (_TopageAddress + index), (uint64_t)((_buf[index+1] << 8) + _buf[index] )) == HAL_OK) //write 16bit data
      {
        // Error occurred while writing data in Flash memory.                                                                 
       setupSoftwareIRQ(drv_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, 0x00, NULL);     //report the error address 
       returnValue = FALSE;
      }  
    }
    /** Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) **/
      HAL_FLASH_Lock();
    return returnValue;
}

    
  /**
  * @brief  Check the active flash content with the last word CRC is valid
  * @param      _FLASH_START_ADDR    CRC sector starting address
  *             _NumOfPage           Number of sector for CRC check (   @Caution each page will contains two bytes of CRC and is accumulated                                             
  *                                                                        for example> page0 CRC= page0, if also has page1 CRC in the last two bytes of page1 will be from page0 to page1 )
  * @retval successful
  */
uint8_t isFlashCRCValid(uint32_t _FLASH_START_ADDR, uint16_t _NumOfPage){               //pam tested
    unsigned char* _pageAddress = (unsigned char*) _FLASH_START_ADDR;
    uint16_t uwCRCValue = Calculate_CRC((uint16_t)((FLASH_PAGE_SIZE * _NumOfPage) - 2) , _pageAddress);   
    uint16_t FlashCRCValue = FlashRead((unsigned char*)_pageAddress, (uint16_t)((FLASH_PAGE_SIZE * NumOfPage) - 2));
    if(uwCRCValue == FlashCRCValue) 
      return(true);
    else
      return (false);
}


/**
  * @brief  Copy page/s of flash memory to another sector
  * @param      drv_id_u8               The function caller module ID in case error occur within this function
  *             _FrompageAddress        The starting address of the source flash page/s
  *             _TopageAddress          The starting address of the sink flash page/s
  * @retval successful
  */
  //copy the whole page to another page only
uint8_t flashPageCopy(uint8_t drv_id_u8, uint32_t _FrompageAddress, uint32_t _TopageAddress)
{
      /** Unlock the Flash to enable the flash control register access **/
    HAL_FLASH_Unlock();
    uint16_t indx = 0;
    uint16_t currentDat;
    for(  ;  indx < (FLASH_PAGE_SIZE * NumOfPage) ; indx +=2)
    {
      currentDat = FlashRead((unsigned char*)_FrompageAddress, indx); 
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (_TopageAddress + indx), (uint64_t)currentDat) == HAL_OK) //write 16bit data
      {
        Address += 2;
      }
     else
      {
        // Error occurred while writing data in Flash memory.                                                                 
       setupSoftwareIRQ(drv_id_u8, MODULE_ERR_LOGHANDLE, 0xE4, 0x00, 0x00, NULL);     //report the error address 
      }  
    }     
    /** Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) **/
    HAL_FLASH_Lock();
  return 0;
}

uint32_t testdat = 0;
/**
  * @brief  Read data from flash
  * @param  pageAddress flash page starting address
  *         offsetByte  offset of the physical flash address (as the offset in byte [caution !!! this is absolute offset address in flash !!!])
  * @retval data (16bit)
  */
uint16_t FlashRead(unsigned char* aDataBuf, uint16_t offsetByte)       //this read a word
{
      return((((uint16_t)aDataBuf[offsetByte]) << 8) +  aDataBuf[offsetByte +1]);
}

/** ----------------------------------------- Flash internal ram storage system ----------------------------------------------------------------- **/

/**
  * @brief  initialize the internal buffer and head, tail pointer
  * @param  None
  * @retval None
  */
void FlashBufInit(void)
{
  for(uint8_t index = 0; index < FlashBufSize ; index++)
  {
    flashBuf[index].offset = 0xffff;
    flashBuf[index].flashDat = 0;
  }  
  Flash_BufHead = 0;
  Flash_BufTail = 0;
}

/**
  * @brief  Empty the setting data according to offset of the setting data
  * @param  _offset     offset of the setting data
  *         _returnBuf  data found 
  * @retval data found successfully = true/false, the data found will store in  _returnBuf
  */
uint16_t FlashBufDeRegistered(uint16_t _offset, uint16_t* _returnBuf)                       //Got the ack from the receiver and de-registered the record 
{
  for(uint8_t index = 0; index < FlashBufSize ; index++)
  {
     if(_offset == flashBuf[index].offset)                           //search the received FrameAckID in buffer
     {  //clear all this record when found
        flashBuf[index].offset = 0xffff;
        *_returnBuf = flashBuf[index].flashDat;                       //return the found value 
        flashBuf[index].flashDat = 0;                                //clear data
        FlashFlushBuf();
        return true;
     }
  }  
  return false;
}

/**
  * @brief  Flush the internal buffer and packed the leading or trailing record is/are empty
  * @param  None
  * @retval None
  */
void FlashFlushBuf(void)                                                           
{ //push back all the trailing empty record, and move the tail back to the last trailing record
  while((Flash_BufHead != Flash_BufTail) && (flashBuf[Flash_BufTail].offset == 0xffff) )    
  { //Move head to the closest in buffer
    if(Flash_BufTail != 0 ){
      Flash_BufTail--;
    }
    else
    {
      Flash_BufTail = (FlashBufSize - 1);
    }
  } 
  //push back all the leading empty record, and move the head back to the first leading record
  while((Flash_BufHead != Flash_BufTail) && (flashBuf[Flash_BufHead].offset == 0xffff) )
  { //Move head to the closest in buffer
    if(++Flash_BufHead == FlashBufSize)  Flash_BufHead = 0;
  }  
}


/**
  * @brief  store current data(16bit offset) into internal buffer
  * @param  _offset: ofset position of the 16bit setting data 
  * @warning [caution !!! not absolute offset address in flash !!! please follow the FlashOffsetIndex ]
  * @retval None
  */
uint8_t FlashDatSet(uint16_t _offset, uint16_t _flashDat)
{
  uint16_t tmpryDat;
  if(!IsFlashBufFull())
  {
    FlashBufDeRegistered((_offset * 2), &tmpryDat); //find this offset has already got data, will be replace    
    if(++Flash_BufTail == FlashBufSize) {
      Flash_BufTail = 0;
    }  
    flashBuf[Flash_BufTail].offset = _offset * 2;               //store as 16bit effective offset as no odd address!!! 
    flashBuf[Flash_BufTail].flashDat = _flashDat;
    return true;
  }
  return false;                         //if return also mean buffer full
}

/**
  * @brief  Internal buffer is full
  * @param  None
  * @retval ture/false
  */
uint8_t IsFlashBufFull(void) 
{
  int16_t result = (int16_t)Flash_BufHead - (int16_t)Flash_BufTail;
  if( (result == -(FlashBufSize-1)) || (result == 1)) 
  { 
    return (true);
  }  
  return (false);
}

uint8_t Reg2Ram(uint32_t _RegNum, uint16_t _Value)
{
  uint8_t stage = 0;
  bool bNoError = FALSE;
  uint16_t Durationms;
  int16_t FinalMecSpeedUnit;
  int16_t FinalTorque;
  switch((FlashOffsetIndex)_RegNum)
  {
    case Index_A_POLE_PAIR_NUM                :     //reg0  _1
      {
        break;
      }
    case Index_A_RS                           :     //reg1  _ 
      {
        break;
      }
    case Index_A_LS                           :     //reg2  _ 
      {
        break;
      }
    case Index_A_NOMINAL_CURRENT              :     //reg3  _1
      {
        break;
      }
    case Index_A_MAX_APPLICATION_SPEED_RPM    :     //reg4  _ 
      {
        break;
      }
    case Index_A_MIN_APPLICATION_SPEED_RPM    :     //reg5  _ 
      {
        break;
      }
    case Index_A_PLL_KP_GAIN                  :     //reg6  _1   
      { //MC_PROTOCOL_REG_PLL_KI,    
        UI_SetReg(&(GetMCP()->_Super), MC_PROTOCOL_REG_PLL_KI, _Value); //tested working
        break;
      }
    case Index_A_PLL_KI_GAIN                  :     //reg7  _1   
      { //MC_PROTOCOL_REG_PLL_KP,  
        UI_SetReg(&(GetMCP()->_Super), MC_PROTOCOL_REG_PLL_KP, _Value);
        break;
      }
    case Index_A_PWM_FREQUENCY                :     //reg8  _ 
      { 
        break;
      }
    case Index_A_PID_TORQUE_KP_DEFAULT        :     //reg9  _1
      { //MC_PROTOCOL_REG_TORQUE_KP
        UI_SetReg(&(GetMCP()->_Super), MC_PROTOCOL_REG_TORQUE_KP, _Value);
        break;
      }
    case Index_A_PID_TORQUE_KI_DEFAULT        :     //reg10  _1
      { //MC_PROTOCOL_REG_TORQUE_KI
        UI_SetReg(&(GetMCP()->_Super), MC_PROTOCOL_REG_TORQUE_KI, _Value);
        break;
      }
    case Index_A_PID_FLUX_KP_DEFAULT          :     //reg11  _1
      { //MC_PROTOCOL_REG_FLUX_KP
        UI_SetReg(&(GetMCP()->_Super), MC_PROTOCOL_REG_FLUX_KP, _Value);
        break;
      }
    case Index_A_PID_FLUX_KI_DEFAULT          :     //reg12  _1
      { //MC_PROTOCOL_REG_FLUX_KI
        UI_SetReg(&(GetMCP()->_Super), MC_PROTOCOL_REG_FLUX_KI, _Value);
        break;
      }
    case Index_A_PID_SPEED_KP_DEFAULT         :     //reg13  _1
      { //MC_PROTOCOL_REG_SPEED_KP
        UI_SetReg(&(GetMCP()->_Super), MC_PROTOCOL_REG_SPEED_KP, _Value);
        break;
      }
    case Index_A_PID_SPEED_KI_DEFAULT         :     //reg14  _1
      { //MC_PROTOCOL_REG_SPEED_KI
        UI_SetReg(&(GetMCP()->_Super), MC_PROTOCOL_REG_SPEED_KI, _Value);
        break;
      }
    case Index_A_IQMAX                        :     //reg15  _1
      {
        break;
      }
    case Index_A_DEFAULT_CONTROL_MODE         :     //reg16  _1 
      { //MC_PROTOCOL_REG_CONTROL_MODE
        UI_SetReg(&(GetMCP()->_Super), MC_PROTOCOL_REG_CONTROL_MODE, _Value);
        break;
      }
    case Index_A_OV_VOLTAGE_THRESHOLD_V       :     //reg17  _
      {
        break;
      }        
    case Index_A_UD_VOLTAGE_THRESHOLD_V       :     //reg18  _
      {
        break;
      }
    case Index_A_OV_TEMPERATURE_THRESHOLD_C   :     //reg19  _
      {
        break;
      }        
    case Index_A_OV_TEMPERATURE_HYSTERESIS_C  :     //reg20  _
      {
        break;
      }
      
    case Index_A_PHASE5_DURATION              :     //reg33  _1
      { //MC_PROTOCOL_CODE_SET_REVUP_DATA
        stage++;
      }
    case Index_A_PHASE4_DURATION              :     //reg30  _1
      { //MC_PROTOCOL_CODE_SET_REVUP_DATA
        stage++;
      } 
    case Index_A_PHASE3_DURATION              :     //reg21  _1
      { //MC_PROTOCOL_CODE_SET_REVUP_DATA      
        stage++;  
      }
    case Index_A_PHASE2_DURATION              :     //reg24  _1
      { //MC_PROTOCOL_CODE_SET_REVUP_DATA
        stage++;
      }
    case Index_A_PHASE1_DURATION              :     //reg27  _1
      { //MC_PROTOCOL_CODE_SET_REVUP_DATA
        //read   ,rpm = (FinalMecSpeedUnit * _RPM) / SPEED_UNIT;
        UI_GetRevupData(&(GetMCP()->_Super), stage, &Durationms, &FinalMecSpeedUnit, &FinalTorque);
        //write
        bNoError = UI_SetRevupData( &(GetMCP()->_Super), stage, _Value, FinalMecSpeedUnit, FinalTorque );
        break;
      }
    
    case Index_A_PHASE5_FINAL_SPEED_UNIT      :     //reg34  _1
      { //MC_PROTOCOL_CODE_SET_REVUP_DATA
        stage++;
      }  
    case Index_A_PHASE4_FINAL_SPEED_UNIT      :     //reg31  _1
      { //MC_PROTOCOL_CODE_SET_REVUP_DATA
        stage++;
      }
    case Index_A_PHASE3_FINAL_SPEED_UNIT      :     //reg28  _1
      { //MC_PROTOCOL_CODE_SET_REVUP_DATA
        stage++;
      }
    case Index_A_PHASE2_FINAL_SPEED_UNIT      :     //reg25  _1
      { //MC_PROTOCOL_CODE_SET_REVUP_DATA
        stage++;
      }  
    case Index_A_PHASE1_FINAL_SPEED_UNIT      :     //reg22  _1
      { //MC_PROTOCOL_CODE_SET_REVUP_DATA
        //read   ,rpm = (FinalMecSpeedUnit * _RPM) / SPEED_UNIT;
        UI_GetRevupData(&(GetMCP()->_Super), stage, &Durationms, &FinalMecSpeedUnit, &FinalTorque);
        //write
        bNoError = UI_SetRevupData( &(GetMCP()->_Super), stage, Durationms, _Value, FinalTorque );     
        break;
      }
    
    case Index_A_PHASE5_FINAL_CURRENT         :     //reg35  _1
      { //MC_PROTOCOL_CODE_SET_REVUP_DATA
        stage++;
      }
    case Index_A_PHASE4_FINAL_CURRENT         :     //reg32  _1
      { //MC_PROTOCOL_CODE_SET_REVUP_DATA
        stage++;
      }
    case Index_A_PHASE3_FINAL_CURRENT         :     //reg29  _1
      { //MC_PROTOCOL_CODE_SET_REVUP_DATA
        stage++;
      }
    case Index_A_PHASE2_FINAL_CURRENT         :     //reg23  _1
      {
        stage++;
      }
    case Index_A_PHASE1_FINAL_CURRENT         :     //reg26  _1
      { //MC_PROTOCOL_CODE_SET_REVUP_DATA
        UI_GetRevupData(&(GetMCP()->_Super), stage, &Durationms, &FinalMecSpeedUnit, &FinalTorque);
        //write
        bNoError = UI_SetRevupData( &(GetMCP()->_Super), stage, Durationms, FinalMecSpeedUnit, _Value );     
        break;
      }

    case Index_A_TRANSITION_DURATION          :     //reg36  _1
      { 
       // VirtualSpeedSensorM1.hTransitionSteps = (int16_t)((uint32_t) ((uint32_t)(A_PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE)) * A_TRANSITION_DURATION/ 1000.0); // TF_REGULATION_RATE
        //REMNG_ExecRamp( pREMNG[M1], StatorCurrent.q, A_TRANSITION_DURATION ); 
        break;
      }
    // below are future configurable data    
    case Index_D_HALL_SENSORS_PLACEMENT       :     //_1
    case Index_D_HALL_PHASE_SHIFT             :     //_1
    case Index_D_M1_ENCODER_PPR               :     //_ 
#if GAIN1 != 0   //pll or cord
    case Index_A_GAIN1                        :     //_ 
    case Index_A_GAIN2                        :     //_ 
#else
    case Index_D_CORD_GAIN1                   :     //_ 
    case Index_D_CORD_GAIN2                   :     //_ 
    case Index_D_CORD_MAX_ACCEL_DPPP          :      //_1
      break;
#endif //GAIN1   //pll or cord

#ifdef _AB_MODULE_MC_STATEMACHINE_H_
    case Index_MIN_COMMANDABLE_SPEED:
      {
        break;
      }
    case Index_MAX_COMMANDABLE_SPEED:
      {
        break;
      }
    case Index_SPEED_UP_RAMP_RATE:
      {
        break;
      }
    case Index_SPEED_DOWN_RAMP_RATE:
      {
        break;
      }
    case Index_SPEED_CONSIDERED_STOPPED: 
      {
        break;
      }
    case Index_MotSpinTimeOut: 
      {
        break;
      }
    case Index_SpinPollPeriod: 
      {
        break;
      }
    case Index_numOfStartRetry: 
      {
        break;
      }
    case Index_StartRetryPeriod: 
      {
        break;
      }
    case Index_StartPeriodInc: 
      {
        break;
      }
    case Index_over_current_threshold: 
      {
        break;
      }
    case Index_over_current_rpm_Reduce: 
      {
        break;
      }
    case Index_OvCurrent_derate_period: 
      {
        break;
      }
    case Index_over_power_threshold: 
      {
        break;
      }
    case Index_over_power_rpm_Reduce: 
      {
        break;
      }
    case Index_OvPower_derate_period:
      {
        break;
      }
    case Index_over_temperature_threshold:
      {
        break;
      }
    case Index_over_temperature_rpm_Reduce: 
      {
        break;
      }
    case Index_OvTemp_derate_period: 
      {
        break;
      }
#endif //_MODULE_MC_STATEMACHINE_H_
  default:
    break;
  }
  return TRUE;
}