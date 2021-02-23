/**
  *************************************************************************************
  * @file    DM_Usart1.c 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    18-Jun-2020
  * @brief   Main Driver Deamon function/s for ADC1 hardware
  * @note    
  *************************************************************************************
  */
//#include "DM_ADC1.h" //SPA
#include "module_adc1.h"

//extern uint32_t getSysTick(void);
/* SysCon handle declaration */
//extern ShareMemory ShMem[totalshMem]; //SPA
//extern ProcessInfo processInfo[]; //SPA
//extern ShareMemory* ADC1ShMem; /SPA

/* Content ---------------------------------------------------------------------------------------------------------------------*/
extern Ram_Buf sharedMemArray[SEQ_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];
extern ADC1_Control adc1_Control; 
static Ram_Buf_Handle adc1_Control_StructMem_u32;

enum AppStates {
  INIT_APP,
  RUN_APP,
  // additional states to be added here as necessary.
  IRQ_APP = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};


void assign_DrvMem(){
  
  adc1_Control_StructMem_u32 =  StructMem_CreateInstance(MODULE_ADC1, sizeof(ADC1_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*adc1_Control_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&adc1_Control ;    //map the ADC1 memory into the structured memory
  uint8_t Drv_ADC1Index = getProcessInfoIndex(MODULE_ADC1);
  processInfoTable[Drv_ADC1Index].Sched_DrvData.p_masterSharedMem_u32 = adc1_Control_StructMem_u32;  
  
  adc1_Control.adc1_Result.adc1_0_10V_Result_u16 = 0;             //clear the ADC1 result
  adc1_Control.adc1_Result.adc1_4_20mA_Result_u16 = 0;            //clear the ADC1 result
  adc1_Control.adc1_Result.adc1_Temp_Result_u16 = 0;       //clear the ADC1 result
  
  adc1_Control.adc1_ResultAvg.adc1_0_10V_Avg_u16 = 0;            //clear the ADC1 Avg result
  adc1_Control.adc1_ResultAvg.adc1_4_20mA_Avg_u16 = 0;           //clear the ADC1 Avg result
  adc1_Control.adc1_ResultAvg.adc1_Temp_Avg_u16 = 0;      //clear the ADC1 Avg result
  adc1_Control.adc1_ResultAvg.errorCode_u8 = 0;
}


uint8_t moduleADC1_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8,
                       uint8_t irq_id_u8)
{
  uint8_t return_state_u8 = INIT_APP;
  //Ram_Buf_Handle adc1_Data_StructMem_u32;
  switch (next_state_u8)
  {
  case INIT_APP:                                                              //Setup Uart2
    {
      assign_DrvMem(); //Assign RAM memory for ADC structures      
      adc1_Init();
      configDma(); //SPA
      
      
      // Find the structured memory for the GPIO driver module, by searching for the GPIO onwer id.
      /*Ram_Buf_Handle this_ram_buf_u32;
      for (uint8_t i = 0; i < TOTAL_NUM_OF_STRUCT_MEM_INSTANCES; i++) {
      this_ram_buf_u32 = &sharedMemArray[i];
      if (RamBuf_GetOwner(this_ram_buf_u32) == drv_id_u8) {
      adc1_Data_StructMem_u32 = &sharedMemArray[i];
    }
    }
      
      // Attach the structured memory to the process's master shared memory.
      uint8_t index = getProcessInfoIndex(drv_id_u8);
      if (index != INDEX_NOT_FOUND) {
      processInfoTable[index].Sched_DrvData.irqState_u8 = PROCESS_STATUS_KILLED;
      processInfoTable[index].Sched_DrvData.p_masterSharedMem_u32 = adc1_Data_StructMem_u32;
      processInfoTable[index].Sched_DrvData.attachedModuleID = MODULE_APP;
    }*///SPA
      // Activate ADC 
      // Perform ADC activation procedure to make it ready to convert. 
      activate_ADC();
      start_ADC1_Conversion();
      
      return_state_u8 = RUN_APP;
      break;
      
    }   
  case RUN_APP:                                                             
    {
      //This Deamon need only execute once than kill this process, in case want to run it again programmer can enable this process again by 
      //set the processStatus parameter to 0x00 and set the nextStage parameter to stage 0 or any stage 
      //******************************************* kill this APP/Deamon example ********************************************************  
      /*uint8_t index = getProcessInfoIndex(drvID);                         //return Process index from processInfo array with the appID
      if(index !=255)
      {
      processInfo[index].DrvDat.processStatus = 0xff;                 //Kill this Deamon after run once
    } */ //SPA  
      //************************************ end of kill this APP/Deamon example ******************************************************* 
      return_state_u8 = KILL_MODULE; //SPA
      break;
    }
  case KILL_MODULE: {
    // The GPIO driver module must only be executed once.
    // Setting processStatus_u8 to PROCESS_STATUS_KILLED prevents the scheduler main loop from calling this module again.
    uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
    if (table_index_u8 != INDEX_NOT_FOUND) {
      processInfoTable[table_index_u8].Sched_DrvData.processStatus_u8 = PROCESS_STATUS_KILLED;
    }
    return_state_u8 = KILL_MODULE;
    break;
  }
  default:
    return_state_u8 = 10; 
  }
  return return_state_u8;
} 

