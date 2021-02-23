/**
  *************************************************************************************
  * @file    module_analog_0_10v.c 
  * @author  Regal, Satya Akkina
  * @version V1.0
  * @date    17-Aug-2020
  * @brief   module for Analog 0-10V input
  * @note    Provide structures to access Analog setting and Analog volts, demand
  *************************************************************************************
  */
#include "module_analog_0_10v.h"

#include <stdio.h>

#include "driver_usart1.h"
#include "module_usart1.h"

extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];
ADC1_Control*  adc1_LocalControl;
static  Ram_Buf_Handle analog_0_10v_StructMem_u32;
void assign_ModuleMem(void);

uint64_t tt_AnalogTime; // Delay time between init and run module

Analog_0_10V_Control analog_0_10V_Control;

enum {
    INIT_MODULE,
    RUN_MODULE,
    // Ddditional states to be added here as necessary.
    IRQ_MODULE = DEFAULT_IRQ_STATE,
    KILL_MODULE = KILL_APP
};

/**
  ********************************************************************************************************************************
  * @brief   Initilize all analog settings and live data
  * @details Read settings from the RAM and initilize the settings
  * @retval  None
  ********************************************************************************************************************************
*/
void Init_Analog_Setting(void){  
  analog_0_10V_Control.analog_0_10V_Setting.onVolts_u16 = 205;              // Demand turn ON volts in counts of ADC
  analog_0_10V_Control.analog_0_10V_Setting.offVolts_u16 = 164;             // Demand trun OFF volts in counts of ADC. Acts as Hysteresis
  analog_0_10V_Control.analog_0_10V_Setting.maxVolts_u16 = 4096;            // Max acceptable analog volts
  analog_0_10V_Control.analog_0_10V_Setting.maxAdcCounts_u16 = 4096;       // Max output of ADC. 12 bit ADC left sifted by 4
  analog_0_10V_Control.analog_0_10V_Setting.debounceThreshold_u16 = 41;     // Change demand if the change in analog is outside this threshold. In ADC counts
  analog_0_10V_Control.analog_0_10V_Setting.calibratonFactor_u16 = 0;       // Add/Delete this form measured for calibration. In ADC counts
  analog_0_10V_Control.analog_0_10V_Setting.upperHysteresis_u16 = 4055 ;    // Hys volts at the top end of demand. In ADC counts
  analog_0_10V_Control.analog_0_10V_Setting.minDemand_u16 = 13107;          // Min allowed demand. 
  analog_0_10V_Control.analog_0_10V_Setting.maxDemand_u16 = 0xFFFF;         // Maximum allowed demand
  analog_0_10V_Control.analog_0_10V_Setting.minDemandHysteresis_u16 = 500; // Lower end hysteresis if Min_Demand > Off_Volts demand. Counts
 
  analog_0_10V_Control.analog_0_10V_Setting.analogLossVolts_u16 = 820;     // Volts below which the bAnalog_Loss_Enable is set. In ADC counts

  analog_0_10V_Control.analog_0_10V_Setting.is_enableAnalog = true;         // If set to "0", 0V = Max demand 10V= 0 Demand
  analog_0_10V_Control.analog_0_10V_Setting.is_invertAnalog = false;        // If set to "0", 0V = Max demand 10V= 0 Demand
  analog_0_10V_Control.analog_0_10V_Setting.is_analogLossEnable = true;    // Enable loss of analog input if set to "1"
  analog_0_10V_Control.analog_0_10V_Setting.is_analogFailSafeEnable = true; // If set to "1", switch to fail safe demand when loss of anlaog is detected
  analog_0_10V_Control.analog_0_10V_Setting.failSafeDemand_u16  = 0x7FFF;    // If set to "1" use Fail safe demand when loss of analog input is detected //50%
  
  // Initilize live data
  analog_0_10V_Control.analog_0_10V_Data.analogVolts_u16 = 0;               // Current Filtered Analog Volts
  analog_0_10V_Control.analog_0_10V_Data.analogVoltsPrev_u16 = 0;      // Previous measured analog volts used for debounce check
  analog_0_10V_Control.analog_0_10V_Data.analogDemand_u16 = 0;              // Calculated analog demand
  analog_0_10V_Control.analog_0_10V_Data.analogVoltsScaled_u16 = 0;        // Analog Volts scalled 0 to 100%
  analog_0_10V_Control.analog_0_10V_Data.analogDemand_u16 = 0;              // Analog Demand scalled 0 to 100%  
  analog_0_10V_Control.analog_0_10V_Data.increasingIntercept_u16 = 0;    // Intercept used to calculate demand when analog is increasing
  analog_0_10V_Control.analog_0_10V_Data.decreasingIntercept_u16 = 0;    // Intercept used to calcualte demand when analog is decreasing
  analog_0_10V_Control.analog_0_10V_Data.decreasingSlope_f = (float)(analog_0_10V_Control.analog_0_10V_Setting.maxDemand_u16 / ((float)(analog_0_10V_Control.analog_0_10V_Setting.upperHysteresis_u16)));        
                                                 // Slope used to calculate demand when analog is decreasing
  analog_0_10V_Control.analog_0_10V_Data.increasingSlope_f = (float)(analog_0_10V_Control.analog_0_10V_Setting.maxDemand_u16 / ((float)(analog_0_10V_Control.analog_0_10V_Setting.maxVolts_u16)));         
                                                 // Slope used to calculate demand when analog is increasing
                                                 // Assumes 0 V = 0 demand; Max volts = Max demand  
  analog_0_10V_Control.analog_0_10V_Data.is_analogLoss = false;             // Set to "1", if loss of analog is detected
  analog_0_10V_Control.analog_0_10V_Data.is_decreasingAnalog = false;       // Set to "1", if analog voltage is decresing
  analog_0_10V_Control.analog_0_10V_Data.is_lowerHysteresisEnable = false;  // Set to "1", if lower end hys is enabled
  analog_0_10V_Control.analog_0_10V_Data.is_upperHysteresisEnable = false;  // Set to "1", if upper end hys is enabled 
  analog_0_10V_Control.analog_0_10V_Data.is_demandOn = false;               // Set to"1", if demand goes from 0 to above min_Demand_u16.
}


/**
  ********************************************************************************************************************************
  * @brief   Check if Analog is increasing or decresing
  * @details Check for debounce. Only update Analog if change is greater than debounce
             debounce. Check for Loss of Analog
  * @retval  None
  ********************************************************************************************************************************
*/
void CheckAnalogVolts(void){
  // Check if measured value is outside debounce threshold
  if (analog_0_10V_Control.analog_0_10V_Data.analogVoltsPrev_u16  > analog_0_10V_Control.analog_0_10V_Data.analogVolts_u16)
  {
    if ((analog_0_10V_Control.analog_0_10V_Data.analogVoltsPrev_u16  - analog_0_10V_Control.analog_0_10V_Data.analogVolts_u16) < (analog_0_10V_Control.analog_0_10V_Setting.debounceThreshold_u16))
    {  // Change is analog is below debounce
      analog_0_10V_Control.analog_0_10V_Data.analogVolts_u16 = analog_0_10V_Control.analog_0_10V_Data.analogVoltsPrev_u16 ;      
    } else { // Change is analog is above debounce
      analog_0_10V_Control.analog_0_10V_Data.is_decreasingAnalog = true;
    }
  }
  // Check if measured value is outside debounce threshold
  if (analog_0_10V_Control.analog_0_10V_Data.analogVoltsPrev_u16  < analog_0_10V_Control.analog_0_10V_Data.analogVolts_u16)
  {
    if ((analog_0_10V_Control.analog_0_10V_Data.analogVolts_u16 - analog_0_10V_Control.analog_0_10V_Data.analogVoltsPrev_u16 ) < (analog_0_10V_Control.analog_0_10V_Setting.debounceThreshold_u16))
    { // Change is analog is below debounce
      analog_0_10V_Control.analog_0_10V_Data.analogVolts_u16 = analog_0_10V_Control.analog_0_10V_Data.analogVoltsPrev_u16 ;
    } else { 
      // Change is analog is above debounce
      analog_0_10V_Control.analog_0_10V_Data.is_decreasingAnalog = false;
    }
  }
  // Check if loss of analog volts
  if (analog_0_10V_Control.analog_0_10V_Data.analogVolts_u16 < analog_0_10V_Control.analog_0_10V_Setting.analogLossVolts_u16)
  {
    analog_0_10V_Control.analog_0_10V_Data.is_analogLoss = true;
  } else {
    analog_0_10V_Control.analog_0_10V_Data.is_analogLoss = false;
  }
}

/**
  ********************************************************************************************************************************
  * @brief   Convert Analog volts to Demand ( min_Demand_u16 to max_Demand_u16)
  * @details Check for hysteresis, turn ON/OFF and calculate demand 
  * @retval  None
  ********************************************************************************************************************************
*/
uint16_t GetAnalogDemand(void){
  uint32_t analog_demand_u32 = 0;
  float slope_f;
  uint16_t intercept_u16;
  uint16_t analog_volts_u16 = 0; 
  
  analog_volts_u16 = analog_0_10V_Control.analog_0_10V_Data.analogVolts_u16;
  if (analog_0_10V_Control.analog_0_10V_Setting.is_enableAnalog == false)
  {
    analog_demand_u32 = 0;
  } else if( (analog_0_10V_Control.analog_0_10V_Setting.is_analogLossEnable == true) && (analog_0_10V_Control.analog_0_10V_Setting.is_analogFailSafeEnable == true ) && (analog_0_10V_Control.analog_0_10V_Data.is_analogLoss == true)) { 
    // Check if Loss of analog
    if(analog_0_10V_Control.analog_0_10V_Data.is_demandOn == false) //demand is never gone above min demand. 
    {  // Analog never crossed turned ON volts
      analog_demand_u32 = 0;
    } else {
      // Use fail safe demand if loss of analog detected
      analog_demand_u32 =   analog_0_10V_Control.analog_0_10V_Setting.failSafeDemand_u16; 
    }
  } else {
    // If invert is enabled invert measured value
    if (analog_0_10V_Control.analog_0_10V_Setting.is_invertAnalog == true) 
    {
      analog_volts_u16 = analog_0_10V_Control.analog_0_10V_Setting.maxAdcCounts_u16 - analog_0_10V_Control.analog_0_10V_Data.analogVolts_u16;
    }
    
    if (analog_0_10V_Control.analog_0_10V_Data.is_decreasingAnalog == false)
    { // Slope used when analog value is decreasing
      slope_f = analog_0_10V_Control.analog_0_10V_Data.increasingSlope_f;
      intercept_u16 = analog_0_10V_Control.analog_0_10V_Data.increasingIntercept_u16;
    } else {
      // Slope used when analog value is increasing
      slope_f = analog_0_10V_Control.analog_0_10V_Data.decreasingSlope_f;
      intercept_u16 = analog_0_10V_Control.analog_0_10V_Data.decreasingIntercept_u16;
    }
    
    // Analog input between OFF volts and uppwer hysteresis
    if ((analog_volts_u16 >= analog_0_10V_Control.analog_0_10V_Setting.offVolts_u16) && (analog_volts_u16 < analog_0_10V_Control.analog_0_10V_Setting.upperHysteresis_u16))
    {
      // If analog is between ON and OFF volts. Hysteresis part
      // This wont executer if analog never went above On volts.
      if ((analog_0_10V_Control.analog_0_10V_Data.is_lowerHysteresisEnable == true))
      {
        analog_demand_u32 = (uint32_t)((uint64_t)(slope_f * analog_volts_u16) + intercept_u16);
      } else if (analog_volts_u16 >= analog_0_10V_Control.analog_0_10V_Setting.onVolts_u16) {
        // Analog voltage when above ON volts for first time
        analog_0_10V_Control.analog_0_10V_Data.is_lowerHysteresisEnable = true;
        analog_demand_u32 = (uint32_t)((uint64_t)(slope_f * analog_volts_u16) + intercept_u16);
      } else if (analog_volts_u16 < analog_0_10V_Control.analog_0_10V_Setting.onVolts_u16) {
        // Demand is zero when analog never crossed ON volts
        analog_demand_u32 = 0;
      } else {
        analog_demand_u32 = 0;
      }
    } else if (analog_volts_u16 >= analog_0_10V_Control.analog_0_10V_Setting.upperHysteresis_u16) {
      // Analog voltage above upper hysteresis voltage
      // Analog already crossed hysteresis point
      if (analog_0_10V_Control.analog_0_10V_Data.is_upperHysteresisEnable  == true)
      {
        if (analog_0_10V_Control.analog_0_10V_Data.is_decreasingAnalog == true)
        {
          analog_demand_u32 = analog_0_10V_Control.analog_0_10V_Setting.maxDemand_u16;
        } else {
          analog_demand_u32 = (uint32_t)((uint64_t)(slope_f * analog_volts_u16) + intercept_u16);
        }
      } else if (analog_volts_u16 <= analog_0_10V_Control.analog_0_10V_Setting.maxVolts_u16) {
        // Analog went above Upper Hysteresis value for the first time
        analog_0_10V_Control.analog_0_10V_Data.is_upperHysteresisEnable  = true;
        analog_demand_u32 = (uint32_t)((uint64_t)(slope_f * analog_volts_u16) + intercept_u16);
      } else if (analog_volts_u16 > analog_0_10V_Control.analog_0_10V_Setting.maxVolts_u16) {
        // Saturate demand value
        analog_0_10V_Control.analog_0_10V_Data.is_upperHysteresisEnable  = true;
        analog_demand_u32 = analog_0_10V_Control.analog_0_10V_Setting.maxDemand_u16;
      }
    }
    // Min demand is min_Demand_u16.
    if (analog_demand_u32 < analog_0_10V_Control.analog_0_10V_Setting.minDemand_u16)
    {
      analog_0_10V_Control.analog_0_10V_Data.is_demandOn = false;
      if (analog_0_10V_Control.analog_0_10V_Data.is_decreasingAnalog == true)
      {
        if (analog_demand_u32 < (analog_0_10V_Control.analog_0_10V_Setting.minDemand_u16 - analog_0_10V_Control.analog_0_10V_Setting.minDemandHysteresis_u16))
        {
          analog_demand_u32 = 0;
        }
      } else {
        analog_demand_u32 = 0;
      }
    } else {
      analog_0_10V_Control.analog_0_10V_Data.is_demandOn = true; // Demand greater than min demand.       
    }
    // Demand can't exceed max_Demand_u16
    if (analog_demand_u32 > analog_0_10V_Control.analog_0_10V_Setting.maxDemand_u16)
    {
      analog_demand_u32 = (uint16_t)analog_0_10V_Control.analog_0_10V_Setting.maxDemand_u16;
    }      
  }
  analog_0_10V_Control.analog_0_10V_Data.analogDemand_u16 = analog_demand_u32;
  return ((uint16_t)analog_demand_u32);  
}
//void Send_Speed_Msg(uint16_t volts_Percent_u16); // SPA For testing only
//Usart1_Control* usart1Control_AppLocal; // SPA for testing only

/*void Send_Speed_Msg(uint16_t volts_Percent_u16){ // SPA For testing only
  uint16_t MIN_COMMANDABLE_SPEED = 300;
  uint16_t MAX_COMMANDABLE_SPEED = 1500;
  uint16_t motorOnThreadhold = 10;                      // percentage of 0 to 10V for turn on Motor
  //uint16_t motorOffThreadhold = 5;                      // percentage of 0 to 10V for turn on Motor
  uint16_t MotorMaxLimPercent = 90 ;                     // Max motor speed in percentage 90%
  unsigned char speedTx[] = {0x55, 0x02, 0x21, 0x00, 0x00, 0xff, 0xff, 0xCC, 0xCC};
  unsigned int speedLen = sizeof(speedTx);
  
  uint32_t SpeedRatePerADCValue  = ((MAX_COMMANDABLE_SPEED - MIN_COMMANDABLE_SPEED) * 100) / (MotorMaxLimPercent  - motorOnThreadhold); 
  speedTx[5] = (unsigned char) ((((((volts_Percent_u16 -  (motorOnThreadhold*100)) * SpeedRatePerADCValue)/10000) + MIN_COMMANDABLE_SPEED)  & 0xff00) >> 8);
  speedTx[6] = (unsigned char) ((((volts_Percent_u16 -  (motorOnThreadhold*100)) * SpeedRatePerADCValue)/10000) + MIN_COMMANDABLE_SPEED) & 0xff;
  
  
  RingBuf_WriteBlock(((*usart1Control_AppLocal).seqMemTX_u32), speedTx, &speedLen); 
}*/

/**
  ********************************************************************************************************************************
  * @brief   Assign structured memory
  * @details Assign structured memory for Analog 0-10V control
  * @retval  None
  ********************************************************************************************************************************
*/
//
void AssignModuleMem(void){   
  analog_0_10v_StructMem_u32 =  StructMem_CreateInstance(MODULE_ANALOG_0_10V, sizeof(Analog_0_10V_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*analog_0_10v_StructMem_u32).p_ramBuf_u8 = (uint8_t *)&analog_0_10V_Control ;    // Map the ADC1 memory into the structured memory
  uint8_t Module_Analog_0_10v_Index = getProcessInfoIndex(MODULE_ANALOG_0_10V);
  processInfoTable[Module_Analog_0_10v_Index].Sched_ModuleData.p_masterSharedMem_u32 = (Ram_Buf_Handle)analog_0_10v_StructMem_u32;
}

/**
  ********************************************************************************************************************************
  * @brief   State machine for Analog 0-10V module
  * @details
  * @retval  return_state_u8
  ********************************************************************************************************************************
*/
uint8_t moduleAnalog_0_10V_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8,
                               uint8_t irq_id_u8)
{
  uint8_t return_state_u8 = INIT_MODULE;
  switch (next_state_u8)
  {
  case INIT_MODULE:
    {     
      AssignModuleMem(); // Assign structured memory to Analog 0-10V setting and data
      tt_AnalogTime = getSysCount() + AnalogPeriod;    // Store time tick value
      
      Init_Analog_Setting(); // Initilize analog 0-10V settings      
      
      // Get structured memory for ADC1 data
      uint8_t module_ADC1_Index = getProcessInfoIndex(MODULE_ADC1);
      adc1_LocalControl = (ADC1_Control*)((*(processInfoTable[module_ADC1_Index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);
      
      // Get structured memory for USART1 data //SPA For testing only
      //uint8_t Usart1index  = getProcessInfoIndex(DRV_USART1); 
      //usart1Control_AppLocal = (Usart1_Control*)((*(processInfoTable[Usart1index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8); //SPA Review
      
      return_state_u8 = RUN_MODULE;
      break;

    }   
  case RUN_MODULE:                                                             
    {
      // Process analog input every "AnalogPeriod" mSec
      if (getSysCount() >= tt_AnalogTime) 
      {
        //if((*adc1_LocalControl).adc1Result_Avg.ADC1_0_10V_Avg_u16)  //SPA Review. Fail safe wont work if analog voltage is 0V
        //{
          tt_AnalogTime = getSysCount() + AnalogPeriod;   // Update time tick value
          uint32_t temp_result = 0;
          temp_result = ((*adc1_LocalControl).adc1_ResultAvg.adc1_0_10V_Avg_u16 * 10000);
          analog_0_10V_Control.analog_0_10V_Data.analogVoltsScaled_u16 = (uint16_t)(temp_result / ADC12B );  // Convert ADC1 value into 100.00%               
          analog_0_10V_Control.analog_0_10V_Data.analogVoltsPrev_u16 = analog_0_10V_Control.analog_0_10V_Data.analogVolts_u16; // Store previous analog voltage
          analog_0_10V_Control.analog_0_10V_Data.analogVolts_u16 = (*adc1_LocalControl).adc1_ResultAvg.adc1_0_10V_Avg_u16; // Get result from ADC module into analog 0-10V module
          analog_0_10V_Control.analog_0_10V_Data.analogErrorCode_u8 = (*adc1_LocalControl).adc1_ResultAvg.errorCode_u8; // Get any error codes
          CheckAnalogVolts(); // Process the analog voltage
          // Check if Loss of analog. If enabled and loss of analog, demand is "0"
          if((analog_0_10V_Control.analog_0_10V_Data.is_analogLoss == true) && (analog_0_10V_Control.analog_0_10V_Setting.is_analogLossEnable == true)){
            analog_0_10V_Control.analog_0_10V_Data.analogDemand_u16 = 0;    
            analog_0_10V_Control.analog_0_10V_Data.analogScaledDemand_u16 = 0;
          } else { // No loss of analog is detected or loss of analog is not enabled
            analog_0_10V_Control.analog_0_10V_Data.analogDemand_u16 = GetAnalogDemand(); // Convert analog voltage into demand 0 - 0xFFFF
            uint32_t temp_result = 0;
            temp_result = (uint32_t)(analog_0_10V_Control.analog_0_10V_Data.analogDemand_u16)* 10000;
            temp_result = (uint16_t)(temp_result >> 16 ); 
            analog_0_10V_Control.analog_0_10V_Data.analogScaledDemand_u16 = temp_result;  // Convert ADC1 value into 0 to 100.00% 
          }

          // SPA For dubugging
          //printf("%d ",analog_0_10V_Control.analog_0_10V_Data.analogDemand_u16);
          //printf("%d\n",analog_0_10V_Control.analog_0_10V_Data.analogVolts_u16);
          //printf("%d\n",analog_0_10V_Control.analog_0_10V_Data.analogVolts_u16);
          //printf("%d\n",analog_0_10V_Control.analog_0_10V_Data.analogVoltsScaled_u16);
          printf("%d\n",analog_0_10V_Control.analog_0_10V_Data.analogScaledDemand_u16);
          //Send_Speed_Msg(analog_0_10V_Control.analog_0_10V_Data.analogVoltsScaled_u16);
          
//        } else { 
//          // Analog voltage is "0v"
//          Check_Analog_Volts(); // Process the analog voltage to ensure flags are set correctly
//          analog_0_10V_Control.analog_0_10V_Data.analogVoltsScaled_u16 = 0;
//          analog_0_10V_Control.analog_0_10V_Data.analogScaledDemand_u16 = 0;
//          analog_0_10V_Control.analog_0_10V_Data.analogVoltsPrev_u16 = analog_0_10V_Control.analog_0_10V_Data.analogVolts_u16;
//          analog_0_10V_Control.analog_0_10V_Data.analogVolts_u16 = 0;
//          analog_0_10V_Control.analog_0_10V_Data.analogDemand_u16 = 0;
//        }
      }     
      return_state_u8 = RUN_MODULE;
      break;
    }
  case IRQ_MODULE: {
    // If there are more than one interrupts, from different drivers, you can identify each individually by:
    // tableIndex_u8 = getProcessInfoIndex(irq_id_u8);
    // Then use processInfoTable[tableIndex_u8] to tailor your response appropriately.
    return_state_u8 = RUN_MODULE;
    break;
  }
  
  case KILL_MODULE: {
    // Setting processStatus_u8 to PROCESS_STATUS_KILLED prevents the scheduler main loop from calling this module again.
    uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
    if (table_index_u8 != INDEX_NOT_FOUND) {
      processInfoTable[table_index_u8].Sched_DrvData.processStatus_u8 = PROCESS_STATUS_KILLED;
    }
    return_state_u8 = INIT_MODULE;
    break;
  }
  default:
    return_state_u8 = KILL_MODULE; //10; 
    break;
  }
  return return_state_u8;
} 

