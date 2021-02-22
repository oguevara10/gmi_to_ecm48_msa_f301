/**
  *****************************************************************************
  * @file    module_analog_0_10v.h 
  * @author  Regal, Satya Akkina
  * @version V1.0
  * @date    17-Aug-2020
  * @brief   Header for Analog 0-10V input
  * @note    
  *****************************************************************************
  */

// Define to prevent recursive inclusion 
#ifndef _MODULE_ANALOG_0_10V_H_
#define _MODULE_ANALOG_0_10V_H_

#ifdef __cplusplus
extern "C" {
#endif
  
// Includes 
#include "driver_adc1.h" 

#include "structured_memory.h"
#include "scheduler.h"
  
extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];
extern Ram_Buf *analog_Input_Settings_StructMem_u32;
extern Ram_Buf *analog_Data_StructMem_u32;

#define ADC12B  4096
#define AnalogPeriod 2000 //Analog voltage Update time mSec

//******************* Analog 0-10V Control (inside shared memory) ************  
// Analog 0-10V settings
struct Analog_0_10V_Settings
{  
  uint16_t       onVolts_u16;               // Demand turn ON volts
  uint16_t       offVolts_u16;              // Demand trun OFF volts. Acts as Hysteresis
  uint16_t       maxVolts_u16;              // Max acceptable analog volts
  uint16_t       maxAdcCounts_u16;         // Max output of ADC
  uint16_t       debounceThreshold_u16;     // Only use if change in analog is outside this threshold
  uint16_t       calibratonFactor_u16;      // Add/Delete this form measured for calibration
  uint16_t       upperHysteresis_u16;       // Hysteresis volts at the top end of demand
  uint16_t       minDemand_u16;             // Min allowed demand
  uint16_t       maxDemand_u16;             // Maximum allowed demand
  uint16_t       minDemandHysteresis_u16;  // Lower end hysteresis if Min_Demand > Off_Volts demand
  uint16_t       analogLossVolts_u16;      // Volts below which the bAnalog_Loss_Enable is set 
  bool           is_enableAnalog;           // If "1" enable analog input for demand
  bool           is_invertAnalog;           // If "1", 0V = Max demand 10V= 0 Demand 
  bool           is_analogLossEnable;      // Enable loss of analog input
  bool           is_analogFailSafeEnable; // If "1" Switch to fail safe demand when loss of anlaog is detected
  uint16_t       failSafeDemand_u16;         // Fail safe demand when loss of analog input is detected
};

// Live Analog Data
struct Analog_0_10V_Data
{  
  uint16_t       analogVolts_u16;            // Analog Volts
  uint16_t       analogVoltsPrev_u16;   // Previous measured analog volts used for debounce check
  uint16_t       analogDemand_u16;           // Calculated analog demand
  uint16_t       analogVoltsScaled_u16;     // Analog Volts scalled 0 to 100%
  uint16_t       analogScaledDemand_u16;    // Analog Demand scalled 0 to 100%
  uint16_t       increasingIntercept_u16;   // Intercept used to calculate demand when analog is increasing
  uint16_t       decreasingIntercept_u16;   // Intercept used to calcualte demand when analog is decreasing
  uint8_t        analogErrorCode_u8;        // Error code  
  float          decreasingSlope_f;         // Slope used to calculate demand when analog is decreasing //SPA REVIEW
  float          increasingSlope_f;         // Slope used to calculate demand when analog is increasing //SPA REVIEW
  bool           is_analogLoss;              // Set to "1" if loss of analog is detected
  bool           is_decreasingAnalog;        // Set to "1" if analog voltage is decreasing
  bool           is_lowerHysteresisEnable;   // Set to "1" if lower end hysteresis need to be enabled
  bool           is_upperHysteresisEnable;   // Set to "1" if upper end hysteresis need to be enabled  
  bool           is_demandOn;                // Set to "1" if demand goes from 0 to above min demand.
};

typedef struct{
 struct Analog_0_10V_Settings analog_0_10V_Setting ;
 struct Analog_0_10V_Data analog_0_10V_Data;  
}Analog_0_10V_Control;
//******* end of Analog 0-10V Control (inside shared memory) ****************

#ifdef __cplusplus
}
#endif

#endif /* _MODULE_ANALOG_0_10V_H_ */

