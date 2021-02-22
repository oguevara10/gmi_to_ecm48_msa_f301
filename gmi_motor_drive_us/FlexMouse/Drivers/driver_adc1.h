/**
  *************************************************************************************
  * @file    D_ADC1.h 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    18-Jun-2020
  * @note    read ADC1 continuously conversion so the sampling clock is slow
  *************************************************************************************
  */
#ifndef _DRV_ADC1_H_
#define _DRV_ADC1_H_
#include "scheduler.h"
#include "structured_memory.h"

#include "stm32g0xx_ll_adc.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_dma.h"

#ifdef __cplusplus
extern "C" {
#endif
  
  /* Setup -----------------------------------------------------------------------------------------------------------------------*/
//static ShareMemory* ADC1ShMem; //SPA
static Ram_Buf_Handle adc1StructMem_u32;

#define NUM_OF_ADC1_CH 3

#define ANALOG_0_10V_Pin LL_GPIO_PIN_0
#define ANALOG_0_10V_GPIO_Port GPIOA
#define ANALOG_4_20MA_Pin LL_GPIO_PIN_1
#define ANALOG_4_20MA_GPIO_Port GPIOA

#define AVERAGING_BUF_SIZE 3 //0 - 3, 4 samples
    
/* Delay between ADC end of calibration and ADC enable.                     */
/* Delay estimation in CPU cycles: Case of ADC enable done                  */
/* immediately after ADC calibration, ADC clock setting slow                */
/* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
/* (CPU clock / ADC clock) is above 32.                                     */
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)
  

/* Definitions of environment analog values */
/* Value of analog reference voltage (Vref+), connected to analog voltage   */
/* supply Vdda (unit: mV).                                                  */
#define VDDA_APPLI                       (3300U)

/* Definitions of data related to this example */
/* Init variable out of expected ADC conversion data range */
#define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//******************* ADC1 Control (inside shared memory) *******************************************************************************************************************************  
//Average Results from ADC1 are stored here
struct ADC1_ResultAvg
{
  uint16_t adc1_0_10V_Avg_u16;             //0-10V ADC1 average counts of last 4 values
  uint16_t adc1_4_20mA_Avg_u16;            //4-20mA ADC1 average counts of last 4 values
  uint16_t adc1_Temp_Avg_u16;       //Micro Temperature average counts of last 4 values
  uint8_t  errorCode_u8; 
};

//Results from ADC1 are stored here by DMA
struct ADC1_Result
{
  uint16_t adc1_0_10V_Result_u16;         //0-10V ADC1 counts
  uint16_t adc1_4_20mA_Result_u16;        //4-20mA ADC1 counts
  uint16_t adc1_Temp_Result_u16;   //Micro Temperature counts 
                  //Error code of this
};

//Main structure used by other modules
typedef struct{
 struct ADC1_Result adc1_Result ;
 struct ADC1_ResultAvg adc1_ResultAvg;  
}ADC1_Control;

//******************* end of ADC1 Control (inside shared memory) ******************************************************************************************************************************* 
  

void activate_ADC(void);
void configDma(void);
void adc1_Init(void);
void start_ADC1_Conversion(void);


#ifdef __cplusplus
}
#endif
#endif
