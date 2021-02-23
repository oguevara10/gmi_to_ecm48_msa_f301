/**
  *************************************************************************************
  * @file    D_ADC1.c 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    18-Jun-2020
  * @brief   Main Driver function/s for ADC1 hardware
  * @note    read ADC1 continuously conversion so the sampling clock is slow
  *************************************************************************************
  */
#include "driver_adc1.h"
#include "stdio.h" //SPA

void MX_ADC1_Init(void); 

//static Ram_Buf_Handle adc1_Control_StructMem_u32;
ADC1_Control adc1_Control; 

extern uint64_t irqRequestReg_u64;
extern ProcessInfo processInfoTable[];

uint8_t ADC1_Digital_filter_index = 0; //
uint16_t ADC1Filter_Ch1[] = {0,0,0,0}; //0-10V input
uint16_t ADC1Filter_Ch2[] = {0,0,0,0}; //4-20mA input
uint16_t ADC1Filter_Ch_Temp[] = {0,0,0,0};

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
    
/**
* @brief ADC1 Initialization Function
* @param None
* @retval None
*/
void MX_ADC1_Init(void)
{
  
    /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_InitTypeDef ADC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC);
  
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration  
  PA0   ------> ADC1_IN0
  PA1   ------> ADC1_IN1 
  */
  GPIO_InitStruct.Pin = ANALOG_0_10V_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ANALOG_0_10V_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ANALOG_4_20MA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ANALOG_4_20MA_GPIO_Port, &GPIO_InitStruct);

  /* ADC1 DMA Init */
  
  /* ADC1 Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_ADC1);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);
  
  //LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&adc1Result); //SPA
 // LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED); //SPA

  /* ADC1 interrupt Init */
  //NVIC_SetPriority(ADC1_IRQn, 2);
  //NVIC_EnableIRQ(ADC1_IRQn);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure Regular Channel 
  */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CHANNEL_TEMPSENSOR
                              );
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED; // LL_ADC_REG_DMA_TRANSFER_LIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  LL_ADC_SetTriggerFrequencyMode(ADC1, LL_ADC_CLOCK_FREQ_MODE_LOW);
  LL_ADC_REG_SetSequencerConfigurable(ADC1, LL_ADC_REG_SEQ_CONFIGURABLE);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_160CYCLES_5);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_2, LL_ADC_SAMPLINGTIME_160CYCLES_5);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);
  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  /** Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_COMMON_1);
  /** Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_COMMON_1);
  /** Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_TEMPSENSOR);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_COMMON_1);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

    
  // USER CODE BEGIN ADC1_Init 2 
  
  //## Configuration of ADC interruptions ####################################
  // Enable interruption ADC group regular end of unitary conversion 
  LL_ADC_EnableIT_EOC(ADC1);
  // USER CODE END ADC1_Init 2 
}

void adc1_Init(){
  MX_ADC1_Init();  
}

void configDma(void)
{
	// Select ADC as DMA transfer request.
	LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_1, LL_DMAMUX_REQ_ADC1);
 
	// DMA transfer addresses and size.
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
	                       LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
	                       (uint32_t)&(adc1_Control.adc1_Result),
	                       LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, NUM_OF_ADC1_CH);
 
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1); // Enable transfer complete interrupt.
	LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1); // Enable half transfer interrupt.
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1); // Enable transfer error interrupt.
 
	// Enable
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

/**
  * @brief  Perform ADC activation procedure to make it ready to convert
  *         (ADC instance: ADC1).
  * @note   Operations:
  *         - ADC instance
  *           - Run ADC self calibration
  *           - Enable ADC
  *         - ADC group regular
  *           none: ADC conversion start-stop to be performed
  *                 after this function
  *         - ADC group injected
  *           Feature not available                                  (feature not available on this STM32 serie)
  * @param  None
  * @retval None
  */
void activate_ADC(void)
{
  __IO uint32_t wait_loop_index = 0U;
  __IO uint32_t backup_setting_adc_dma_transfer = 0U;
  #if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0U; /* Variable used for timeout management */
  #endif /* USE_TIMEOUT */
  
  /*## Operation on ADC hierarchical scope: ADC instance #####################*/
  
  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled.                                              */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADC1);
    
    /* Delay for ADC internal voltage regulator stabilization.                */
    /* Compute number of CPU cycles to wait for, from delay in us.            */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    /* Note: If system core clock frequency is below 200kHz, wait time        */
    /*       is only a few CPU processing cycles.                             */
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }
    
    /* Disable ADC DMA transfer request during calibration */
    /* Note: Specificity of this STM32 serie: Calibration factor is           */
    /*       available in data register and also transfered by DMA.           */
    /*       To not insert ADC calibration factor among ADC conversion data   */
    /*       in DMA destination address, DMA transfer must be disabled during */
    /*       calibration.                                                     */
    backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
    //LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED); //SPA
    /* Run ADC self calibration */
    LL_ADC_StartCalibration(ADC1);
    
    /* Poll for ADC effectively calibrated */
    #if (USE_TIMEOUT == 1)
    Timeout = ADC_CALIBRATION_TIMEOUT_MS;
    #endif /* USE_TIMEOUT */
    
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
    {
    #if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if(Timeout-- == 0)
        {
        /* Time-out occurred. Set LED to blinking mode */
        LED_Blinking(LED_BLINK_ERROR);
        }
      }
    #endif /* USE_TIMEOUT */
    }
    
    /* Delay between ADC end of calibration and ADC enable.                   */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    wait_loop_index = (ADC_DELAY_CALIB_ENABLE_CPU_CYCLES >> 1);
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }
    
    /* Restore ADC DMA transfer request after calibration */
    LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);
    //LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED); //SPA
    
    /* Enable ADC */
    LL_ADC_Enable(ADC1);
    
    /* Poll for ADC ready to convert */
    #if (USE_TIMEOUT == 1)
    Timeout = ADC_ENABLE_TIMEOUT_MS;
    #endif /* USE_TIMEOUT */
    
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
    {
    #if (USE_TIMEOUT == 1)
      /* Check Systick counter flag to decrement the time-out value */
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if(Timeout-- == 0)
        {
        /* Time-out occurred. Set LED to blinking mode */
  //      LED_Blinking(LED_BLINK_ERROR);
        }
      }
    #endif /* USE_TIMEOUT */
    }
    
    /* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
    /*       status afterwards.                                               */
    /*       This flag should be cleared at ADC Deactivation, before a new    */
    /*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
  }
  
  /*## Operation on ADC hierarchical scope: ADC group regular ################*/
  /* Note: No operation on ADC group regular performed here.                  */
  /*       ADC group regular conversions to be performed after this function  */
  /*       using function:                                                    */
  /*       "LL_ADC_REG_StartConversion();"                                    */
  
  /*## Operation on ADC hierarchical scope: ADC group injected ###############*/
  /* Note: Feature not available on this STM32 serie */ 
  
}

void start_ADC1_Conversion(){
  LL_ADC_REG_StartConversion(ADC1);
}
/**
  * @brief  DMA1 IRQ handler
  * @note   This function is executed when the DMA complets
  *         transfer for data from ADC1 to adc1_Control and calculate average
  * @retval None
  */

void DMA1_Channel1_IRQHandler(void)
//void AdcGrpRegularUnitaryConvComplete_Callback(void)
{
  /* Retrieve ADC conversion data */
  /* (data maximum amplitude corresponds to ADC resolution: 12 bits) */
  /* four data sample will pass into a average filter for smooth out the input 0 to 10V */
  //printf("%d",7);
  
  /* Clear ADC End of Conversion Sequence flag. */
  //LL_ADC_ClearFlag_EOS(ADC1);
  //LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, NUM_OF_ADC1_CH);
 
  /* Start conversion. */
  //LL_ADC_REG_StartConversion(ADC1);
 
  /* Wait for completion. */
  //while (!LL_ADC_IsActiveFlag_EOS(ADC1))
  //  ;
 
  
  if(LL_DMA_IsActiveFlag_TC1(DMA1) == 1){ //Check if transfer is complete
    ADC1Filter_Ch1[ADC1_Digital_filter_index & 0x03] = adc1_Control.adc1_Result.adc1_0_10V_Result_u16;//LL_ADC_REG_ReadConversionData12(ADC1);
    ADC1Filter_Ch2[ADC1_Digital_filter_index & 0x03] = adc1_Control.adc1_Result.adc1_4_20mA_Result_u16;
    ADC1Filter_Ch_Temp[ADC1_Digital_filter_index & 0x03] = adc1_Control.adc1_Result.adc1_Temp_Result_u16;
    if( ADC1_Digital_filter_index & 0x3 == 0x3)
    {
      adc1_Control.adc1_ResultAvg.adc1_0_10V_Avg_u16 = (ADC1Filter_Ch1[0] + ADC1Filter_Ch1[1] + ADC1Filter_Ch1[2] + ADC1Filter_Ch1[3]) >> 2;
      adc1_Control.adc1_ResultAvg.adc1_4_20mA_Avg_u16 = (ADC1Filter_Ch2[0] + ADC1Filter_Ch2[1] + ADC1Filter_Ch2[2] + ADC1Filter_Ch2[3]) >> 2;
      adc1_Control.adc1_ResultAvg.adc1_Temp_Avg_u16 = (ADC1Filter_Ch_Temp[0] + ADC1Filter_Ch_Temp[1] + ADC1Filter_Ch_Temp[2] + ADC1Filter_Ch_Temp[3]) >> 2;
    }
    ADC1_Digital_filter_index++;
    //printf("%d\n",adc1Result_Avg.ADC1_0_10V_Avg_u16);
    //printf("%d\n",adc1Result_Avg.ADC1_4_20mA_Avg_u16);
    //printf("%d\n",adc1Result_Avg.ADC1_Temp_Avg_u16);
    LL_DMA_ClearFlag_TC1(DMA1); //Clear transfer complete flag for DMA to get next set of data
  }
  /* Check whether DMA transfer complete caused the DMA interruption */
  if(LL_DMA_IsActiveFlag_TC1(DMA1) == 1)
  {
    /* Clear flag DMA transfer complete */
    LL_DMA_ClearFlag_TC1(DMA1);
    
    /* Call interruption treatment function */
  }
  
  /* Check whether DMA half transfer caused the DMA interruption */
  if(LL_DMA_IsActiveFlag_HT1(DMA1) == 1)
  {
    /* Clear flag DMA half transfer */
    LL_DMA_ClearFlag_HT1(DMA1);
    
    /* Call interruption treatment function */
  }
  
  /* Note: If DMA half transfer is not used, possibility to replace        */
  /*       management of DMA half transfer and transfer complete flags by  */
  /*       DMA global interrupt flag:                                      */
  /* Clear flag DMA global interrupt */
  /* (global interrupt flag: half transfer and transfer complete flags) */
  // LL_DMA_ClearFlag_GI1(DMA1);
  
  /* Check whether DMA transfer error caused the DMA interruption */
  if(LL_DMA_IsActiveFlag_TE1(DMA1) == 1)
  {
    /* Clear flag DMA transfer error */
    LL_DMA_ClearFlag_TE1(DMA1);
  }
}

