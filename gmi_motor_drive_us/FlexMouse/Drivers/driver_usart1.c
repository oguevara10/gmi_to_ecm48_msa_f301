/***************************************************************************
***** Set_parity ***********************************************************
*			Called from Modbus module and Testmode to go between even
*     and no parity on the fly. 
*     No action is taken if the present setting matches the spec.
*
*     Input Param is the new value for USART1_CR1 M, PCE and RE bits.
***************************************************************************/
/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "driver_usart1.h"

#include "main.h"

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration  
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  PA12 [PA10]   ------> USART1_DE 
  */
  GPIO_InitStruct.Pin = MODBUS_TX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(MODBUS_TX_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MODBUS_RX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(MODBUS_RX_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MODBUS_EN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(MODBUS_EN_GPIO_Port, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_EnableDEMode(USART1);
  LL_USART_SetDESignalPolarity(USART1, LL_USART_DE_POLARITY_HIGH);
  LL_USART_SetDEAssertionTime(USART1, 0);
  LL_USART_SetDEDeassertionTime(USART1, 0);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void usart1_FlushTX_Buf(){
//  USART1_CR1_TE = 0; //TX off
//  LL_USART_DIRECTION_TX
//  LL_USART_DisableIT_TXE(USART1); //disable Tx interrupt
}

void usart1_FlusRXBuf(){
  
}

void usart1_Enable_RX(){
//  USART1_CR1_RXNEIE = 1; //Turn on RXNE interrupt
//  USART1_CR1_RE = 1;     //Enable receiver
  
}
void enable_TX(){
//  USART1_CR1_RE = 0;     //Diasble receiver
//  USART1_CR1_RXNEIE = 0; //Disable RXE interrupt.
}

void usart1_Init(){
  MX_USART1_UART_Init();
}

void Set_parity(uint16_t spec) @ "program"
{
//    volatile u32 i;
//
//    //REVIEW : VJ rootcause why the entire UART reset sequence is needed for parity switching
//    // and optimize below code.
//    //First disable UART, RE & RXNE interrupt, Re-Initialize DSI
//    USART1_CR1_RE = 0;     //Diasble receiver
//    USART1_CR1_RXNEIE = 0; //Disable RXE interrupt.
//    USART1_CR1_UE = 0;
//    USART1_DSI_Configuration();
//    //Next switch the parity
//    USART1->CR1 &= 0xFFFFF9FF; //Clear only the bits PS and PCE (Parity Control Enable and Parity Selection)
//    USART1->CR1 |= spec;
//    //Next Initialize buffers, flags, and pointers
//    i = USART1->SR; // clear errors
//    i = USART1->DR; // clear RX buffer (dummy read)
//    //Turn RXNE interrupt back on and ensure receiver enabled
//    USART1_CR1_UE = 1;
//    USART1_CR1_RXNEIE = 1; //Turn on RXNE interrupt
//    USART1_CR1_RE = 1;     //Enable receiver
}


/******************* variable for Usart2 TX interrupt **************************/
__IO uint8_t indexTx_uint8_t = 0;
uint8_t ubSizeToSend_uint8_t = 0;
unsigned char* usart1TXbuf;
/*******************************************************************************/
/**
  * @brief  Function called for achieving next TX Byte sending
  * @param  None
  * @retval None
  */
void USART1_TXEmpty_Callback(void)
{
//  /* Fill TDR with a new char */
//  LL_USART_TransmitData8(USART1, usart1TXbuf[indexTx_u8++]);
//  
//  if (indexTx_uint8_t == (ubSizeToSend_uint8_t ))
//  {
//    /* Disable TXE interrupt */
//    LL_USART_DisableIT_TXE(USART2);
//    indexTx_uint8_t = 0;
//  } 
}

/**
  * @brief  Function called at completion of last byte transmission
  * @param  None
  * @retval None
  */
void USART1_CharTransmitComplete_Callback(void)
{
//  if (indexTx == sizeof(ubSizeToSend))
//  {
//    indexTx = 0;

    /* Disable TC interrupt */
//    LL_USART_DisableIT_TC(USART1);
//  }
}

void USART1_CharReception_Callback(){
  
  
}


/**
  * @brief  This function performs Tx process for USAR2
  * @param  
  * @retval 
  */
uint8_t usart1_TxProcess(void)
{
  //disable RX. Since half-duplex
  
}



/**
  * @brief  This function performs CRC calculation on BufSize bytes from input data buffer aDataBuf.
  * @param  BufSize Nb of bytes to be processed for CRC calculation
  * @retval 16-bit CRC value computed on input data buffer
  */
uint16_t Calculate_USART1_CRC(uint8_t BufSize, unsigned char* aDataBuf)
{
  register uint8_t index = 0;
  LL_CRC_ResetCRCCalculationUnit(CRC);
  /* Compute the CRC of Data Buffer array*/
  for (index = 0; index < BufSize ; index++)
  {
    LL_CRC_FeedData8(CRC,aDataBuf[index] );
  }
  /* Return computed CRC value */
  return (LL_CRC_ReadData16(CRC));
}

/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void usart1_Error_Callback(void)
{
  __IO uint32_t isr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USART1_IRQn);

  /* Error handling example :
    - Read USART ISR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  isr_reg = LL_USART_ReadReg(USART1, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    /* case Noise Error flag is raised : ... */
//    LED_Blinking(LED_BLINK_FAST);
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
//    LED_Blinking(LED_BLINK_ERROR);
  }
}


/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  /* Check RXNE flag value in ISR register */
  if (LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1))
  {
    /* RXNE flag will be cleared by reading of RDR register (done in call) */
    /* Call function in charge of handling Character reception */
    USART1_CharReception_Callback();
  }
  
  if (LL_USART_IsEnabledIT_TXE(USART1) && LL_USART_IsActiveFlag_TXE(USART1))
  {
    /* TXE flag will be automatically cleared when writing new data in TDR register */

    /* Call function in charge of handling empty DR => will lead to transmission of next character */
    USART1_TXEmpty_Callback();
  }

  if (LL_USART_IsEnabledIT_TC(USART1) && LL_USART_IsActiveFlag_TC(USART1))
  {
    /* Clear TC flag */
    LL_USART_ClearFlag_TC(USART1);
    /* Call function in charge of handling end of transmission of sent character
       and prepare next charcater transmission */
    USART1_CharTransmitComplete_Callback();
  }
  
  if (LL_USART_IsEnabledIT_ERROR(USART1) && LL_USART_IsActiveFlag_NE(USART1))
  {
    /* Call Error function */
    usart1_Error_Callback();
  }
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}