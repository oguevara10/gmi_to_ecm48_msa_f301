/**
  ********************************************************************************************************************************
  * @file    drv_usart2.c 
  * @author  Pamela Lee
  * @brief   Main Driver function/s for serial protocol with Usart2 hardware
  * @details Protocol Usart2, after decode a whole valid frame from serial port2,
  *          trigger the system control to execute the relative APP in the int stage
  *          the Rx data is in usart2SeqMemRX_u32.
  *          To Transmitt data : put data into usart2SeqMemTX_u32, and call this function
  *                              USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "driver_usart2.h"
#include "module_AutoAck.h"

#include "main.h"
#include "ui_task.h" // !error TODO: Remove, only for Regal_SendDebugMessage testing

/* Private variables ---------------------------------------------------------*/
//UART_HandleTypeDef huart2; //SPA

Usart2_Control *usart2Control;

AutoAck_Control *autoAckControl_u32;             

__IO ITStatus UartReady = RESET;
uint8_t usart2CaptureLen;                     //default Universal Protocol header length
//uint8_t counter = 0;
uint16_t uwCRCValue = 0;

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
{

   /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  
//  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
//  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration  
  PB6   ------> USART2_TX
  PB7   ------> USART2_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);



  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_DisableIT_CTS(USART1);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);

  /* USER CODE BEGIN WKUPType USART2 */

  /* USER CODE END WKUPType USART2 */
  LL_USART_ClearFlag_ORE(USART1);               //reset all usart error bit
  /* Polling USART2 initialisation */
 while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
 {
 }
  LL_USART_EnableIT_RXNE(USART1);
  /* USER CODE BEGIN USART2_Init 2 */


  /* USER CODE END USART2_Init 2 */

}


void usart2_Init(){
  MX_USART2_UART_Init();
}


/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART2_CharReception_Callback(void)
{
  /*
  __IO uint32_t received_char;

  // Read Received character. RXNE flag is cleared by reading of RDR register //
  received_char = LL_USART_ReceiveData8(USART2);
  */
 
  unsigned char rxDat2;
  rxDat2 = LL_USART_ReceiveData8(USART1);

  RingBuf_WriteCharacter((*usart2Control).seqMem_InternalPipe_u32,&rxDat2);
  //RBWrite(usart2InternalSeqMem_u32->systemInstanceIndex_u8,&rxDat2); 
  
  /* Echo received character on TX */
 // LL_USART_TransmitData8(USART2, received_char);
}

/******************* variable for Usart2 TX interrupt **************************/
__IO uint8_t indexTx = 0;
uint8_t ubSizeToSend = 0;
unsigned char* wholeFramebuf;
/*******************************************************************************/
/**
  * @brief  Function called for achieving next TX Byte sending
  * @param  None
  * @retval None
  */
void USART2_TXEmpty_Callback(void)
{
  /* Fill TDR with a new char */
  LL_USART_TransmitData8(USART1, wholeFramebuf[indexTx++]);
  
  if (indexTx == (ubSizeToSend ))
  {
    /* Disable TXE interrupt */
    LL_USART_DisableIT_TXE(USART1);
    indexTx = 0;
  } 
}

/**
  * @brief  Function called at completion of last byte transmission
  * @param  None
  * @retval None
  */
void USART2_CharTransmitComplete_Callback(void)
{
//  if (indexTx == sizeof(ubSizeToSend))
//  {
//    indexTx = 0;

    /* Disable TC interrupt */
//    LL_USART_DisableIT_TC(USART2);
//  }
}


static uint8_t dataLen = 0;                                         //length of data byte/s plus CRC expected
uint8_t UniProtocolState = protocolstart;
unsigned char* protocolBuf ;


/**
  *@brief   Function for decode a valid frame of universal protocol
  *         the Rx data will put into an internal pipe and process after 
  *         the first 5 byte contain sync byte and then wait for the whole header 
  *         bytes in side the internal pipe, then perform the header veification.
  *         If the the header bytes are valid, then prepare for getting the whole data fields 
  *         and perform CRC check.
  * @param  None
  * @retval None
  */
void protocolHeaderfetch(void)
{
//  usart2SeqMemRX_u32 = (*usart2Control).seqMemRX_u32;   //pam bug without
  switch(UniProtocolState)
  {
    case  protocolstart:
     Reentryprotocalstart: 
      { // assume this is very beginning of universal-frame, check and clear any garble before sync byte
        usart2CaptureLen = UniHeaderlen; 
        dataLen = 0;
        unsigned int SyncPosition = RingBuf_Search((*usart2Control).seqMem_InternalPipe_u32, RxSyncChr, 0);              //search sync byte within internal pipe,[return position or 0xf000 == not found 0xffff == error]
        if (SyncPosition != 0) 
        {
          if(SyncPosition & 0xf000)                                                            
          { //sync byte not found or error, then clear the first 5 bytes
            SyncPosition = UniHeaderlen;                                                                // this will clear all 5 byte                                       
          }
          if((protocolBuf = (unsigned char*) realloc(protocolBuf,SyncPosition)) == NULL) reallocError++;
          RingBuf_ReadBlock((*usart2Control).seqMem_InternalPipe_u32, protocolBuf, &SyncPosition);                          //truncate any char before SYNC char 
          if((protocolBuf = (unsigned char*)realloc(protocolBuf,1)) == NULL) reallocError++;
          usart2CaptureLen = UniHeaderlen; 
          UniProtocolState = protocolstart;                                                             // still to back here next stage for complete header frame 
          break;
        }
        // otherwise mean the the first byte in buffer is sync byte, then can directly process to header Validate state !!!!!!!!!
        UniProtocolState = protocolstart;
      }
    case headerValidate:
      { //check this is a valid header frame in internal pipe
        unsigned char headerBuf[]={ 0, 0, 0, 0, 0, 0, 0, 0, 0};                 //9 byte array  for header including advanced CMD 
        unsigned int headerLen = 9;     
        RingBuf_Observe((*usart2Control).seqMem_InternalPipe_u32, headerBuf, 0, &headerLen);      //get the pipe data without clear internal pipe, headerLen will return number of bytes in pipe if less than 9 byte

        uint8_t dataA ;
        uint8_t headerCRCLen = 7;                                               //default normal frame header + CRC bytes = 7 byte
        uint8_t dataC = (uint8_t) headerBuf[1] & 0xC0;
        if (dataC == 0)                                                         //check this is advanced frame [00]+[data length]
        {
          dataA = (uint8_t)headerBuf[4] & 0xf0 ;                                //this is normal frame, store dataA as the expected Header-end in byte 5
        }
        else
        {  //this is advanced frame [11]+[data length]   or  garbage 
          if (dataC == 0xC0)
          { //this is advanced frame 
            dataA = (uint8_t)headerBuf[6] & 0xf0 ;                              //this is Advanced frame,  store dataA as the expected Header-end in byte 7 
            headerCRCLen = 9;                                                   //header + CRC bytes = 9 byte
          }
          else
          { //this is garbage, because the frame type pointer not valid
            unsigned char tmpbuf3;
            unsigned int truncateLen = 1;   
            RingBuf_ReadBlock((*usart2Control).seqMem_InternalPipe_u32, &tmpbuf3, &truncateLen);                             //truncate the first byte incase the garbage is the same of sync byte
            usart2CaptureLen = UniHeaderlen; 
            UniProtocolState = protocolstart;    
            if( *RingBuf_GetPointerToRingBuf((*usart2Control).seqMem_InternalPipe_u32) >= usart2CaptureLen)                     //after truncase the carbage byte still contain more then header length then goto back to do again
            {
              goto Reentryprotocalstart;                                                                 //this line needed to use goto for finish the frame check in one go
            }
            break;   
          }
        }
        if(dataA == (~((uint8_t)RxSyncChr) & 0xf0))                                  //check header-end byte is valid
        {// this is a valid header        
          dataLen = (uint8_t) headerBuf[1] & 0x3F;     
          usart2CaptureLen = (uint8_t)(dataLen + headerCRCLen) ; // header+ CRC +data length = number of bytes for next capture
          UniProtocolState = frameCRC;
          break;
        }
        else
        {
         // if the header frame is not valid, then delete the first byte only
          unsigned char tmpbuf3;
          unsigned int truncateLen = 1;   
          RingBuf_ReadBlock((*usart2Control).seqMem_InternalPipe_u32, &tmpbuf3, &truncateLen);                             //truncate the first byte incase the garbage is the same of sync byte
          usart2CaptureLen = UniHeaderlen; 
          UniProtocolState = protocolstart;    
          if( *RingBuf_GetPointerToRingBuf((*usart2Control).seqMem_InternalPipe_u32) >= usart2CaptureLen)                     //after truncase the carbage byte still contain more then header length then goto back to do again
          {
            goto Reentryprotocalstart;                  //this line needed to use goto for finish the frame check in one go
          }
          break;
        }
      }
    case frameCRC:
      {
        if((protocolBuf = (unsigned char*) realloc(protocolBuf,usart2CaptureLen)) == NULL) reallocError++;        
        unsigned int DataLen2 = (unsigned int)usart2CaptureLen;
        RingBuf_Observe((*usart2Control).seqMem_InternalPipe_u32, protocolBuf, 0, &DataLen2);                             //copy the whole frame into buffer
        uint16_t frameCRC = (((uint16_t)protocolBuf[DataLen2 - 2]) << 8) + ((uint16_t)protocolBuf[DataLen2 - 1]) ;
        
        uwCRCValue = Calculate_CRC((DataLen2 - 2) , protocolBuf);                                       //Get calculated CRC of this frame
        if(uwCRCValue == frameCRC)
        { //CRC match put whole frame into RX pipe      
          //decode from the CMD of group(1-2, 3 or 4 )
         // RingBuf_WriteBlock((*usart2Control).seqMemRX_u32, protocolBuf, &DataLen2);                                //put fully checked (Valid) Rx frame into Rx Pipe
          if((protocolBuf[3]) && (protocolBuf[2] != 0x3f))                                              //if auto ACK
          {
            unsigned char ackTx[] = {0x55, 0x00, 0x3F, protocolBuf[3], 0x00, 0xCC, 0xCC};
            unsigned int TxLen = sizeof(ackTx);
            RingBuf_WriteBlock((*usart2Control).seqMemTX_u32, ackTx, &TxLen); 
          }          
          // RPa: to signal that there is valid packet
          Set_ValidRx();
          
          switch(protocolBuf[2] & 0xf0)
          {     //groups decoding  please refer to the document of Universal protocol 
            case 0x00:          //group 1
            case 0x10:          //group 1
            case 0x20:          //group 2
            case 0x30:          //group 2
              {    
                if(protocolBuf[2] == 0x3f)
                {
                  //Auto-Ack feedback from receiver
                  AckDeRegistered((uint8_t)protocolBuf[3]);  /** Pam!!!! this is the only function linking from outside driver_Usart2.c can be replace by using structured-memory for loose coupling **/ 
 //                 (*autoAckControl_u32).RxAckFrameID = (uint8_t)protocolBuf[3];
                }
                else
                {       //group1_2 as module_ShortCmd
                  RingBuf_WriteBlock((*usart2Control).seqMemRXG1_2_u32, protocolBuf, &DataLen2);                                //put fully checked (Valid) Rx frame into Rx Pipe
                }
                break;
              }
            case 0x40:          //group 3
            case 0x50:          //group 3
            case 0x60:            //group 3
              {         //group3 module_ReplyCMD
                RingBuf_WriteBlock((*usart2Control).seqMemRXG3_u32, protocolBuf, &DataLen2);                                //put fully checked (Valid) Rx frame into Rx Pipe
                break;
              }
          
            case 0x70:            //group 4
              {        //group 4 module_FlashCMD 
                if(protocolBuf[2] & 0x08)
                {
                   RingBuf_WriteBlock((*usart2Control).seqMemRXG4H_u32, protocolBuf, &DataLen2);                                //put fully checked (Valid) Rx frame into Rx Pipe
                }
                else
                {
                  RingBuf_WriteBlock((*usart2Control).seqMemRXG4L_u32, protocolBuf, &DataLen2);                                //put fully checked (Valid) Rx frame into Rx Pipe
                }
                break;
              }
            default:              // the rest will be truncase
              {   
                break;
              }
          }
          
          /*********** this part only for testing the Tx message *************/
          /*
          //Echo back the full valid frame to sender
          protocolBuf[DataLen2 - 2] = 0xCC;                                                             //Tx frame always use 0xCC as CRC byte, the Tx sending routine should process it with the final CRC value
          protocolBuf[DataLen2 - 1] = 0xCC;                                                             //Tx frame always use 0xCC as CRC byte, the Tx sending routine should process it with the final CRC value
          protocolBuf[0] = 0x55;                                                                           //Tx frame always use 0x55 as Sync, the Tx sending routine should process it for Master or Slave
          if(protocolBuf[1] & 0xC0)
          {//Advanced frame
            protocolBuf[5] = 0x00;                                                                         //Tx frame always use 0xFF as Source Frame ID for ACK require, and 0x00 as non-ACK frame, the Tx sending routine should process with the value for ACK frame        
            protocolBuf[6] = 0x01;                                                                         //Tx frame slways put motor address to lower nibble, and leave upper nibble as 0 for Tx routine to put the Header-end value to this field
          }
          else
          {//Normal frame
            protocolBuf[3] = 0x00;                                                                         //Tx frame always use 0xFF as Source Frame ID for ACK require, and 0x00 as non-ACK frame, the Tx sending routine should process with the value for ACK frame        
            protocolBuf[4] = 0x01;                                                                         //Tx frame slways put motor address to lower nibble, and leave upper nibble as 0 for Tx routine to put the Header-end value to this field
          }
          RingBuf_WriteBlock(usart2SeqMemTX_u32->systemInstanceIndex_u8, protocolBuf, &DataLen2);        
*/
          /*********** this part only for testing the Tx message End End End *************/
        }                                
        else
        { 
          DataLen2 = UniHeaderlen;                                                                   //only truncase the header         
        }                                                                  

        RingBuf_ReadBlock((*usart2Control).seqMem_InternalPipe_u32, protocolBuf, &DataLen2);                              //extract the whole frame  
#if ENABLE_DEBUG_OUTPUT_VIA_STMC_PORT >= 1
        Regal_SendDebugMessage(protocolBuf, DataLen2); // !error
#endif
        if((protocolBuf = (unsigned char*) realloc(protocolBuf,1)) == NULL) reallocError++;                                          //free heap only leave 1 byte 
                
        usart2CaptureLen = UniHeaderlen; 
        UniProtocolState = protocolstart;    
        break;
      }

    default:
      {
        usart2CaptureLen = UniHeaderlen; 
        dataLen = 0;
        UniProtocolState = protocolstart;
        break;
      } 
    
  }

}


/**
  *@brief   Universal protocol transmit
  * @ usage  To send a frame out user should setup a frame buffer
  *          always start with a 0x55 (no matter Master or slave) 
  *            |    count how many data length
  *            |     |   Command of this frame                 
  *            |     |     |   FrameID as 0x00 = non-ack frame or 0xff = auto ack frame
  *            |     |     |     |    put the motor addres for the low 4bits and always 0x0 for the upper 4 bits 
  *            |     |     |     |     |   data for this frame or omit these two byte if no data
  *            |     |     |     |     |     |     |   if this is Auto ack frame put the module ID here, otherwise always leave it as 0xCC
  *            |     |     |     |     |     |     |     |    Always leave as 0xCC
  *            |     |     |     |     |     |     |     |     |
  *          0x55, 0x02, 0x40, 0x00, 0x00, 0xff, 0xff, 0xCC, 0xCC
  *          Then put them into Tx sequential-memory of Usart2, the system will add all the correct sync byte and CRC.
  *          User can put as many frame as the Tx sequential-memory is not full, so please check the Tx sequential-memory status before append new frame.
  * @param  None
  * @retval None
  */
unsigned char* headerFramebuf;

uint8_t TxProcess(void)
{ //process Tx frame in Tx pipe
  int txHeaderSize = UniHeaderlen;
  uint8_t currentFrameID = 0;
  if((headerFramebuf = (unsigned char*) realloc(headerFramebuf, txHeaderSize)) == NULL) reallocError++;
  unsigned int DataLenTx = (unsigned int)txHeaderSize;
  if(indexTx == 0)
  {
    unsigned int HeaderSize = 0;
    if(RingBuf_GetUsedNumOfElements((*usart2Control).seqMemTX_u32) >= txHeaderSize)
    {
      RingBuf_Observe((*usart2Control).seqMemTX_u32, headerFramebuf, 0, &DataLenTx);                //copy the whole header frame into buffer
      if(headerFramebuf[1] & 0xc0) 
      {
        HeaderSize = 9; //Advanced frame +CRC
      }
      else
      {
        HeaderSize = 7; //normal frame +CRC                                                 //put back the lastest frameID into the Tx frame 
      }
      DataLenTx = HeaderSize + ((unsigned int)(headerFramebuf[1] & 0x3F)); 
      if(headerFramebuf[HeaderSize - 4]) 
      { /**check this is auto-ack frame  **/
        if(!IsAckBufFull())
        {
          if((headerFramebuf = (unsigned char*) realloc(headerFramebuf, DataLenTx)) == NULL) reallocError++;       
          RingBuf_Observe((*usart2Control).seqMemTX_u32, headerFramebuf, 0, &DataLenTx);                      //get the whole frame to capture the second last byte of module ID data in the frame
          currentFrameID = AckDatSet((uint8_t)headerFramebuf[2], 0, (uint8_t)headerFramebuf[DataLenTx - 2]);  //put the current Auto Ack frame data to UniversalAckInfo for timout or auto dismiss  
        }
        else
        { //if this is auto ack frame but autoACk UniversalAckInfo is full
          //then wait until the buffer clear 
          /**!!!!!!!!!!!!!!!!!! The communication may slow down alot if completely no reply by the other end, need to be carefull !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!**/
          if((headerFramebuf = (unsigned char*) realloc(headerFramebuf, 1)) == NULL) reallocError++;                            //free heap only leave 1 byte 
          return 1;               //Tx buffer full can't accept anymore frame
        }
      }
      if((headerFramebuf = (unsigned char*) realloc(headerFramebuf, 1)) == NULL) reallocError++;                            //free heap only leave 1 byte 
      if((wholeFramebuf = (unsigned char*) realloc(wholeFramebuf, DataLenTx)) == NULL) reallocError++;       
      RingBuf_ReadBlock((*usart2Control).seqMemTX_u32, wholeFramebuf, &DataLenTx);             //copy the complete frame into buffer
      wholeFramebuf[HeaderSize - 3] = (~TxSyncChr & 0xf0) + (wholeFramebuf[HeaderSize - 3] & 0xf); 
      wholeFramebuf[HeaderSize - 4] = currentFrameID; 
    }   
    wholeFramebuf[0] = TxSyncChr;
    uwCRCValue = Calculate_CRC((DataLenTx - 2) , wholeFramebuf);    
    wholeFramebuf[DataLenTx - 2] = (unsigned char)((uwCRCValue & 0xff00) >> 8);                  //put calculated CRC back into Tx frame
    wholeFramebuf[DataLenTx - 1] = (unsigned char)(uwCRCValue & 0xff) ;                          //put calculated CRC back into Tx frame  
  }    
  
  if((!indexTx) && (LL_USART_IsActiveFlag_TXE(USART1)))
  {
    ubSizeToSend = DataLenTx;      //set TX length                                                      
    /* Start USART transmission : Will initiate TXE interrupt after TDR register is empty */
    LL_USART_TransmitData8(USART1, wholeFramebuf[indexTx++]);                                       //put buffer in
    /* Enable TXE interrupt */
    LL_USART_EnableIT_TXE(USART1);
  }
  /**********************for TX interrupt still using this variable , so don't free it!!!!!!!*******/
  //wholeFramebuf = (unsigned char*) malloc(1);   //for 
  return 0;
}




/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void Error_Callback(void)
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
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  /* Check RXNE flag value in ISR register */
  if (LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1))
  {
    /* RXNE flag will be cleared by reading of RDR register (done in call) */
    /* Call function in charge of handling Character reception */
    USART2_CharReception_Callback();
  }
  
  if (LL_USART_IsEnabledIT_TXE(USART1) && LL_USART_IsActiveFlag_TXE(USART1))
  {
    /* TXE flag will be automatically cleared when writing new data in TDR register */

    /* Call function in charge of handling empty DR => will lead to transmission of next character */
    USART2_TXEmpty_Callback();
  }

  if (LL_USART_IsEnabledIT_TC(USART1) && LL_USART_IsActiveFlag_TC(USART1))
  {
    /* Clear TC flag */
    LL_USART_ClearFlag_TC(USART1);
    /* Call function in charge of handling end of transmission of sent character
       and prepare next charcater transmission */
    USART2_CharTransmitComplete_Callback();
  }
  
  if (LL_USART_IsEnabledIT_ERROR(USART1) && LL_USART_IsActiveFlag_NE(USART1))
  {
    /* Call Error function */
    Error_Callback();
  }
  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}


