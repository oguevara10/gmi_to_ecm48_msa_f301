/***************************************************************************
****** Endura Platform (Crosstour) ********************************************
*
*  File:    Modbus_SL.c   Modbus processing routines.
*                   
*
*       !!!!!   REGAL BELOIT COMPANY PROPRIETARY !!!!!
*
*  Author:  B.Beifus, Vijay G
*  Toolchain: V5.20.2.31007
*  Target:	STMicro STM32F103/101
****************************************************************************
****** REVLOG **************************************************************
*   Created 12-1-14
*   
*
****** DO LIST *************************************************************
* Next:
*   
* Completed:
***************************************************************************
* General Notes:
*  
*    
*
***************************************************************************/
//#include "stm32f10x_lib.h"
//#include "Const_SL.h"
//#include "Externs_SL.h"
//#include "IoDef_SL.h"
//#include "Cmand_SL.h"
//#include "Diag_SL.h"
//#include "IIC_SL.h"
//#include "SafetyVars_SL.h"

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "driver_usart1.h"
#include "module_modbus.h"
#include "main.h"
/***************************************************************************/
#if REMOVE_MODBUS == 0
    #define BROADCAST (0)
    /*   Module Constant Macros Definitions */
    #define MAX_SLAVE_ADDRESS 247
    #define MAX_COILS_MODBUS (2000)
    #define MAX_COILS_QUANTITY (MAX_COILS_MODBUS)
    #define MIN_COILS_QUANTITY (1)

    #define MAX_DISCRETE_MODBUS (224) // Tx Buffer 32bytes so 28 bytes * 8 = 224 Registers
    #define MAX_DISCRETE_QUANTITY (MAX_DISCRETE_MODBUS)
    #define MIN_DISCRETE_QUANTITY (1)

    #define MAX_INPUTS_MODBUS (14) // 125 -> As Per ModBus Standard // 124 // Do not increase this!! - buffer limitation

    #define MAX_INPUTS_QUANTITY (MAX_INPUTS_MODBUS)
    #define MIN_INPUTS_QUANTITY (1)

    #define MAX_HOLDING_READ_MODBUS (14) // 125 -> As Per ModBus Standard // (124) // limited by USB stack limit
    #define MAX_HOLDING_WRITE_MODBUS (14)
    #define MAX_HOLDING_READ_QUANTITY (MAX_HOLDING_READ_MODBUS)
    #define MAX_HOLDING_WRITE_QUANTITY (MAX_HOLDING_WRITE_MODBUS)
    #define MIN_HOLDING_QUANTITY (1)

    #define MAX_REG_ADDRESS (0xFFFF)
    #define MIN_PDU_SIZE (5)

    #define MODBUS_SET (0xFF00)
    #define MODBUS_CLEAR (0x0000)

    #define BYTE_COUNT_USED (1) // used to conditionally compile code to check for  byte count in Function Code 16

/* Module Constant Array Definitions */
const uint16_t MB_ParityValues[MB_NUM_PARITY_SETT] = {MB_PARITY_NONE, MB_PARITY_EVEN,
                                                 MB_PARITY_ODD}; // parity values as written to USART
//const uint16_t MB_BaudRateValues[MB_NUM_BAUDRATES] = {
//    BR_4800_64, BR_1200_64, BR_2400_64,
//    BR_4800_64, BR_9600_64, BR_19200_64}; // note: item[0] is not part of protocol and is a placeholder

/* Module Variable Definitions */
TS_ErrorCounters mb_sErrorCounters;
uint16_t mb_u16ResponsePDUSize;
uint8_t *mb_pau8PDU;
//-------------------
// mb_ctrl_def mbcc;       		//Command control flags and counters (was cc)
mb_msg_ctrl_def mb_msgCntrl;         //Message control ptr and flags (was mc)
uint8_t mb_resp_numBytes;                 //Number of bytes in reponse
uint8_t mb_resp_pos;                      //first byte postion in mb_resp_parm[]
union mb_resp_parm_def mb_resp_parm; //Holds Parameters for response
uint16_t mb_comSetUpdate;
uint16_t mb_blockComSetUpdate;
uint16_t mb_u16BaudRate;
uint16_t mb_u16Parity;
uint16_t mb_delayUpdate;
uint8_t mb_comSetDelay;

/* Module Function Definitions */
void (*m_pfCallBackMB)(TE_ExceptionCode);

void MB_Process_FC01(uint8_t *au8PDU, TS_Locals *psLocals);
void MB_Process_FC02(uint8_t *au8PDU, TS_Locals *sLocals);
void MB_Process_FC03(uint8_t *au8PDU, TS_Locals *sLocals);
void MB_Process_FC04(uint8_t *au8PDU, TS_Locals *sLocals);
void MB_Process_FC05(uint8_t *au8PDU, TS_Locals *sLocals);
void MB_Process_FC06(uint8_t *au8PDU, TS_Locals *sLocals);
void MB_Process_FC16(uint8_t *au8PDU, TS_Locals *sLocals);

void MB_CallBackActionCompleted(TE_ExceptionCode eMBStatus);
uint16_t MB_DataHandle_GetHoldingData(uint16_t u16RegAddress);
TE_ExceptionCode MB_DataHandle_SetHoldingData(uint16_t u16RegAddress, uint16_t u16Data, void (*pfCallBackMB)(TE_ExceptionCode));
uint16_t MB_DataHandle_GetInputData(uint16_t u16RegAddress);
TE_ExceptionCode MB_DataHandle_SetCoilData(uint16_t u16RegAddress, uint16_t u16Data, void (*pfCallBackMB)(TE_ExceptionCode));
uint16_t MB_DataHandle_GetCoilData(uint16_t u16RegAddress);
uint16_t MB_DataHandle_GetDiscreteData(uint16_t u16RegAddress);



Modbus_Control modbus_Control;

void init_Modbus_Settings(void);


/**
  ********************************************************************************************************************************
  * @brief   Update modbus settings
  * @details Update parity, stop bits, baud rate and flow control
  * @retval  None
  ********************************************************************************************************************************
*/
void init_Modbus_Settings(){
  modbus_Control.modbus_Settings.baudRate_u16 = 9600;        //4800, 9600, 14400, 19200, 38400, 57600, 115200
  modbus_Control.modbus_Settings.data_Bits_u8 = 8;         // 8 or 9
  modbus_Control.modbus_Settings.parity_u8 = 0;            //1= Odd; 2 = Even; 0 = None;
  modbus_Control.modbus_Settings.stop_Bits_u8 = 1;         //1 or 2
  modbus_Control.modbus_Settings.flow_Control = 0;         //0 = None; 1= Xon/Xoff; 2 = Hardware
  modbus_Control.modbus_Settings.modbus_Slave_Address_u8 = 1;   //modbus address
  modbus_Control.modbus_Settings.modbus_Master_Address_u8 = 0x55; //modbus master address 
  
}

void set_BaudRate(uint16_t baud){
  modbus_Control.modbus_Settings.baudRate_u16 = baud;
  switch(baud){
    case 4800:  modbus_Control.modbus_Settings.baudRate_u16 = baud;
                break;
    case 9600:  modbus_Control.modbus_Settings.baudRate_u16 = baud;
                break;
    case 14400: modbus_Control.modbus_Settings.baudRate_u16 = baud;
                break;
    case 19200: modbus_Control.modbus_Settings.baudRate_u16 = baud;
                break;
    case 38400: modbus_Control.modbus_Settings.baudRate_u16 = baud;
                break;
    case 57600: modbus_Control.modbus_Settings.baudRate_u16 = baud;
                break;
    case 115200: modbus_Control.modbus_Settings.baudRate_u16 = baud;
                break;
    default:    modbus_Control.modbus_Settings.baudRate_u16 = DEFAULT_BAUD;
                break;    
  }
}

void set_SlaveAddress(uint8_t address){
  modbus_Control.modbus_Settings.modbus_Slave_Address_u8 = address;   //modbus address
}

void set_StopBits(uint8_t stop_bits){
  if((stop_bits != 1) || (stop_bits != 2)){ //stopbits out of range
    modbus_Control.modbus_Settings.stop_Bits_u8 = 1;         //1 or 2 
  }
  else{
    modbus_Control.modbus_Settings.stop_Bits_u8 = stop_bits;         //1 or 2
  }
}





/***************************************************************************
****** MB_UpdateComSettings ******************************************
*
*   This function is called during main_tasks when the mb_comSetUpdate flag is set
*	It updates the UART Baudrate and Parity settings using mb_comSet variable.
*   Parameters:  None - Execption code for delayed response
*
*   Inputs:	mb_comSet, MB_BaudRateValues, MB_ParityValues
*   Returns:	void
***************************************************************************/
void MB_UpdateComSettings(void) //@ "program"
{
//    mb_u16BaudRate = mb_comSet & 0x0F;
//    mb_u16Parity = (mb_comSet & 0xF0) >> 4;
//    if (mb_u16Parity < MB_NUM_PARITY_SETT)
//    {
//        uint16_t parity_data = MB_ParityValues[mb_u16Parity];
//        Set_parity(parity_data);
//    } // else {} // don't change parity if unknown value
//
//    if (mb_u16BaudRate == 0)
//    {
//    } // don't change baudrate if unsupported value
//    else if (mb_u16BaudRate < MB_NUM_BAUDRATES)
//    {
//        uint16_t baud_data = MB_BaudRateValues[mb_u16BaudRate];
//        USART1->BRR = baud_data;
//        BaudRateDiv = USART1->BRR;
//    } // else {} // don't change baudrate if unknown value
//
//    //
//    if (bIdle)
//        USART1->BRR = (BaudRateDiv >> 3); //Load Active rate/8 for 8MHz
//    addr_AK = copyaddr_AK;                //update address from Read back value
//    mb_comSetUpdate = 0;
}


/***************************************************************************
****** MB_CallBackActionCompleted ******************************************
*
*   This function is called back when a modbus requested operation is completed 
*   after some delay so that a delayed response can be sent for completion
*
*   Parameters:  TE_ModbusStatus- Execption code for delayed response
*
*   Inputs:	au8PDU, sLocals
*   Returns:	void
***************************************************************************/
void MB_CallBackActionCompleted(TE_ExceptionCode eMBStatus) //@ "program"
{
//    if (eMBStatus == EXCEPTION_00)
//    {
//        // MB_Send_response();        //Send response (reply or error)
//    }
//    else
//    {
//        /*** Build the exception response to send to Master ***/
//        mb_pau8PDU[0] |= 0x80; // error response  code
//        mb_pau8PDU[1] = EXCEPTION_04;
//        mb_u16ResponsePDUSize = 2; // All exception response PDUs are 2 bytes long
//                                   // MB_Send_response();        //Send response (reply or error)
//    }
}

/***************************************************************************
****** MB_Appl_ProcessPDU ******************************************
*
*   Implements Modbus Function Codes 01,02,03,04,05,06, and 16.  For data reads, 
*   copies the data from data handling function to local data structures & then 
*   into the buffer.  For data writes,  saves copy of data in to local data 
*   structures and calls Datahandling function to perform appropriate write
*
*
*   Parameters:  au8PDU: Modbus Protocol Data Unit (PDU) data buffer pointer
*               au8PDUSize: Modbus Protocol Data Unit Size 
*
*   Inputs:	au8PDU, sLocals
*   Returns:	void
***************************************************************************/
void MB_Appl_ProcessPDU(uint8_t *au8PDU, uint8_t u8PDUSize) //@ "program"
{
//    TS_Locals sLocals;
//    sLocals.eExceptionCode = EXCEPTION_00; // initialize exception code to no exception
//    sLocals.eMBStatus = MBSTATUS_NoError;
//
//    if (u8PDUSize < MIN_PDU_SIZE)
//    {
//        /* PDU size smaller than minimum expected; return exception code */
//        sLocals.eExceptionCode = EXCEPTION_01;
//    }
//    else
//    {
//        switch (au8PDU[0]) // check Function Code
//        {
//            case FC_01: // Read Coils
//                sLocals.u16NumOfReg = ((u16)au8PDU[3] << 8) | au8PDU[4];
//                sLocals.u16StartingAddress = ((u16)au8PDU[1] << 8) | (au8PDU[2]);
//                if (sLocals.u16NumOfReg < MIN_COILS_QUANTITY || sLocals.u16NumOfReg > MAX_COILS_QUANTITY)
//                {
//                    sLocals.eExceptionCode = EXCEPTION_03;
//                }
//                else
//                {
//                    if (((u32)sLocals.u16StartingAddress + sLocals.u16NumOfReg) > MAX_REG_ADDRESS)
//                    {
//                        sLocals.eExceptionCode = EXCEPTION_02;
//                    }
//                    else
//                    {
//                        /*** Process the request and send normal response to Master ***/
//                        MB_Process_FC01(au8PDU, &sLocals);
//                    }
//                }
//                break;
//            case FC_02: // Read Dicrete Inputs
//                sLocals.u16NumOfReg = ((u16)au8PDU[3] << 8) | au8PDU[4];
//                sLocals.u16StartingAddress = ((u16)au8PDU[1] << 8) | (au8PDU[2]);
//                if (sLocals.u16NumOfReg < MIN_DISCRETE_QUANTITY || sLocals.u16NumOfReg > MAX_DISCRETE_QUANTITY)
//                {
//                    sLocals.eExceptionCode = EXCEPTION_03;
//                }
//                else
//                {
//                    if (((u32)sLocals.u16StartingAddress + sLocals.u16NumOfReg) > MAX_REG_ADDRESS)
//                    {
//                        sLocals.eExceptionCode = EXCEPTION_02;
//                    }
//                    else
//                    {
//                        MB_Process_FC02(au8PDU, &sLocals);
//                    }
//                }
//                break;
//            case FC_03: // Read Holding Registers
//                sLocals.u16NumOfReg = ((u16)au8PDU[3] << 8) | au8PDU[4];
//                sLocals.u16StartingAddress = ((u16)au8PDU[1] << 8) | (au8PDU[2]);
//                if (sLocals.u16NumOfReg < MIN_HOLDING_QUANTITY ||
//                    sLocals.u16NumOfReg > MAX_HOLDING_READ_QUANTITY)
//                {
//                    sLocals.eExceptionCode = EXCEPTION_03;
//                }
//                else
//                {
//                    if (((u32)sLocals.u16StartingAddress + sLocals.u16NumOfReg) > MAX_REG_ADDRESS)
//                    {
//                        sLocals.eExceptionCode = EXCEPTION_02;
//                    }
//                    else
//                    {
//                        /*** Process the request and send normal response to Master ***/
//                        MB_Process_FC03(au8PDU, &sLocals);
//                    }
//                }
//                break;
//            case FC_04: // Read Input Registers
//                sLocals.u16NumOfReg = ((u16)au8PDU[3] << 8) | au8PDU[4];
//                sLocals.u16StartingAddress = ((u16)au8PDU[1] << 8) | (au8PDU[2]);
//                //if( sLocals.u16NumOfReg < 0  || // removed to eliminate compiler warn - uint cant be less than 0
//                if (sLocals.u16NumOfReg > MAX_INPUTS_QUANTITY)
//                {
//                    sLocals.eExceptionCode = EXCEPTION_03;
//                }
//                else
//                {
//                    if (((u32)sLocals.u16StartingAddress + sLocals.u16NumOfReg) > MAX_REG_ADDRESS)
//                    {
//                        sLocals.eExceptionCode = EXCEPTION_02;
//                    }
//                    else
//                    {
//                        /*** Process the request and send normal response to Master ***/
//                        MB_Process_FC04(au8PDU, &sLocals);
//                    }
//                }
//                break;
//            case FC_05: // Write Single Coil
//                sLocals.u16StartingAddress = ((u16)au8PDU[1] << 8) | (au8PDU[2]);
//                sLocals.u16OutputValue = ((u16)au8PDU[3] << 8) | (au8PDU[4]);
//                if (!(sLocals.u16OutputValue == MODBUS_CLEAR || sLocals.u16OutputValue == MODBUS_SET))
//                {
//                    sLocals.eExceptionCode = EXCEPTION_03;
//                }
//                else
//                {
//                    /*** Process the request and send response to Master ***/
//                    MB_Process_FC05(au8PDU, &sLocals);
//                }
//                break;
//            case FC_06: // Write Single Holding Register
//                sLocals.u16StartingAddress = ((u16)au8PDU[1] << 8) | (au8PDU[2]);
//                sLocals.u16RegValue = ((u16)au8PDU[3] << 8) | (au8PDU[4]);
//                /* Process the request  */
//                MB_Process_FC06(au8PDU, &sLocals);
//                break;
//            case FC_16: // Write Multiple Holding Registers
//                sLocals.u16StartingAddress = ((u16)au8PDU[1] << 8) | (au8PDU[2]);
//                sLocals.u16NumOfReg = ((u16)au8PDU[3] << 8) | (au8PDU[4]);
//                sLocals.u16RegValue = ((u16)au8PDU[6] << 8) | (au8PDU[7]);
//    #if BYTE_COUNT_USED == 1
//                sLocals.u16ByteCount = au8PDU[5];
//    #endif
//                if ((sLocals.u16NumOfReg < MIN_HOLDING_QUANTITY) ||
//                    (sLocals.u16NumOfReg > MAX_HOLDING_WRITE_QUANTITY))
//                {
//                    sLocals.eExceptionCode = EXCEPTION_03;
//                }
//                else
//                {
//    #if BYTE_COUNT_USED == 1
//                    if (sLocals.u16ByteCount != (sLocals.u16NumOfReg * 2))
//                    {
//                        sLocals.eExceptionCode = EXCEPTION_03;
//                    }
//                    else
//                    {
//    #endif
//                        if (((u32)sLocals.u16StartingAddress + sLocals.u16NumOfReg) > MAX_REG_ADDRESS)
//                        {
//                            sLocals.eExceptionCode = EXCEPTION_02;
//                        }
//                        else
//                        {
//                            /*** Process the request and send normal response to Master ***/
//                            MB_Process_FC16(au8PDU, &sLocals);
//                        }
//    #if BYTE_COUNT_USED == 1
//                    }
//    #endif
//                }
//                break;
//
//            default:
//                sLocals.eExceptionCode = EXCEPTION_01;
//                break;
//        }
//    }
//    if ((sLocals.eExceptionCode != EXCEPTION_00) && (sLocals.eExceptionCode != EXCEPTION_05))
//    {
//        /*** Build the exception response to send to Master ***/
//        au8PDU[0] |= 0x80;     // error response  code
//        au8PDU[1] = au8PDU[0]; // error response  code
//        au8PDU[2] = sLocals.eExceptionCode;
//        sLocals.u16ResponsePDUSize = 2; // All exception response PDUs are 2 bytes long
//    }
//
//    if (sLocals.eExceptionCode == EXCEPTION_05)
//    {
//        /* In case of EXCEPTION_05, the response will be sent from the
//		call back function after the delayed operation is completed */
//        mb_u16ResponsePDUSize = sLocals.u16ResponsePDUSize;
//        mb_pau8PDU = au8PDU;
//    }
//    else
//    {
//        // MB_Serial_Response(sLocals.u16ResponsePDUSize);
//        mb_resp_numBytes = sLocals.u16ResponsePDUSize + MB_BASELEN - 1;
//    }
}

/***************************************************************************
****** MB_Process_FC01() ******************************************
*
*   Processing of M0dbus Function Code 01 (Read Coils)
*
*   Parameters:  au8PDU: Modbus Protocol Data Unit (PDU) data buffer pointer
*               psLocals: pointer to data structure of local variables 
                          holding various PDU parameters like Starting Address, 
                          and Register Value
*   Inputs:	au8PDU, sLocals
*   Returns:	void
***************************************************************************/
void MB_Process_FC01(uint8_t *au8PDU, TS_Locals *psLocals) //@ "program"
{
    uint16_t i, j;

//    au8PDU[0] = FC_01;
//    psLocals->u8CoilReg = 0;
//    psLocals->u8BitPosition = 0;
//
//    // number of byte counts to follow
//    if (psLocals->u16NumOfReg % 8 == 0)
//    {
//        au8PDU[1] = psLocals->u16NumOfReg / 8;
//    }
//    else
//    {
//        au8PDU[1] = 1 + (psLocals->u16NumOfReg / 8);
//    }
//
//    j = 2; // starting address for the byte count for the discrete data
//    for (i = psLocals->u16StartingAddress; i < (psLocals->u16NumOfReg + psLocals->u16StartingAddress); i++)
//    {
//        psLocals->u16RegValue = MB_DataHandle_GetCoilData(i);
//        if (psLocals->u16RegValue == 0x0001)
//        {
//            /* If the register value is 1, set the corresponding bit in Coil Register */
//            psLocals->u8CoilReg |= (1 << psLocals->u8BitPosition);
//        }
//        if (psLocals->u8BitPosition++ == 7)
//        {
//            au8PDU[j++] = psLocals->u8CoilReg;
//            psLocals->u8BitPosition = 0;
//            psLocals->u8CoilReg = 0;
//        }
//    }
//    // copy the last partial byte into response PDU
//    if (psLocals->u16NumOfReg < 8)
//    {
//        au8PDU[j] = psLocals->u8CoilReg;
//        psLocals->u16ResponsePDUSize = j + 1;
//    }
//    else if ((psLocals->u16NumOfReg % 8) == 0)
//    {
//        psLocals->u16ResponsePDUSize = j;
//    }
//    else
//    {
//        au8PDU[j] = psLocals->u8CoilReg;
//        psLocals->u16ResponsePDUSize = j + 1;
//    }
}


/***************************************************************************
****** MB_Process_FC02() ******************************************
*
*   Processing of M0dbus Function Code 02 (read Discrete Inputs)
*
*   Parameters:  au8PDU: Modbus Protocol Data Unit (PDU) data buffer pointer
*               psLocals: pointer to data structure of local variables 
                          holding various PDU parameters like Starting Address, 
                          and Register Value
*   Inputs:	au8PDU, sLocals
*   Returns:	void
***************************************************************************/
void MB_Process_FC02(uint8_t *au8PDU, TS_Locals *sLocals) //@ "program"
{
    uint16_t i, j;

//    /*** Process the request and send normal response to Master ***/
//    au8PDU[0] = FC_02;
//    sLocals->u8DiscreteRegs = 0;
//    sLocals->u8BitPosition = 0;
//
//    // number of byte counts to follow
//    if (sLocals->u16NumOfReg % 8 == 0)
//    {
//        au8PDU[1] = sLocals->u16NumOfReg / 8;
//    }
//    else
//    {
//        au8PDU[1] = 1 + (sLocals->u16NumOfReg / 8);
//    }
//
//    j = 2; // starting address for the byte count for the discrete data
//    for (i = sLocals->u16StartingAddress; i < (sLocals->u16NumOfReg + sLocals->u16StartingAddress); i++)
//    {
//        sLocals->u16RegValue = MB_DataHandle_GetDiscreteData(i);
//        if (sLocals->u16RegValue >= 0x0001)
//        {
//            /* If the register value is 1, set the corresponding bit in Coil Register */
//            sLocals->u8DiscreteRegs |= (1 << sLocals->u8BitPosition);
//        }
//        if (sLocals->u8BitPosition++ == 7)
//        {
//            au8PDU[j++] = sLocals->u8DiscreteRegs;
//            sLocals->u8BitPosition = 0;
//            sLocals->u8DiscreteRegs = 0;
//        }
//    }
//    // copy the last partial byte into response PDU
//    if (sLocals->u16NumOfReg < 8)
//    {
//        au8PDU[j] = sLocals->u8DiscreteRegs;
//        sLocals->u16ResponsePDUSize = j + 1;
//    }
//    else if ((sLocals->u16NumOfReg % 8) == 0)
//    {
//        sLocals->u16ResponsePDUSize = j;
//    }
//    else
//    {
//        au8PDU[j] = sLocals->u8DiscreteRegs;
//        sLocals->u16ResponsePDUSize = j + 1;
//    }
}

/***************************************************************************
****** MB_Process_FC03() ******************************************
*
*   Processing of M0dbus Function Code 03 (Read Holding Registers)
*
*   Parameters:  au8PDU: Modbus Protocol Data Unit (PDU) data buffer pointer
*               psLocals: pointer to data structure of local variables 
                          holding various PDU parameters like Starting Address, 
                          and Register Value
*   Inputs:	au8PDU, sLocals
*   Returns:	void
***************************************************************************/
void MB_Process_FC03(uint8_t *au8PDU, TS_Locals *sLocals) //@ "program"
{
//    uint8_t i;
//    uint16_t j;
//
//    au8PDU[0] = FC_03;
//    au8PDU[1] = sLocals->u16NumOfReg * 2;
//    j = 2;
//    for (i = sLocals->u16StartingAddress; i < (sLocals->u16NumOfReg + sLocals->u16StartingAddress); i++)
//    {
//        sLocals->u16RegValue = MB_DataHandle_GetHoldingData(i);
//        au8PDU[j++] = sLocals->u16RegValue >> 8;
//        au8PDU[j++] = (u8)(sLocals->u16RegValue);
//    }
//    sLocals->u16ResponsePDUSize = j;
}

/***************************************************************************
****** MB_Process_FC04() ******************************************
*
*   Processing of M0dbus Function Code 04 (Read Input Registers)
*
*   Parameters:  au8PDU: Modbus Protocol Data Unit (PDU) data buffer pointer
*               psLocals: pointer to data structure of local variables 
                          holding various PDU parameters like Starting Address, 
                          and Register Value
*   Inputs:	au8PDU, sLocals
*   Returns:	void
***************************************************************************/
void MB_Process_FC04(uint8_t *au8PDU, TS_Locals *sLocals) //@ "program"
{
//    uint8_t i;
//    uint16_t j;
//
//    au8PDU[0] = FC_04;
//    au8PDU[1] = sLocals->u16NumOfReg * 2;
//    j = 2;
//
//    for (i = 0; i < sLocals->u16NumOfReg; i++)
//    {
//        sLocals->u16RegValue = MB_DataHandle_GetInputData(sLocals->u16StartingAddress);
//        au8PDU[j++] = sLocals->u16RegValue >> 8;
//        au8PDU[j++] = (u8)(sLocals->u16RegValue);
//        sLocals->u16StartingAddress++;
//    }
//    sLocals->u16ResponsePDUSize = j;
}

/***************************************************************************
****** MB_Process_FC05() ******************************************
*
*   Processing of M0dbus Function Code 05 (Write Single Coil)
*   Check Coil Register Address & set data for the coil
*
*   Parameters:  au8PDU: Modbus Protocol Data Unit (PDU) data buffer pointer
*               psLocals: pointer to data structure of local variables 
                          holding various PDU parameters like Starting Address, 
                          and Register Value
*   Inputs:	au8PDU, sLocals
*   Returns:	void
***************************************************************************/
void MB_Process_FC05(uint8_t *au8PDU, TS_Locals *sLocals) //@ "program"
{
//    au8PDU[0] = FC_05;
//    if (sLocals->u16OutputValue == MODBUS_SET)
//    {
//        sLocals->u16OutputValue = 1;
//    }
//    else
//    {
//        sLocals->u16OutputValue = 0;
//    }
//    sLocals->eExceptionCode =
//        MB_DataHandle_SetCoilData(sLocals->u16StartingAddress, sLocals->u16OutputValue, MB_CallBackActionCompleted);
//    sLocals->u16ResponsePDUSize = 5;
}

/***************************************************************************
****** MB_Process_FC06() ******************************************
*
*   Processing of M0dbus Function Code 06 (Write Single Holding Register)
*   Check Holding Register Address & report the formated data for same
*
*   Parameters:  au8PDU: Modbus Protocol Data Unit (PDU) data buffer pointer
*               psLocals: pointer to data structure of local variables 
                          holding various PDU parameters like Starting Address, 
                          and Register Value
*   Inputs:	au8PDU, sLocals
*   Returns:	void
***************************************************************************/
void MB_Process_FC06(uint8_t *au8PDU, TS_Locals *sLocals) //@ "program"
{
//    au8PDU[0] = FC_06;
//    sLocals->eExceptionCode =
//        MB_DataHandle_SetHoldingData(sLocals->u16StartingAddress, sLocals->u16RegValue, MB_CallBackActionCompleted);
//    /* Now send normal response to Master */
//    sLocals->u16ResponsePDUSize = 5;
}

/***************************************************************************
****** MB_Process_FC16() ******************************************
*
*   Processing of M0dbus Function Code 16 (Write Multiple Holding Registers)
*   Check Holding Register Address & report the formated data for same
*
*   Parameters:  au8PDU: Modbus Protocol Data Unit (PDU) data buffer pointer
*               psLocals: pointer to data structure of local variables 
                          holding various PDU parameters like Starting Address, 
                          and Register Value
*   Inputs:	au8PDU, sLocals
*   Returns:	void
***************************************************************************/
void MB_Process_FC16(uint8_t *au8PDU, TS_Locals *sLocals) //@ "program"
{
    uint16_t i;
    uint16_t j;

//    au8PDU[0] = FC_16;
//    #if BYTE_COUNT_USED == 1
//    j = 6; // starting address of data
//    #else
//    j = 5;                                                      // starting address of data
//    #endif
//    for (i = sLocals->u16StartingAddress; i < (sLocals->u16StartingAddress + sLocals->u16NumOfReg); i++)
//    {
//        sLocals->u16RegValue = ((u16)au8PDU[j] << 8) | au8PDU[j + 1];
//        sLocals->eExceptionCode = MB_DataHandle_SetHoldingData(i, sLocals->u16RegValue, MB_CallBackActionCompleted);
//        if (sLocals->eExceptionCode != EXCEPTION_00)
//        {
//            break;
//        }
//        j += 2;
//    }
//    /* Now send normal response to Master */
//    sLocals->u16ResponsePDUSize = 5;
}

/***************************************************************************
****** MB_Appl_HandleError() ******************************************
*
*   Keeps track of communication erros encountered while processing Modbus requests.
*
*   Parameters:   eMBErrors: pointer to data structure of Error status 
*
*   Inputs:	TE_MBStatus eMBErrors
*   Returns:	void
***************************************************************************/
void MB_Appl_HandleError(TE_MBStatus eMBErrors) //@ "program"
{
    switch (eMBErrors)
    {
        case MBSTATUS_CommError:
            mb_sErrorCounters.CommErrorCount++;
            break;
        case MBSTATUS_CRCError:
            mb_sErrorCounters.CRCErrorCount++;
            break;
        case MBSTATUS_InvalidSlaveAddress:
            mb_sErrorCounters.InvalidSlaveAddressCount++;
            break;
        case MBSTATUS_InvalidPDUSize:
            mb_sErrorCounters.InvalidPDUSize++;
            break;
        default:
            break;
    }
}

/***************************************************************************
****** MB_DataHandle_GetHoldingData ******************************************
*
*       	Datahandling for ModBus GetHoldingData FC03 will be perfomed here.
*   Check Holding Register Address & report the formated data for same
*
*   Parameters:	u16RegAddress = Register address to be read
*               u16DataValue = data value returned after reading register
*   Inputs:	u16RegAddress
*   Returns:	u16DataValue
***************************************************************************/
uint16_t MB_DataHandle_GetHoldingData(uint16_t u16RegAddress) //@ "program"
{
    uint16_t u16DataValue;

    u16DataValue = 0;
//    switch (u16RegAddress)
//    {
//        case 102: // Get Relative Speed Demand (returns a uint16_t scaled between VspdMin and VspdMax)
//            if (constSpd)
//            {
//                extern uint8_t nvVspdMin;
//                extern uint8_t nvVspdMax;
//                union word_def max;
//                union word_def min;
//                min.word = nvVspdMin << 8;
//                max.word = nvVspdMax << 8;
//                if (constSpd && !nvbSpd2X)
//                {
//                    max.word >>= 1;
//                    min.word >>= 1;
//                }
//                // - Scale The Incoming Speed Demand Value between VspdMin and VspdMax and set to gDSI_demand
//                if (nvVspdMax < nvVspdMin)
//                { // Instruction Cycles (40 in function)/ (25 in place) from Here to end of inverted LerpInt2
//                    u16DataValue = Lerp2Int(min.word, max.word, ~gDSI_demand);
//                    // u16DataValue = LinearInterpolation_CustomRangeToU16(min.word, max.word, ~gDSI_demand); // Note: this is as fast as Lerp2Int
//                }
//                else
//                {
//                    u16DataValue = Lerp2Int(min.word, max.word, gDSI_demand); //Normal
//                    // u16DataValue = LinearInterpolation_CustomRangeToU16(min.word, max.word, gDSI_demand); // Note: this is as fast as Lerp2Int
//                }
//            }
//            else
//            {
//                u16DataValue = 0;
//            }
//            break;
//        case 103:
//            if (constSpd)
//            {
//                u16DataValue = 0;
//            }
//            else
//            {
//                u16DataValue = gDSI_demand; //Get the set demand with value from master.
//            }
//            break;
//        case 106:
//            u16DataValue = sr_step;
//            break;
//        case 110:
//            u16DataValue = gKp;
//            break;
//        case 111:
//            u16DataValue = gKi;
//            break;
//        case 112:
//            u16DataValue = gKd;
//            break;
//        case 113:
//            u16DataValue = addr_AK;
//            break;
//        case 114:
//            u16DataValue = mb_comSet;
//            break;
//        default:
//            break;
//    }
    return (u16DataValue);
}

/***************************************************************************
****** MB_DataHandle_SetHoldingData ******************************************
*
*       	Datahandling for ModBus SetHoldingData FC06/FC16 will be perfomed here.
*   Check Holding Register Address & set the data for selected register
*
*   Parameters:  u16RegAddress = Register address to be set
*               u16Data = data value to be writted
*               pfCallBackMB = function pointer for any callback routine
*               in case of delay in response (not used presently)
*   Inputs:	u16RegAddress, u16Data, *pfCallBackMB
*   Returns:	TE_ExceptionCode - exception type in response
***************************************************************************/
TE_ExceptionCode MB_DataHandle_SetHoldingData(uint16_t u16RegAddress, uint16_t u16Data,
                                              void (*pfCallBackMB)(TE_ExceptionCode)) //@ "program"
{
//    TE_ExceptionCode eExceptionCode;
//    //u32 u32Value;
//
//    eExceptionCode = EXCEPTION_00;
//    m_pfCallBackMB = pfCallBackMB;
//
//    switch (u16RegAddress)
//    {
//        case 102: // Set Relative Speed Demand (receives uint16_t that will be scaled to VspdMin and VspdMax)
//            if (nvbCspdDAlwd)
//            {                                  //Is this mode allowed?
//                eExceptionCode = EXCEPTION_04; //Send error
//            }
//            else
//            {
//                //- Get min and max values and convert to int.
//                extern uint8_t nvVspdMin;
//                extern uint8_t nvVspdMax;
//                union word_def max;
//                union word_def min;
//                min.word = nvVspdMin << 8;
//                max.word = nvVspdMax << 8;
//                if (constSpd && !nvbSpd2X)
//                {
//                    max.word >>= 1;
//                    min.word >>= 1;
//                }
//
//                // - Scale The Incoming Speed Demand Value between VspdMin and VspdMax and set to gDSI_demand
//                if (nvVspdMax < nvVspdMin) // 28 Instruction Cycles (C version) vs. 32 Instuction Cycles (asm version)
//                {
//                    gDSI_demand = LerpInt2(max.word, min.word, ~u16Data); //Inverted
//                    // gDSI_demand = LinearInterpolation_u16ToCustomRange(max.word, min.word, ~u16Data);
//                }
//                else
//                {
//                    gDSI_demand = LerpInt2(min.word, max.word, u16Data); //Normal
//                    // gDSI_demand = LinearInterpolation_u16ToCustomRange(max.word, min.word, ~u16Data);
//                }
//                //gDSI_demand = u16Data;        //Set the speed with value from master.
//                bmc_dmnd_set = TRUE; //The demand has been set.
//                SETREGMODE(REG_SPD); //Set speed requlation mode
//    #if REMOVE_SERIAL_PIN_SWITCHING == 1
//                // Switch the Device to serial demand mode if required.
//                if (nvbOp_ser)
//                {
//                    op_ser = 1;
//                    op_pwm = 0;
//                    op_tstat = 0;
//                }
//            }
//    #endif //REMOVE_SERIAL_PIN_SWITCHING == 1
//            break;
//        case 103:
//            //Verify 2 parameters sent
//            gDSI_demand = u16Data; //Set the torque with value from master.
//            bmc_dmnd_set = TRUE;   //The demand has been set.
//            SETREGMODE(REG_TQ);    //Set torque requlation mode
//    #if REMOVE_SERIAL_PIN_SWITCHING == 1
//            // Switch the Device to serial demand mode if required.
//            if (nvbOp_ser)
//            {
//                op_ser = 1;
//                op_pwm = 0;
//                op_tstat = 0;
//            }
//    #endif //REMOVE_SERIAL_PIN_SWITCHING == 1
//            break;
//        case 106:
//            if (u16Data <= 255)
//            {
//                sr_step = u16Data;
//            }
//            else
//            {
//                eExceptionCode = EXCEPTION_03;
//            }
//            break;
//        case 110:
//            if (u16Data <= 255)
//            {
//                gKp = u16Data;
//            }
//            else
//            {
//                eExceptionCode = EXCEPTION_03;
//            }
//            break;
//        case 111:
//            if (u16Data <= 255)
//            {
//                gKi = u16Data;
//            }
//            else
//            {
//                eExceptionCode = EXCEPTION_03;
//            }
//            break;
//        case 112:
//            if (u16Data <= 255)
//            {
//                gKd = u16Data;
//            }
//            else
//            {
//                eExceptionCode = EXCEPTION_03;
//            }
//            break;
//        case 113:
//            if (!Is_motor_state_stop())
//            {                                  //Can't change address if running
//                eExceptionCode = EXCEPTION_04; //Send error
//            }
//            else if (u16Data > MAX_SLAVE_ADDRESS)
//            {                                  //Don't address > 247
//                eExceptionCode = EXCEPTION_03; //Send error - zero is broadcast address.
//            }
//            else
//            {
//                if (EEbytemodify(eeAddr_AK, u16Data, &copyaddr_AK))
//                {
//                    mb_delayUpdate = 1;
//                }
//                else
//                {
//                    eExceptionCode = EXCEPTION_04; //Send error - EE write failed
//                }
//            }
//            break;
//        case 114:
//            if (!Is_motor_state_stop())
//            {                                  //Can't change baudrate if running
//                eExceptionCode = EXCEPTION_04; //Send error
//            }
//            else if (u16Data == 0)
//            {                                  //Don't allow zero as argument
//                eExceptionCode = EXCEPTION_04; //Send error .
//            }
//            else
//            {
//                // if (EEbytemodify(eeCOMSET, u16Data, &mb_comSet)) // ERM Gen1 baudrate is stored inside of Flash Map
//                {
//                    mb_comSet = u16Data;
//                    mb_delayUpdate = 1;
//                }
//                //   else
//                //   {
//                //     eExceptionCode = EXCEPTION_04;  //Send error - EE write failed
//                //   }
//            }
//            break;
//
//        default:
//            break;
//    }
//    return (eExceptionCode);
}

/***************************************************************************
****** MB_DataHandle_GetInputData ******************************************
*
*       	Datahandling for ModBus GetInputData FC02 will be perfomed here.
*   Check Input Register Address & report the formated data for same
*
*   Parameters:	u16RegAddress = Register address to be read
*               u16DataValue = data value returned after reading register
*   Inputs:	u16RegAddress
*   Returns:	u16DataValue
***************************************************************************/
uint16_t MB_DataHandle_GetInputData(uint16_t u16RegAddress) //@ "program"
{
//    uint16_t u16DataValue;
//    uint16_t *revWord; //Pointer to revision bytes
//                  //int test;
//    u16DataValue = 0;
//    switch (u16RegAddress)
//    {
//            // 0-9 non volatile type parameters
//        case 0:
//            revWord = (uint16_t *)(&rom_fw_rev + 6); // FW Rev ASCII Lower Word
//            u16DataValue = *revWord;
//            u16DataValue = __Swap(u16DataValue);
//            break;
//        case 1:
//            revWord = (uint16_t *)(&rom_fw_rev + 3); // FW Rev ASCII Mid Word
//            u16DataValue = *revWord;
//            u16DataValue = __Swap(u16DataValue);
//            break;
//        case 2:
//            revWord = (uint16_t *)(&rom_fw_rev); // FW Rev ASCII High Word
//            u16DataValue = *revWord;
//            u16DataValue = __Swap(u16DataValue);
//            break;
//        case 3:
//            u16DataValue = (u8)nvFWRevShort; // FW Rev short
//            break;
//        case 4:
//            u16DataValue = (u8)nvContRev; // control revision
//            break;
//        case 5:
//            u16DataValue = (u8)nvContType; // control type
//            break;
//        // reserved 6 - 9
//        // 10 - 39 - volatile type parameters
//        case 10:
//            if (constSpd)
//            { //Send filtered version if constant speed mode.
//                //Send filtered version bits 8:23
//                u16DataValue = (u16)(gF_torque >> 8);
//            }
//            else
//            {
//                u16DataValue = gTorque; //Return gTorque
//            }
//            break;
//        case 11:
//            u16DataValue = gDSI_demand;
//            break;
//        case 12:
//            u16DataValue = gSpeed / 22; //(gSpeed << 2)/22;         // RPM = (gSpeed * 4)/22
//            break;
//        case 13:
//            u16DataValue = gF_demand;
//            break;
//        case 14:
//            u16DataValue = gV_bus; // DC bus = gV_bus / 16
//            break;
//        case 15:
//            u16DataValue = (u16)Form_status_byte();
//            break;
//        case 16:
//            u16DataValue = (u16)Form_diag_resp();
//            break;
//        case 17:
//            u16DataValue = gDemand;
//            break;
//        case 18:
//            // u16DataValue = gF_duty_fac>>8;
//            u16DataValue = dfin_duty;
//            break;
//        case 19:
//            u16DataValue = duty_fac;
//            break;
//        case 20:
//            u16DataValue = gDeviceTemp;
//            break;
//        case 21:
//            u16DataValue = *(uint16_t *)&last_pout;
//            break;
//        case 22:
//            u16DataValue = gModule_temp;
//            break;
//        // 23-39 reserved
//        case 40:
//            u16DataValue = *(uint8_t *)(&time_pwrd.ub);
//            break;
//        case 41:
//            u16DataValue = *(uint8_t *)(&time_pwrd.ub + 1);
//            break;
//        case 42:
//            u16DataValue = *(uint8_t *)(&time_pwrd.ub + 2);
//            break;
//        case 43:
//            u16DataValue = *(uint8_t *)(&time_run.ub);
//            break;
//        case 44:
//            u16DataValue = *(uint8_t *)(&time_run.ub + 1);
//            break;
//        case 45:
//            u16DataValue = *(uint8_t *)(&time_run.ub + 2);
//            break;
//        case 46:
//            u16DataValue = *(uint8_t *)(&time_p80.ub);
//            break;
//        case 47:
//            u16DataValue = *(uint8_t *)(&time_p80.ub + 1);
//            break;
//        case 48:
//            u16DataValue = *(uint8_t *)(&time_p80.ub + 2);
//            break;
//        case 49:
//            u16DataValue = *(uint8_t *)(&timinband2.ub);
//            break;
//        case 50:
//            u16DataValue = *(uint8_t *)(&timinband2.ub + 1);
//            break;
//        case 51:
//            u16DataValue = *(uint8_t *)(&timinband2.ub + 2);
//            break;
//        case 52:
//            u16DataValue = *(uint8_t *)(&time_cutb.ub);
//            break;
//        case 53:
//            u16DataValue = *(uint8_t *)(&time_cutb.ub + 1);
//            break;
//        case 54:
//            u16DataValue = *(uint8_t *)(&time_cutb.ub + 2);
//            break;
//        case 55:
//            u16DataValue = *(uint8_t *)(&run_cycles.ub);
//            break;
//        case 56:
//            u16DataValue = *(uint8_t *)(&run_cycles.ub + 1);
//            break;
//        case 57:
//            u16DataValue = *(uint8_t *)(&run_cycles.ub + 2);
//            break;
//        case 58:
//            u16DataValue = *(uint8_t *)(&time_mxtmp_addr.ub);
//            break;
//        case 59:
//            u16DataValue = *(uint8_t *)(&time_mxtmp_addr.ub + 1);
//            break;
//        case 60:
//            u16DataValue = *(uint8_t *)(&time_mxtmp_addr.ub + 2);
//            break;
//        case 61:
//            revWord = (uint16_t *)(&nvSerNum);
//            u16DataValue = *revWord;
//            break;
//        case 62:
//            revWord = (uint16_t *)(&nvSerNum) + 1;
//            u16DataValue = *revWord;
//            break;
//        case 63:
//            revWord = (uint16_t *)(&nvHistory_L);
//            u16DataValue = *revWord;
//            break;
//        default:
//            break;
//    }
//    return (u16DataValue);
}

/***************************************************************************
****** MB_DataHandle_SetCoilData ******************************************
*
*       	Datahandling for ModBus SetCoilData FC05 will be perfomed here.
*   Check Coil Register Address & set the data for selected register
*
*   Parameters:	u16RegAddress = Register address to be set
*               u16Data = data value to be writted
*               pfCallBackMB = function pointer for any callback routine
*               in case of delay in response (not used presently)
*   Inputs:	u16RegAddress, u16Data, *pfCallBackMB
*   Returns:	TE_ExceptionCode - exception type in response
***************************************************************************/
TE_ExceptionCode MB_DataHandle_SetCoilData(uint16_t u16RegAddress, uint16_t u16Data, void (*pfCallBackMB)(TE_ExceptionCode)) //@ "program"
{
//    TE_ExceptionCode eExceptionCode;
//    uint8_t btmp;
//
//    eExceptionCode = EXCEPTION_00;
//
//    m_pfCallBackMB = pfCallBackMB;
//
//    switch (u16RegAddress)
//    {
//        case 1:
//            if (u16Data == 1)
//            {
//                //#if GPM10H_MODEL == 0
//                //  #if SAMM_MODEL == 0
//                //     ioPFCena = 1;                    //Start PFC
//                //  #endif
//                //#endif
//                if (!OK_to_start())
//                {                                  //Are all necessary run parms available?
//                    eExceptionCode = EXCEPTION_04; //Missing parameter error
//                }
//                //#if GPM10H_MODEL == 0
//                //  #if SAMM_MODEL == 0
//                //else if (bRas) {                      //UL lockout. Brownout can be checked due to PFC until 100msec
//                //  #else
//                //else if (bRas || bBrownout) {         //UL lockout or brownout?
//                //  #endif
//                //#else
//                else if (bRas || bBrownout)
//                {                                  //UL lockout or brownout?
//                                                   //#endif
//                    eExceptionCode = EXCEPTION_04; //Indicate fault condition.
//                }
//                else
//                {              //Necessary run parameters available (including demand)
//                    onoff = 1; //Set onoff flag to start the motor
//                               //Demand will be restored in Calc_dmd.
//                }
//                if (!onoff)
//                {
//                    //#if GPM10H_MODEL == 0
//                    //  #if SAMM_MODEL == 0
//                    //     ioPFCena = 0;                  //Start PFC
//                    //  #endif
//                    //#endif
//                }
//            }
//            else
//            {
//                if (pver1 || nvbNoSlewPv2)
//                    gF_demand = 0; //No slew on stop for pver1 (same as 2.5)
//                                   // or pver2 if so optioned.
//                Stop_motor();      //Begin motor stop sequence.
//                onoff = 0;         //Clear onoff flag
//                                   //#if GPM10H_MODEL == 0
//                                   //    #if SAMM_MODEL == 0
//                                   //       ioPFCena = 0;      //Off PFC
//                                   //    #endif
//                                   //#endif
//            }
//            break;
//        case 2:
//            if (!Load_dir(u16Data))
//            {                                  //Direction is passed in 1st byte, bit 0
//                eExceptionCode = EXCEPTION_04; //Send error
//            }
//            else
//            {
//                btmp = bDir;
//                if (!nvbInvDir)
//                    btmp = ~btmp;   //Invert response if necessary
//                bmc_dir_set = TRUE; //Set dir_set flag
//            }
//        default:
//            break;
//    }
//    return (eExceptionCode);
}

/***************************************************************************
****** MB_DataHandle_GetCoilData ******************************************
*
*       	Datahandling for ModBus GetCoilData FC01 will be perfomed here.
*   Check Coil Register Address & report the formated data for same
*
*   Parameters:	u16RegAddress = Register address to be read
*               u16DataValue = data value returned after reading register
*   Inputs:	u16RegAddress
*   Returns:	u16DataValue
***************************************************************************/
uint16_t MB_DataHandle_GetCoilData(uint16_t u16RegAddress) //@ "program"
{
    uint16_t u16DataValue;

//    u16DataValue = 0;
//    switch (u16RegAddress)
//    {
//        case 1:
//            u16DataValue = onoff;
//            break;
//        case 2:
//            u16DataValue = bDir;
//            break;
//        default:
//            break;
//    }
    return (u16DataValue);
}

/***************************************************************************
****** MB_DataHandle_GetDiscreteData ******************************************
*
*       	Datahandling for ModBus Dicrete Data FC02 will be perfomed here.
*   Check DicretedData Register Address & report the formated data for same
*
*   Parameters:	u16RegAddress = Register address to be read
*               u16DataValue = data value returned after reading register
*   Inputs:	u16RegAddress
*   Returns:	u16DataValue
***************************************************************************/
uint16_t MB_DataHandle_GetDiscreteData(uint16_t u16RegAddress) //@ "program"
{
    uint16_t u16DataValue;

    u16DataValue = 0;
//    switch (u16RegAddress)
//    {
//        case 0:
//            u16DataValue = bOvertemp;
//            break;
//        case 1:
//            u16DataValue = bRamping;
//            break;
//        case 2:
//            u16DataValue = (motor_state == MOTOR_START);
//            break;
//        case 3:
//            u16DataValue = (motor_state > MOTOR_IDLE);
//            break;
//        case 4:
//            u16DataValue = bCutback || bScutback;
//            break;
//        case 5:
//            u16DataValue = bLckdRtr;
//            break;
//        case 6:
//            u16DataValue = bRas;
//            break;
//        case 7:
//            u16DataValue = (motor_state == MOTOR_BRAKE);
//            break;
//        case 8:
//            u16DataValue = bVlimited;
//            break;
//        case 9:
//            u16DataValue = bOpenPh;
//            break;
//        case 10:
//            u16DataValue = bOvervoltg;
//            break;
//        case 11:
//            u16DataValue = bBrownout;
//            break;
//        case 12:
//            //reserved
//            break;
//        case 13:
//            u16DataValue = gULFaultCopy & 0x07FF;
//            break;
//        case 14:
//            u16DataValue = constSpd;
//            break;
//        case 15:
//            u16DataValue = bCoherence_fail;
//            break;
//        case 16:
//            u16DataValue = bCoherence_fail;
//            break;
//        case 17:
//            u16DataValue = bLostPh_line;
//            break;
//        case 18:
//            u16DataValue = bVlimited;
//            break;
//        case 19:
//            u16DataValue = bLostPh_line;
//            break;
//        case 20:
//            u16DataValue = bSafety_st;
//            break;
//        case 21:
//            u16DataValue = bIaFault;
//            break;
//        case 22:
//            u16DataValue = bIbFault;
//            break;
//        case 23:
//            u16DataValue = bIcFault;
//            break;
//        case 24:
//            u16DataValue = op_pwm;
//            break;
//        default:
//            break;
//    }
   return (u16DataValue);
}

/***************************************************************************
****** MB_msg_plausible ****************************************************
*               
*   Determine if the start of a message received is plausable.  Also
*   determines max msg length based on msg type and number of registers.
*   Returns TRUE if valid.
*
*   Parameters: None                 
*   Returns:    TRUE/FALSE
*
***************************************************************************/
uint8_t MB_msg_plausible(void) //@ "program"
{
//    if ((Get_rxByte(MB_ADDR_POS) != addr_AK) && (Get_rxByte(MB_ADDR_POS) != BROADCAST))
//    { // Check preamble
//        return (FALSE);
//    }
//
//    //Is there enough of the message in the buffer to do anything yet?
//    if (Get_rxBuf_Space() < MBI_MSGMIN)
//    {
//        bmc_msgproc = TRUE; //Need more of msg, set cancel loop flag.
//        return (FALSE);
//    }
//
//    if (Get_rxByte(MB_ADDR_POS) == BROADCAST)
//        bmc_broacast = 1;
//    else
//        bmc_broacast = 0;
//
//    mb_msgCntrl.msg_cmd = Get_rxByte(MB_MSGTYPE_POS); //Set message type
//
    return (MB_valid_len()); //Determine if length ok and return
}

/***************************************************************************
****** MB_valid_len *****************************************************
*               
*     Determine if complete message has been received. If partial message and
*     the buffer contains the first two characters of the read register
*     command, set the uart to even parity and enable framing error detect.
*     Enable framing error detect after a complete message has been validated.
*  
*     Returns TRUE if valid.
*
*     Parameters:                  
*     Returns:    TRUE/FALSE
*
***************************************************************************/
uint8_t MB_valid_len() //@ "program"
{
//    uint8_t tmpBufLen;   //Amount of space used in buffer
//    uint8_t tmpMsgLen;   //Len of message byte
//    uint8_t payloadsize; //Bytes in payload
//
//    //Calculate the length of this msg
//    switch (mb_msgCntrl.msg_cmd)
//    {
//        case MB_WRITE_COILS:
//        case MB_READ_COILS:
//        case MB_READ_MLT:   // Read n registers
//                            //If modbus settings have not been configured yet, do so.
//                            //Falls thru..
//        case MB_READ_INPUT: // Read n registers
//                            //If modbus settings have not been configured yet, do so.
//                            //Falls thru..
//        case MB_READ_DIS_INPUTS:
//            //If modbus settings have not been configured yet, do so.
//            //Falls thru..
//        case MB_WRT_REG:            // Write one register
//            tmpMsgLen = MBI_MSGLEN; // Fixed length
//            //if(!rxctrl_even_par) Set_parity(MB_PARITY_EVEN);  //Switch Uart for Modbus.
//            //Set_parity(MB_PARITY_EVEN);          //Switch Uart for Modbus.
//            break;
//        case MB_WRT_MLT: // Write n registers
//            payloadsize = (MB_Get_rxWord(MBI_REGCNT_POS)) << 1;
//            if (payloadsize > (MB_MAXREGS << 1))
//                payloadsize = (MB_MAXREGS << 1); //Clip
//            tmpMsgLen = MBI_MSGMIN + payloadsize + 3;
//            //if(!rxctrl_even_par) Set_parity(MB_PARITY_EVEN);  //Switch Uart for Modbus.
//            //Set_parity(MB_PARITY_EVEN);          //Switch Uart for Modbus.
//            break;
//        default:
//            // Note - Ths used to return FALSE, but we want to trap based on 2 bytes,
//            // but only after a valid full message has been received.
//            if (firstc)
//            {
//                tmpMsgLen = MBI_MSGMIN; // Pass thru to handle error
//                bmc_send_err = 1;
//                mb_msgCntrl.err_code = MB_FCODE_ERR;
//            }
//            break;
//    }
//
//    //Looks like a real command, allow echo
//    mb_msgCntrl.noecho = FALSE;
//    //Get amount of space used in buffer (length)
//    tmpBufLen = Get_rxBuf_Space();
//
//    //Is there enough of the message in the buffer to do CRC?
//    if (tmpBufLen < tmpMsgLen)
//    {
//        bmc_msgproc = TRUE; //Msg hasn't fully arrived, set cancel loop flag.
//        return (FALSE);     //Not valid length yet.
//    }
//    else
//    {                        //Possibly enough bytes in buffer to process
//        msg_len = tmpMsgLen; //Set reference length
//        return (TRUE);       //Return valid
//    }
}

/***************************************************************************
****** Valid_crc ***********************************************************
*               
*   CRC for Modbus RTU, iterative method.
*   The checksum is calculated on the message len to determine if it
*   is valid, and to add the crc to a message ready to send. 
*   Returns TRUE if valid. Adding the crc to a new message returns FALSE,
*   which is ignored.
*       
*   Parameters: len  = bytes of message to cover
*   Returns:    TRUE/FALSE, glb CkH, CkL
*
***************************************************************************/
uint8_t MB_Valid_crc(uint8_t len) //@ "program"
{
//    uint8_t pos;
//    uint8_t lastByte; //Index of the last byte to include in checksum
//    uint16_t oldcrc;
//
//    lastByte = (len - 2); // Refers to the first byte of the CRC
//    mb_crcCalc.word = 0xFFFF;
//    // Determine buffer to operate on.
//
//    for (pos = 0; pos < lastByte; pos++)
//    {
//
//        MB_Update_crc(Get_rxByte(pos));
//    }
//    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
//
//    oldcrc = MB_Get_rxWord(lastByte); // Compare old one, (not valid vor Tx)
//
//    if (oldcrc != mb_crcCalc.word)
//    {
//        return (FALSE);
//    }
//    else
//    {
//        return (TRUE);
//    }
}

/***************************************************************************
****** MB_Update_crc *********************************************************
*               
*   Process one byte into CRC for Modbus RTU.
*
*   Returns:    glb CkH, CkL
***************************************************************************/
void MB_Update_crc(uint8_t thisByte) //@ "program"
{
//    uint8_t i;
//
//    mb_crcCalc.word ^= (u16)thisByte; // XOR byte into least sig. byte of crc
//
//    for (i = 8; i != 0; i--)
//    { // Loop over each bit
//        if ((mb_crcCalc.word & 0x0001) != 0)
//        {                          // If the LSB is set
//            mb_crcCalc.word >>= 1; // Shift right and XOR 0xA001
//            mb_crcCalc.word ^= 0xA001;
//        }
//        else
//        {                          // Else LSB is not set
//            mb_crcCalc.word >>= 1; // Just shift right
//        }
//    }
}

/***************************************************************************
****** MB_DecodeCmd ***************************************************
*               
           Determine the type of Modbus command and execute it.
*   At this point the entire message has been verified.
*   This routine dispatches execution based on the command type.
*   It also checks to be sure serial mode is an acceptable mode
*   of operation.
*
*   All parameters possibly needed by most commands are extracted from
*   the rx buffer here.  This saves having to do it everywhere else.
*
*   Parameters: None                 
*   Returns:    None
*
***************************************************************************/
void MB_DecodeCmd(void) //@ "program"
{
//    uint8_t n;
//    //uint8_t numbytes;
//
//    //rxctrl_even_par = 1;       //Mark as even parity enabled.
//    fer_count = 0; //Reset the framing error count
//
//    //Extract parameters from msg assuming the current command needs these.
//    //Doing this in one place means that we don't have to do it everywhere needed.
//
//    // re-calc payloadsize.
//    // numbytes = msg_len - MB_BASELEN;
//
//    //for (n=0; n < numbytes; n++)
//    for (n = 0; n < msg_len; n++)
//    {
//        //mb_resp_parm.byte[n] = Get_rxByte(n + MBI_FIRSTREG_POS);
//        mb_resp_parm.byte[n] = Get_rxByte(n);
//    }
//
//    Activate_dsi_mode(); //Configure for serial mode.
//
//    #if REMOVE_SERIAL_PIN_SWITCHING == 0
//    if (op_ser) //Serial operation mode OK
//    {
//        firstc = TRUE;                                          // Set flag first valid msg received.
//        ser_act = TRUE;                                         // Set flag indicating recent serial activity.
//        MB_Appl_ProcessPDU(&mb_resp_parm.byte[1], msg_len - 3); // Execute
//    }
//    else
//    {                              // Not valid command mode
//        mb_msgCntrl.noecho = TRUE; // Set so no response sent
//    }
//    #else  // REMOVE_SERIAL_PIN_SWITCHING == 1
//        firstc = TRUE;                                          // Set flag first valid msg received.
//        ser_act = TRUE;                                         // Set flag indicating recent serial activity.
//        MB_Appl_ProcessPDU(&mb_resp_parm.byte[1], msg_len - 3); // Execute
//    #endif // REMOVE_SERIAL_PIN_SWITCHING == 1
//
//    // Respond to MB testmode command
//    if ((mb_msgCntrl.msg_cmd == MB_WRT_REG) && (mb_resp_parm.word[1] == MB_TESTKEY) && (mb_resp_parm.word[0] == TSTM_KEY))
//    {                       //sent bytes have to be swapped
//        gTM_key = TSTM_KEY; //Set key to enter test mode
//    }
//
//    MB_Send_response(); //Send response (reply or error)
}

/****************************************************************************
****** MB_Send_response *****************************************************
*               
*                       Sends a response message to the master. There is a
*     provision for initiating a delay before sending the response if needed.
*     Also no message is sent if the noecho flag is set.  If there isn't
*     enough space in the transmit buffer no message is sent this pass. 
*     In this case, wait4tx is set and the head pointer won't be moved. 
*     On the next execution of MB_Process a send will be attempted once more.
*     If the tx buffer has space, the message is assembled and transferred to the
*     tx buffer. Finally, if the transmitter isn't enabled, call the interrupt
*     to start the transmitter.
*     Note that if a byte is received when the Tx buffer is not empty, it will
*     be transmitted immediately. Delayed send requires that this process be
*     suspended until the delay expires and Tx_start is called. Use flag
*      to gate the Tx process in USART1_IRQHandler().
*
*
*   Parameters: None                 
*   Returns:    None
*
***************************************************************************/
void MB_Send_response(void) //@ "program"
{
//    uint8_t buf_space;   //Amount of space available in buffer
//    uint8_t rply_len;    //Amount of space required for reply
//    uint8_t n;           //Index for parameter loop
//    uint8_t first_ckpos; //1st CRC position in mb_resp_parm[]
//    extern uint8_t nvTx_delay;
//
//    register uint8_t temp; //temporary storage of msg byte
//
//    mb_crcCalc.word = 0xFFFF;
//
//    if ((mb_msgCntrl.noecho == 0) && (bmc_broacast == 0))
//    { //Was a response requested? or No broadcast address used
//
//        //How much space in tx buffer?
//        buf_space = Get_txBuf_Space();
//
//        //How much space needed?
//        rply_len = mb_resp_numBytes;
//        if (bmc_send_err != 0)
//            rply_len = 5; //Error response length
//
//        //Check if enough space
//        if (buf_space < rply_len)
//        {
//            bmc_wait4tx = TRUE; //Not enough, set flag and don't proceed
//        }
//        else
//        {                       //Enough, continue
//            txctrl_not_cts = 1; //Block the transmitter in case we need a delay
//            //Add reply preamble/address
//            if (rxctrl_xof != 0)
//            {                       // Rx buffer nearly full?  Temp!!
//                temp = DATX;        // yes, send reply with XOF warning
//                rxctrl_xof = FALSE; //If sent once, don't send again.
//            }
//            else
//            {
//                temp = addr_AK; //otherwise, use normal reply preamble
//            }
//
//            MB_add2msg(temp);
//
//            //Add command byte
//            temp = mb_msgCntrl.msg_cmd;
//            mb_msgCntrl.msg_cmd = 0;
//
//            if (bmc_send_err != 0)
//            {                       //If error occured
//                temp |= MB_CMD_ERR; //Response cmd is high bit set
//                bmc_send_err = 0;
//                MB_add2msg(temp);
//                MB_add2msg(mb_msgCntrl.err_code); //Error code
//            }
//            else
//            { //Normal message
//                MB_add2msg(temp);
//                //Add command parameters if any
//                mb_resp_pos = 2;
//                first_ckpos = mb_resp_numBytes - MB_BASELEN + mb_resp_pos;
//                for (n = mb_resp_pos; n < first_ckpos; n++)
//                {
//                    //Add byte to outgoing msg
//                    MB_add2msg(mb_resp_parm.byte[n]);
//                }
//            }
//
//
//            //Add checksum bytes
//            temp = mb_crcCalc.byte.high; //Next call destroys crc
//            MB_add2msg(mb_crcCalc.byte.low);
//            MB_add2msg(temp);
//
//
//    #if USE_TXDELAY == 1
//            temp = nvTx_delay; //Needs to be set first...
//            if ((temp > 0) && (gTx_delay_wd < DLY_WD))
//            {
//                gTx_delay_wd++;    //Start the delay watchdog
//                TIM2->SR = 0;      //Clr pending in case UIF ever gets set.
//                TIM2_DIER_UIE = 1; //enable update interrupt
//                gTim2_count = 4;   //Multiplier for TIM2 delay
//                TIM2_CR1_CEN = 1;  //Start timer
//            }
//            else
//            {
//                Tx_start();
//            }
//
//    #else
//                //Turn on the transmitter
//                Tx_start();
//    #endif
//
//        } // endif enough buffer space check
//
//    } //noecho check
//    else
//    {
//        mb_msgCntrl.msg_cmd = 0; // Only broadcast or no echo requested
//                                 // VJ REVIEW find why mb_msgCntrl.msg_cmd clearing is need in
//                                 // both if & else case here, since it is found sometime
//                                 // earlier message in event of error was not clearing. Why this is needed?
//    }
}

/****** MB_Get_txByte **********************************************************
*               
*   Using the offset passed relative to txhead, a byte from the txbuf is 
*   returned.  This routine handles the wrapping of the ptr in the buffer.
*
*   Parameters: offset - relative to txhead.
*   Returns:    reference byte from txbuf.
*
***************************************************************************/
uint8_t MB_Get_txByte(uint8_t offset) //@ "program"
{
//    uint8_t ptrtmp;
//
//    ptrtmp = (txhead + offset) & (u8)TXSIZE_BYTE_MASK; //00111111b, 64 byte buffer size
//    // JJM - was & 0x3F, but that greatly limited buffer sizes, put this constant with TXSIZE
//
//    return (txbuf[ptrtmp]);
}

/***************************************************************************
****** MB_Get_rxWord **********************************************************
*               
*   Using the offset passed relative to rxhead, a word from the rxbuf is 
*   returned.  This routine handles the wrapping of the ptr in the buffer.
*
*   Parameters: byte offset - relative to rxhead.
*   Returns:    reference word from rxbuf.
*
***************************************************************************/
uint16_t MB_Get_rxWord(uint8_t offset) //@ "program"
{
//    uint8_t ptrtmp;
//    union word_def res;
//
//    //Get first (high) byte
//    ptrtmp = (rxhead + offset) & (u8)RXSIZE_BYTE_MASK; //00111111b, 64 byte buffer size
//    res.byte.low = (rxbuf[ptrtmp]);
//    ptrtmp = (ptrtmp + 1) & (u8)RXSIZE_BYTE_MASK;
//    //Get second (low) byte
//    res.byte.high = (rxbuf[ptrtmp]);
//
//    return (res.word);
}

/***************************************************************************
****** MB_add2msg **********************************************************
*               
*                       Add a byte to the transmit buffer. CRC is separate.
*
*   Parameters: byte to be addad                 
*   Returns:    None
*
***************************************************************************/
void MB_add2msg(uint8_t element) //@ "program"
{
//    MB_Update_crc(element); // Update crc
//    txnext = element;       // Macro to insert byte
//    Inc_txtail();           // Adjust pointer
}
#endif // REMOVE_MODBUS == 0
