/***************************************************************************
****** ECM59F Platform (ECM59F) ********************************************
*
*  File:    Modbus_SL.h   Variable, Constants, Prototypes used in 
*				    implementing Modbus  protocol.
*				    First used by Carrier.
*
*		!!!!!	REGAL BELOIT COMPANY PROPRIETARY !!!!!
*
*  Author:  B.Beifus
*  Compiler: NC30WA V.5.30 Release 1
*  Target:  Renesas R5F21134
****************************************************************************
****** REVLOG **************************************************************
*   Created 12-4-14
*
*   
***************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MODULE_MODBUS_H_
#define _MODULE_MODBUS_H_

/* Includes ------------------------------------------------------------------*/
#include "structured_memory.h"
#include "scheduler.h"
#include "driver_adc1.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
                            CONSTANTS
*******************************************************************************/

#define DEFAULT_BAUD 9600

// Note: items controlled by the spec are marked with "*"

    #define MB_SCHEMA 1 //Modbus protocol register definition schema
    #define REMOVE_MODBUS 0 //1 = Remove Modbus

    // Misc constants...
    #define MB_BASELEN (u8)4          //*Packet overhead: addr, function code and crc.
    #define MBI_MSGLEN (u8)8          //*Shortest message (Write one)
    #define MBI_MSGMIN (u8)6          //*Message bytes needed to calculate length
    #define MBO_MSGMAX (u8)57         //*Max sent message length.
    #define MB_TMOUT (u8)4            //Delay to flush buffer
    #define MB_MAXPASS (u8)8          //Max number of passes through CMAND
    #define MB_MAXREGS 20             //*Max number of registers to R/W
    #define MB_REGTOP 78              //*Highest register address implemented this schema.
    #define MODBUS_T (u8) - 3         //resp_pver used to indicate Modbus_T
    #define MODBUS_ALL (u8)4          //resp_pver used to indicate Modbus_T
    #define COMSET_MIN_DELAY (u8)0x04 // Min delay to allow enter test mode. 2.4sec 1count = 800mSec

    // Packet positions for all messages, base zero --------------------------
    #define MB_ADDR_POS (u8)0    // (Byte)
    #define MB_MSGTYPE_POS (u8)1 // (Byte)

    // Packet positions for incoming messages, base zero ---------------------
    #define MBI_FIRSTREG_POS (u8)2 // (Word)
    #define MBI_REGCNT_POS (u8)5   // (Byte) Read Write multiple command
    #define MBI_WRVAL_POS (u8)4    // (Word) Write single command

    // Packet positions for outgoing messages, base zero ---------------------
    #define MBO_DATBYTES_POS (u8)2 // (Byte)
    #define MBO_WRTMLT_POS (u8)2   // (Byte)

    // Packet lengths for outgoing messages ----------------------------------
    #define MBO_WRTONE_LEN MB_BASELEN + (u8)4
    #define MBO_WRTMLT_LEN MB_BASELEN + (u8)2


    #define MB_OEMID 0x01 //*Manufacturer ID, This is RBC's OEM ID.

    // Addresses....
    #define MB_BRDCST_ADDR 0 //Broadcast address
    #define MB_SLV_ADDR 0x81 //*Slave unit address
    #define MB_CMD_ERR 0x80  //*ORed with command byte in echo

    // Message types....
    #define MB_READ_MLT 0x3        //*Read  multiple register command
    #define MB_WRT_REG 0x6         //*Write register command
    #define MB_WRT_MLT 0x10        //*Write multiple register command
    #define MB_READ_COILS 0x1      // Read coils
    #define MB_WRITE_COILS 0x5     // write single coil
    #define MB_READ_DIS_INPUTS 0x2 // Read discrete inputs
    #define MB_READ_INPUT 0x4      // Read input registers
    #define MB_READ_FILE 20
    #define MB_WRITE_FILE 21

    //Modbus error codes
    #define MB_FCODE_ERR 1  //*Function code not supported.
    #define MB_RADDR_ERR 2  //*Register not implemented.
    #define MB_WPROT_ERR 16 // Attempt to write a read only reg.
    #define MB_NRDY_ERR 17  // Motor not allowed to run now.
    #define MB_RANGE_ERR 18 // Value out of range.

    //Other constants
    #define PRECLR_PAR 0x200C     //Std config with parity bits cleared
    #define MB_PARITY_ODD 0x1600  //CR1 bits to switch parity to odd
    #define MB_PARITY_EVEN 0x1400 //CR1 bits to switch parity to even
    #define MB_PARITY_NONE 0x0000 //Bits for no parity
    #define MB_NUM_PARITY_SETT 3
    #define MB_NUM_BAUDRATES 6 // 0=none, 1-5=valid baudrates

    #define MB_TESTKEY 0x5B39 //Test mode 2nd key value (swap H/L)
    #define MB_RST 4          //Max framing errors to reset port.

    //  Not used yet...
    #define MB_MODE_CSPD 0 //Control mode is constant speed
    #define MB_MODE_CTQ 1  //Control mode is constant torque
    #define MB_MODE_CAF 2  //Control mode is constant airflow
    #define MB_DIR_CCW 0   //Direction is CCW LE of motor
    #define MB_DIR_CW 1    //Direction is CW LE of motor
    #define MB_CMD_STOP 0  //Stop the motor
    #define MB_CMD_RUN 1   //Run the motor
    #define MB_DMND_CSPD 1 //Speed demand mask
    #define MB_DMND_CTQ 2  //Torque demand mask
    #define MB_DMND_CAF 4  //Airflow demand mask

    //Registers assigned to fw rev string fields
    #define FW_REG_L 52
    #define FW_REG_M 53
    #define FW_REG_H 54

    // Error messages for debug
    #define MB_REGX 0x10 // Register not found.
    #define MB_WRTX 0x11 // Register write protected.

    // Macros
    #define modbus (resp_pver == MODBUS_T)

    #define DLY_WD     10 //Passes thru Send_response + main loop until                                                                        \
            // send delay is skipped
// Note: These have to be powers of two.
//#define 	txsize	64 					//TX Buffer length (bytes)
//#define 	rxsize	64					   //RX Buffer length (bytes)


/*******************************************************************************
                            TYPEDEF DECLARATIONS
*******************************************************************************/

typedef struct tag_modbus_ctrl
{
    //    uint8_t       cmtime:3;      //Timeout counter
    //    uint8_t       cmwait:1;      //Timeout flag
    //    uint8_t      msgproc:1;      //CMAND loop is to stop as messaged has been processed
    //  Either 1 decoded and reply sent if requested
    //      (Only 1 reply per CMAND call (~117 ms))
    //  Or not enough msg bytes (len) in buffer yet.
    //    uint8_t      wait4tx:1;      //Not enough space in Tx buffer (try again next time)
    //    uint8_t        spare:2;      //spare
    uint8_t dir_set : 1;  //Rotation set via serial command already
    uint8_t mcfm_set : 1; //MaxCFM set via serial command already
    uint8_t blwr_set : 1; //Blower constants set via serial command already
    uint8_t dmnd_set : 1; //Demand value set via serial command already
    uint8_t brk_on : 1;   //Brake feature enabled via serial command (used in test mode only)
    uint8_t spare2 : 3;   //spare
    //    u8	 msg_bad:1;	     //Error in received message.
} mb_ctrl_def;

typedef struct tag_mbmsg_ctrl
{
    uint8_t send_ack : 1; //Send ack response
    uint8_t send_err : 1; //Send err response  -- use bmc_send_err
    uint8_t rx00 : 1;     //Broadcast addr used
    uint8_t noecho : 1;   //Echo is suppressed
    uint8_t spare : 4;    //spare
    uint8_t msg_cmd;      //message command sent by master
    uint8_t err_code;     //code to return
} mb_msg_ctrl_def;

    #pragma pack(1)      // .odd.word needs to be offset by one byte.
union mb_resp_parm_def { //Define a structure for response parameters
    struct
    {
        uint8_t byte;
        uint16_t word[(MB_MAXREGS)-1];
    } odd;
    uint8_t byte[MB_MAXREGS * 2]; //
    uint16_t word[MB_MAXREGS];
};
    #pragma pack()

/***************Start of Modbus Control (inside shared memory)******************/
struct Modbus_Settings{
  uint16_t baudRate_u16;        //4800, 9600, 14400, 19200, 38400, 57600, 115200
  uint8_t data_Bits_u8;         // 8 or 9
  uint8_t parity_u8;            //1= Odd; 2 = Even; 0 = None;
  uint8_t stop_Bits_u8;         //1 or 2
  uint8_t flow_Control;         //0 = None; 1= Xon/Xoff; 2 = Hardware
  uint8_t modbus_Slave_Address_u8;   //modbus address
  uint8_t modbus_Master_Address_u8; //modbus master address 

  
};

struct Modbus_Data{
  bool is_first_Valid_Msg;           //Flag indicating first valid msg received.
  bool is_modbus_Activity;           //Flag indicating recent serial activity.
  bool is_admin_Mode;                //Flag indicating enter Administrator Mode
  
};

typedef struct{
 struct Modbus_Settings modbus_Settings ;
 struct Modbus_Data modbus_Data;  
}Modbus_Control;

/**************End of Modbus Control (inside shared memory)******************/

void set_SlaveAddress(uint8_t);
void set_StopBits(uint8_t);
void set_BaudRate(uint16_t);

/***************************************************************************/

extern mb_msg_ctrl_def mb_msgCntrl;
extern uint16_t mb_comSetUpdate;
extern uint16_t mb_blockComSetUpdate;
extern uint16_t mb_delayUpdate;
extern uint16_t mb_u16BaudRate;
extern uint16_t mb_u16Parity;
extern uint8_t rom_fw_rev;
extern const int nvContRev;
extern const int nvContType;
//extern uint8_t nvCustContType; // GPM10_0H Uses this only for a 'Carrier' 3/4HP long/short customization, likely not needed for ERM
extern const int nvSerNum;
extern const int nvFWRevShort;
extern const int nvHistory_L;
extern const int nvHistory_H;
extern const int nvBBflg1; //Want to get all 16 flags at once
// GPM10_0H uses uint8_t for all vars in this block (END)

extern uint8_t nvSpdExp;

extern uint8_t mb_comSetDelay;

/*******************************************************************************
                           FUNCTION PROTOTYPES
*******************************************************************************/

/* These routines reside in Modbus_CRST.c */
void MB_UpdateComSettings(void);
//uint8_t MB_get_regofset(reg_lookup_def *p, uint8_t num, uint8_t ubound); // Scan a register list for a hit. //SPA
uint16_t *MB_scan_reg(uint8_t num, uint8_t *wrt_priv);                   // Find register in def arrays.
uint8_t MB_read_reg(uint8_t num, uint8_t addr);                          // Read num registers at addr
uint8_t MB_write_reg(uint8_t num, uint8_t addr, uint8_t src);                 // Read num registers at addr
extern uint8_t MB_msg_plausible(void);
uint8_t MB_valid_len(void);          // Check length of partial meassage
uint16_t MB_Get_rxWord(uint8_t offset);      // Get word at byte offset
uint8_t MB_Valid_crc(uint8_t len);           // Check CRC
void MB_Update_crc(uint8_t thisByte);   // Calculate one byte
void MB_DecodeCmd(void);       // Get params and call MB_Std_Cmd
void MB_Std_Cmd(void);         // Execute Modbus read/write etc.
void MB_add2msg(uint8_t element);       // Add message byte to Tx buffer
void MB_Send_response(void);       // Send reply to master.
void MB_Update_regs(void);         // Register transfer with active RAM
uint8_t MB_MB_MB_Get_rxWord(uint8_t offset); // Read a byte from the tx buffer
// uint16_t Xchg(uint16_t inpw);                       // switch endian format
uint8_t MB_GetWriteErr(uint8_t regnum, uint16_t wrtval); // Check validity of write command.


    /******************************************************************************
*   Module Constant Macros Definitions */
    #define SOURCE_COMMUNICATION (0)


/******************************************************************************
*   Module Function Macros Definitions  */

/******************************************************************************
*   Module Enumerated Type Definitions */

typedef struct
{
    uint8_t CommErrorCount;
    uint8_t CRCErrorCount;
    uint8_t InvalidSlaveAddressCount;
    uint8_t InvalidPDUSize;
} TS_ErrorCounters;
/******************************************************************************
*   Module Enumerated Type Definitions */
typedef enum
{
    FC_01 = 1,  //read Coils
    FC_02 = 2,  // read Discrete Inputs
    FC_03 = 3,  //read holding registers
    FC_04 = 4,  // read input registers
    FC_05 = 5,  // write single coil
    FC_06 = 6,  // write single holding register
    FC_08 = 8,  // diagnostics
    FC_16 = 16, // write multiple holding registers
    FC_20 = 20, //Read File Records
    FC_21 = 21  //Write File Records
} TE_FunctionCodes;

typedef enum
{
    EXCEPTION_00,      // no exception
    EXCEPTION_01,      // Illegal Function
    EXCEPTION_02,      // Illegal Data Address (or data password protected, or invalid mode)
    EXCEPTION_03,      // Illegal Data Value
    EXCEPTION_04,      // Slave Device Failure
    EXCEPTION_05,      // Long response time by Slave device (will call callback function when the request is completed)
    EXCEPTION_12 = 12, // Data password protected
    EXCEPTION_16 = 16  //No WFC Data available
} TE_ExceptionCode;

typedef enum
{
    MBSTATUS_NoError,
    MBSTATUS_CommError,
    MBSTATUS_CRCError,
    MBSTATUS_InvalidSlaveAddress,
    MBSTATUS_InvalidPDUSize,
    MBSTATUS_DeviceBusy,
    MBSTATUS_DeviceFailed
} TE_MBStatus;

/******************************************************************************
*    Module Data Type Definitions */
typedef struct
{
    TE_ExceptionCode eExceptionCode;
    TE_MBStatus eMBStatus;
    uint16_t u16NumOfReg;
    uint16_t u16StartingAddress;
    uint16_t u16RegValue;
    uint8_t u8ResponseDataSize;
    uint8_t u8CoilReg;
    uint16_t u16OutputValue;
    uint8_t u8DiscreteRegs;
    uint8_t u8BitPosition;
    uint16_t u16ResponsePDUSize;
    uint16_t u16ByteCount;
    uint8_t u8ReferenceType;
    uint16_t u16FileNum;
    uint16_t u16RecordNum;
} TS_Locals; // these variables are used in the Process_PDU function in the switch statement

/******************************************************************************
*    Module Data Type Definitions */


/******************************************************************************
*    Module Data Declarations */
extern void MB_Appl_ProcessPDU(uint8_t *au8PDU, uint8_t u8PDUSize);
extern void MB_Appl_HandleError(TE_MBStatus eMBErrors);


#ifdef __cplusplus
}
#endif

#endif /* _MODULE_MODBUS_H_ */
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
