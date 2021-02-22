/**
  ***************************************************************************************************
  * @file    App_Template.h 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    7-Jul-2020
  * @brief   Main function/s of control motor speed by 0 to 10V
  * @note    
  ***************************************************************************************************
  */
//^** Tips: APPs/Drivers adding process example step6  [refer to user manual ) 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MODULE_MOTOR_COM_H_
#define _MODULE_MOTOR_COM_H_

/* Includes ------------------------------------------------------------------*/
//#include "stm32F0xx.h"
//#include "stm32f0_discovery.h"
//#include "core_cm0.h"
#include "main.h"
#include "structured_memory.h"
#include "scheduler.h"
#include "ring_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

  
extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];
static Ram_Buf_Handle motor_Com_StructMem_u32;
//extern Ram_Buf *analog_Input_Settings_StructMem_u32;
//extern Ram_Buf *analog_Data_StructMem_u32;
//static Ring_Buf_Handle ;
//static Ring_Buf_Handle ;

//static RingBuf* App_TemplatePipe;
//static RingBuf* App_TemplatePipe1;
//static ShareMemory* App_TemplateShMem;  

  
//******************* App_Template Control (inside shared memory) *******************************************************************************************************************************  
typedef struct 
{
  Ring_Buf_Handle*   InternalPipe;
  uint8_t  ErrorCode;                                                           //Error code
}App_TemplateControl;
  
struct Motor_Settings{
  uint8_t control_Mode_u8;      //00 = torque; 01 = Speed;
  uint8_t comm_Address_u8;      //address for univeral protocol
  uint16_t max_Speed_u16;        //Maximum allowed speed in RPM 0dp
  uint16_t min_Speed_u16;        //minumum speed in RPM 0dp
  uint16_t hysteresis_Speed_u16; //hysteresis speed in RPM 0dp  
  uint16_t max_Torque_u16;       //Maximum allowed torque
  uint16_t min_Torque_u16;       //Minmium torque
  uint16_t set_Torque_u16;       //Set torque. Changes based on user inputs  
};

struct Motor_Metering_Data{
  uint16_t analog_0_10v_u16;                     //Analog volts;
  uint16_t analog_0_10v_Demand_Percent_u16;  //Analog demand percentage 100.00%;
  uint16_t demand_Reference_Percent_u16;     //demand reference percentage
  uint16_t demand_Reference_Speed_u16;       //speed reference 100.00% 2dp
  uint16_t demand_Reference_Torque_u16;      //torque reference 100.00% 2dp
  uint16_t user_Set_Speed_u16;                   //Set speed commanded by user 0dp
  uint16_t user_Set_Torque_u16;                  //Set torque. Changes based on user inputs
  uint16_t motor_Status_u16;                     //Status of the motor
  uint8_t  errorCode_u8;                     //error codes
  bool     is_Motor_On;                      //1= ON, 0 = OFF
  bool     is_Direction;                     //1= CCW, 0 = CW
  
};

typedef struct{
  struct Motor_Settings motor_Setting;
  struct Motor_Metering_Data motor_Metering_Data;
}Motor_Com_Control;



//******************* end of App_Template Control (inside shared memory) ******************************************************************************************************************************* 
  

#ifdef __cplusplus
}
#endif

#endif /* _MODULE_MOTOR_COM_H_ */

