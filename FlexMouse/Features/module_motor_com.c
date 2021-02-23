/**
  ***************************************************************************************************
  * @file    module_motor_com.c 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    7-Jul-2020
  * @brief   Main function/s of control motor speed by 0 to 10V
  * @note   
  ***************************************************************************************************
  */

#include "module_motor_com.h"
#include "driver_usart1.h"
#include "module_analog_0_10v.h"

void init_Motor_Setting(void);
void assign_Module_Motor_Com_Mem(uint8_t);
extern uint64_t getSysCount(void);

/* SysCon handle declaration */
//extern ShareMemory ShMem[totalshMem];
//extern RingBuf RB[totalpipe];
//extern ProcessInfo processInfo[];

extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];
//ADC1_Control* adc1_Module_Control;
//static  Ram_Buf_Handle analog_0_10v_StructMem_u32;


uint64_t tt_DemandTime;                                                         //time tick storage for 0-10 V 

//bool isMotorOn = false;
uint16_t adcFilterVal = 0;
uint64_t tt_StopMotorResume;                                                    //time tick storage motor stop resume period 

#define DemandPollPeriod 100                                                       //time period for checking and sending 0-10V and speed data to motor board
#define MotorStopResumePeriod 450//00                                              //after send out stop cmd to motor board and wait period 
#define MotorStopResumePeriodMax 900//00

// Application Constants
#define MIN_COMMANDABLE_SPEED		300             // RPM
#define MAX_COMMANDABLE_SPEED		2500            //MAX_APP_SPEED_RPM 
#define ADC12B  4096
#define motorOnThreadhold       10                      // percentage of 0 to 10V for turn on Motor
#define motorOffThreadhold       5                      // percentage of 0 to 10V for turn on Motor
#define MotorMaxLimPercent      90                      // Max motor speed in percentage 90%
//#define SpeedRatePerADCValue   (MAX_COMMANDABLE_SPEED - MIN_COMMANDABLE_SPEED) / ((MotorMaxLimPercent *100) - (MIN_COMMANDABLE_SPEED * 1000))

//Usart1Control* usart1Control_AppLocal;
//ADC1Control* adc1Control_AppLocal;
Usart1_Control* usart1Control_AppLocal;
Analog_0_10V_Control* analog_0_10v_Control_AppLocal;
Motor_Com_Control *motor_Com_Control;
//Motor_Com_Control motor_Com_StructMem_u32;


enum                                                                            //Default APPs/Driver stage template
{ 
  AppInit,
  AppStart,
  //any other stage in here !!!
  startMotor,
  slowDnMotor2Stop,
  stopMotor,
  waitForIdle,
  WaitReset,
  SpdUpdate,

  //above 200 will be all interrupt for this APP
  AppIrq = 200,
  killApp = 255
};

void init_Motor_Setting(){
  (*motor_Com_Control).motor_Setting.control_Mode_u8 = 1;      //00 = torque; 01 = Speed;
  (*motor_Com_Control).motor_Setting.comm_Address_u8 = 55;     //address for univeral protocol
  (*motor_Com_Control).motor_Setting.max_Speed_u16 = 1500;      //Maximum allowed speed in RPM 0dp
  (*motor_Com_Control).motor_Setting.min_Speed_u16 = 300;       //minumum speed in RPM 0dp
  (*motor_Com_Control).motor_Setting.hysteresis_Speed_u16 = 20; //hysteresis speed in RPM 0dp  
  (*motor_Com_Control).motor_Setting.max_Torque_u16 = 0;        //Maximum allowed torque
  (*motor_Com_Control).motor_Setting.min_Torque_u16 = 10;       //Minmium torque Nm

}

/**
* @brief Assign structured memory for module
* @param drv_id_u8
* @retval None
*/
void assign_Module_Motor_Com_Mem(uint8_t drv_id_u8){  
  motor_Com_StructMem_u32 =  StructMem_CreateInstance(drv_id_u8, sizeof(Motor_Com_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a structured memory for this driver [should map it back to this driver local struct]
  motor_Com_Control = (Motor_Com_Control*)(*motor_Com_StructMem_u32).p_ramBuf_u8;
}


//^**Tips: APPs/Drivers adding process example step7 (Add the Additional funtion itself)
//uint8_t _App_template(uint8_t appID, uint8_t previousStage, uint8_t nextStage, uint8_t interruptIdentfer)  
uint8_t module_Motor_Com_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8,
                               uint8_t irq_id_u8)
{ 
  uint8_t     return_state_u8 = 0; 
  uint16_t    analog_0_10v_Demand_Percent_u16 = 0;
  //uint32_t adc1buf = (uint32_t)(*adc1Control_AppLocal).ADC1result;
  switch (next_state_u8)
    {
      case AppInit:                                                              //initial stage
        {   
          assign_Module_Motor_Com_Mem(drv_id_u8); //Assign structured memory
          init_Motor_Setting(); //initilize motor settings
          
          /*Attach Uart2 structured memory into this App*/
          //uint8_t Usart1index  = getProcessInfoIndex(DRV_USART1); //return Process index from processInfo array          
          uint8_t Usart1index  = getProcessInfoIndex(MODULE_USART1); //return Process index from processInfo array
          usart1Control_AppLocal = (Usart1_Control*)((*(processInfoTable[Usart1index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);    //Get structured memory for USART1
          
          /*Attach Analog 0-10V module structured memory into this App*/
          uint8_t module_analog_0_10V_index  = getProcessInfoIndex(MODULE_ANALOG_0_10V);   //return Process index from processInfo array
          analog_0_10v_Control_AppLocal = (Analog_0_10V_Control*)((*(processInfoTable[module_analog_0_10V_index].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);    //Get structured memory for Analog 0-10V input
          
          analog_0_10v_Demand_Percent_u16 = (*analog_0_10v_Control_AppLocal).analog_0_10V_Data.analogScaledDemand_u16;
          tt_DemandTime = getSysCount() + DemandPollPeriod;                          //store time tick value 
          
          return_state_u8 = startMotor;// stopMotor; 
          break;
        }       
      case AppStart:
        { 
          
          if (getSysCount() >= tt_DemandTime) 
          {
            ((*motor_Com_Control).motor_Metering_Data.motor_Status_u16) = (*usart1Control_AppLocal).motorStatus_u16;
            uint32_t temp_result = (uint32_t)(analog_0_10v_Demand_Percent_u16 * ((*motor_Com_Control).motor_Setting.max_Speed_u16));
            (*motor_Com_Control).motor_Metering_Data.demand_Reference_Speed_u16 = (uint16_t)(temp_result/10000); //Convert Speed ref % to RPM
            
            //if((adcFilterVal > (motorOnThreadhold * 100)) && (!isMotorOn))                           //turn motor one at 10% of 0 to 10V
            //if((analog_0_10v_Demand_Percent_u16 > (motorOnThreadhold * 100)) && (!motor_Com_Control.motor_Metering_Data.is_Motor_On))                           //turn motor one at 10% of 0 to 10V
            if((*motor_Com_Control).motor_Metering_Data.demand_Reference_Speed_u16 >= (*motor_Com_Control).motor_Setting.min_Speed_u16) //Demand > min speed
            {
              return_state_u8 = startMotor;
              break;
            }
            //if((analog_0_10v_Demand_Percent_u16 < (motorOffThreadhold * 100)) && motor_Com_Control.motor_Metering_Data.is_Motor_On)                             //stop at 5% of 0 to 10V
            if((*motor_Com_Control).motor_Metering_Data.demand_Reference_Speed_u16 < ((*motor_Com_Control).motor_Setting.min_Speed_u16 + (*motor_Com_Control).motor_Setting.hysteresis_Speed_u16)) //Demand > min speed
            {             
              return_state_u8 = slowDnMotor2Stop;
              break;
            }
            return_state_u8 = SpdUpdate;
            break;             
          }
          return_state_u8 = AppStart ;
          break;
        }
      case SpdUpdate:
        {
          
          unsigned char speedTx[] = {0x55, 0x02, 0x21, 0x00, 0x00, 0xff, 0xff, 0xCC, 0xCC};
          unsigned int speedLen = sizeof(speedTx);
          speedTx[5] = (unsigned char) (((uint16_t) ((*motor_Com_Control).motor_Metering_Data.demand_Reference_Speed_u16) & 0xff00) >> 8);
          speedTx[6] = (unsigned char) ((*motor_Com_Control).motor_Metering_Data.demand_Reference_Speed_u16) & 0xff;
          
          if((*motor_Com_Control).motor_Metering_Data.is_Motor_On)
          {
            RingBuf_WriteBlock((*usart1Control_AppLocal).seqMemTX_u32, speedTx, &speedLen);
          }
          
          tt_DemandTime = getSysCount() + DemandPollPeriod;                          //update next time tick value 
          return_state_u8 = AppStart;
          break;
            
          /*if(motor_Com_Control.motor_Metering_Data.demand_Reference_Speed_u16  < (motorOnThreadhold*100))       
          {     //set min speed when 0 to 10V lower then threadhold
            speedTx[5] = (unsigned char) (((uint16_t) (motor_Com_Control.motor_Setting.min_Speed) & 0xff00) >> 8);
            speedTx[6] = (unsigned char) (motor_Com_Control.motor_Setting.min_Speed) & 0xff;            
          }
          else
          {
            if(analog_0_10v_Demand_Percent_u16 > (MotorMaxLimPercent*100))
            {   //set max speed when 0 to 10V higher then threadhold
              speedTx[5] = (unsigned char) (((uint16_t)MAX_COMMANDABLE_SPEED & 0xff00) >> 8);
              speedTx[6] = (unsigned char) MAX_COMMANDABLE_SPEED & 0xff;                   
            }
            else
            {
              uint32_t SpeedRatePerADCValue  = ((MAX_COMMANDABLE_SPEED - (motor_Com_Control.motor_Setting.min_Speed)) * 100) / (MotorMaxLimPercent  - motorOnThreadhold);                     //use 32bit for higher motor speed motor
              //speedTx[5] = (unsigned char) ((adcFilterVal  & 0xff00) >> 8);
              //speedTx[6] = (unsigned char) adcFilterVal & 0xff;
              speedTx[5] = (unsigned char) ((((((adcFilterVal -  (motorOnThreadhold*100)) * SpeedRatePerADCValue)/10000) + (motor_Com_Control.motor_Setting.min_Speed))  & 0xff00) >> 8);
              speedTx[6] = (unsigned char) ((((adcFilterVal -  (motorOnThreadhold*100)) * SpeedRatePerADCValue)/10000) + (motor_Com_Control.motor_Setting.min_Speed)) & 0xff;
            }
          }
          if(motor_Com_Control.motor_Metering_Data.is_Motor_On)
          {
            RBWriteBlk((*usart1Control_AppLocal).TxPipe->SystemIndex, speedTx, &speedLen); 
          }
          
          tt_DemandTime = getSysCount() + DemandPollPeriod;                          //update next time tick value 
          return_state_u8 = AppStart;
          break;*/
        }
      case startMotor:
        {
          unsigned char speedTx[] = {0x55, 0x00, 0x00, 0x00, 0x00, 0xCC, 0xCC};
          unsigned int speedLen = sizeof(speedTx);
          RingBuf_WriteBlock((*usart1Control_AppLocal).seqMemTX_u32, speedTx, &speedLen);  
          (*motor_Com_Control).motor_Metering_Data.is_Motor_On = true;
          return_state_u8 = AppStart;
          break;
        }
      case slowDnMotor2Stop:
        {
          LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6);
          unsigned char speedTx[] = {0x55, 0x02, 0x21, 0x00, 0x00, 0xff, 0xff, 0xCC, 0xCC};
          unsigned int speedLen = sizeof(speedTx);
          //put 200 rpm lower speed 
          speedTx[5] = (unsigned char) (((uint16_t)((*motor_Com_Control).motor_Setting.min_Speed_u16) & 0xff00) >> 8);
          speedTx[6] = (unsigned char) ((*motor_Com_Control).motor_Setting.min_Speed_u16) & 0xff;     
//          speedTx[5] = 0;
//          speedTx[6] = 0xc8;      
          if((*motor_Com_Control).motor_Metering_Data.is_Motor_On)
          {
            RingBuf_WriteBlock((*usart1Control_AppLocal).seqMemTX_u32, speedTx, &speedLen);
          }
          return_state_u8 = stopMotor;
          break;
        }       
      case stopMotor: 
        {   
          unsigned int speedLen;
           if(!((*motor_Com_Control).motor_Metering_Data.is_Motor_On) || ((*motor_Com_Control).motor_Metering_Data.motor_Status_u16 != 0)) //motor is off already or any fault happen
            { // in unknow situation
              LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6);
              unsigned char speedTx[] = {0x55, 0x00, 0x01, 0x00, 0x00, 0xCC, 0xCC};                     
              speedLen = sizeof(speedTx);
              RingBuf_WriteBlock((*usart1Control_AppLocal).seqMemTX_u32, speedTx, &speedLen);
              unsigned char speedTx1[] = {0x55, 0x00, 0x01, 0x00, 0x00, 0xCC, 0xCC};
              speedLen = sizeof(speedTx1);
              RingBuf_WriteBlock((*usart1Control_AppLocal).seqMemTX_u32, speedTx1, &speedLen); //send stop command  
              tt_StopMotorResume = getSysCount() + MotorStopResumePeriodMax;                 //set wait time delay to max
              return_state_u8 = waitForIdle;
              break;
            }
           else
           {
              if((*usart1Control_AppLocal).motorSpeed_s16 <= (((*motor_Com_Control).motor_Setting.min_Speed_u16) + 10) )   //keep track of slow down
              { //issue final stop command and set time delay
                unsigned char speedTx1[] = {0x55, 0x00, 0x01, 0x00, 0x00, 0xCC, 0xCC};
                speedLen = sizeof(speedTx1);
                RingBuf_WriteBlock((*usart1Control_AppLocal).seqMemTX_u32, speedTx1, &speedLen);
                //RBWriteBlk((*usart1Control_AppLocal).TxPipe->SystemIndex, speedTx1, &speedLen);  //send stop command 
                tt_StopMotorResume = getSysCount() + MotorStopResumePeriod;                 //set normal wait time delay
                
                return_state_u8 = waitForIdle;
                break;
              }
              else
              {
                //if user turn back on before motor speed drop below abs min rpm
                //if(((adc1buf * 10000) / ADC12B ) > (motorOffThreadhold * 100)) 
                if((*motor_Com_Control).motor_Metering_Data.demand_Reference_Speed_u16 >= (*motor_Com_Control).motor_Setting.min_Speed_u16)
                {
                  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);
                  return_state_u8 = SpdUpdate;
                  break; 
                }            
              }
           }
                     
          return_state_u8 = stopMotor; 
          break;
        }
      case waitForIdle:
        {
          if(getSysCount() >= tt_StopMotorResume)                                               //wait for the motor stop period before restart                        
          {
            unsigned char speedTx[] = {0x55, 0x00, 0x03, 0x00, 0x00, 0xCC, 0xCC};                    //Send faluty ack
            unsigned int speedLen = sizeof(speedTx);
            RingBuf_WriteBlock((*usart1Control_AppLocal).seqMemTX_u32, speedTx, &speedLen);
            tt_StopMotorResume = 0;
            (*motor_Com_Control).motor_Metering_Data.is_Motor_On = false;
            
            return_state_u8 = WaitReset;       
            break;
          } 
          return_state_u8 = waitForIdle;
          break;  
        }
      case WaitReset:
        {
          //adc1buf = (uint32_t)(*adc1Control_AppLocal).ADC1result;
          if( ((*motor_Com_Control).motor_Metering_Data.demand_Reference_Speed_u16) < ((*motor_Com_Control).motor_Setting.min_Speed_u16))
          {
             LL_GPIO_ResetOutputPin(GPIOC, LED_ONBOARD_Pin); //on board LED output
             return_state_u8 = AppStart;
             break;
          }
          return_state_u8 = WaitReset;
          break;
        }
      case AppIrq:
        {
          break;
        }               
      case killApp:
        {
          return_state_u8 = AppInit;
          break;
        }
      default:
        return_state_u8 = killApp;   
    }
  return return_state_u8;
}

