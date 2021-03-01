/**
********************************************************************************************************************************
* @file    module_Mc_StateMachine.c 
* @author  Pamela Lee
* @brief   The main motor interface with ST motor libraries.
* @details This module will keep track with the motor start/stop and running status, and perform startup fail retry, active breaking 
*          , on the fly startup and derating control for the motor
* @note    when de-rating was happening, the target speed will reduce to below the threshold value, will not go back to the orginial 
*          value until user override the speed with command again!
********************************************************************************************************************************
*/

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "ab_module_Mc_StateMachine.h"
#include "mc_api.h"
#include "state_machine.h"
#include "mc_config.h"
#include "math.h"
#include "zz_module_flash.h"
#include "regal_mc_lib.h"


/* Content ---------------------------------------------------------------------------------------------------------------------*/
extern MCT_Handle_t MCT[NBR_OF_MOTORS];
static MCT_Handle_t *pMCT = &MCT[M1];                          /* pointer on motor control tuning handler */
extern PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS];

extern ProcessInfo processInfoTable[];
Module_StateMachineControl  module_StateMachineControl;

/************************************ All the setting should be mapped into FLash *********************************************/
//__weak __no_init const uint16_t   MIN_COMMANDABLE_SPEED	       @(FLASH_USER_START_ADDR + (2 *  Index_MIN_COMMANDABLE_SPEED	   ) );
__weak const uint16_t   MIN_COMMANDABLE_SPEED	     @(FLASH_USER_START_ADDR + (2 *  Index_MIN_COMMANDABLE_SPEED	) ) = 300       ;             
__weak const uint16_t   MAX_COMMANDABLE_SPEED        @(FLASH_USER_START_ADDR + (2 *  Index_MAX_COMMANDABLE_SPEED        ) ) = 2500      ;             
__weak const uint16_t   SPEED_UP_RAMP_RATE           @(FLASH_USER_START_ADDR + (2 *  Index_SPEED_UP_RAMP_RATE           ) ) = 75        ;             
__weak const uint16_t   SPEED_DOWN_RAMP_RATE         @(FLASH_USER_START_ADDR + (2 *  Index_SPEED_DOWN_RAMP_RATE         ) ) = 100       ;             
__weak const uint16_t   SPEED_CONSIDERED_STOPPED     @(FLASH_USER_START_ADDR + (2 *  Index_SPEED_CONSIDERED_STOPPED     ) ) = 200       ;             
__weak const uint16_t   MotSpinTimeOut               @(FLASH_USER_START_ADDR + (2 *  Index_MotSpinTimeOut               ) ) = 4         ;             
__weak const uint16_t   SpinPollPeriod               @(FLASH_USER_START_ADDR + (2 *  Index_SpinPollPeriod               ) ) = 1000      ;             
__weak const uint16_t   numOfStartRetry              @(FLASH_USER_START_ADDR + (2 *  Index_numOfStartRetry              ) ) = 6         ;             
__weak const uint16_t   StartRetryPeriod             @(FLASH_USER_START_ADDR + (2 *  Index_StartRetryPeriod             ) ) = 2000      ;             
__weak const uint16_t   StartPeriodInc               @(FLASH_USER_START_ADDR + (2 *  Index_StartPeriodInc               ) ) = 10000     ;             
__weak const uint16_t   over_current_threshold       @(FLASH_USER_START_ADDR + (2 *  Index_over_current_threshold       ) ) = 10000      ;             
__weak const uint16_t   over_current_rpm_Reduce      @(FLASH_USER_START_ADDR + (2 *  Index_over_current_rpm_Reduce      ) ) = 10        ;             
__weak const uint16_t   OvCurrent_derate_period      @(FLASH_USER_START_ADDR + (2 *  Index_OvCurrent_derate_period      ) ) = 200       ;             
__weak const uint16_t   over_power_threshold         @(FLASH_USER_START_ADDR + (2 *  Index_over_power_threshold         ) ) = 3000      ;             
__weak const uint16_t   over_power_rpm_Reduce        @(FLASH_USER_START_ADDR + (2 *  Index_over_power_rpm_Reduce        ) ) = 10        ;             
__weak const uint16_t   OvPower_derate_period        @(FLASH_USER_START_ADDR + (2 *  Index_OvPower_derate_period        ) ) = 200       ;             
__weak const uint16_t   over_temperature_threshold   @(FLASH_USER_START_ADDR + (2 *  Index_over_temperature_threshold   ) ) = 60        ;             
__weak const uint16_t   over_temperature_rpm_Reduce  @(FLASH_USER_START_ADDR + (2 *  Index_over_temperature_rpm_Reduce  ) ) = 10        ;             
__weak const uint16_t   OvTemp_derate_period         @(FLASH_USER_START_ADDR + (2 *  Index_OvTemp_derate_period         ) ) = 30000     ;      


// Application Constants

//#define MIN_COMMANDABLE_SPEED		300     	// RPM
//#define MAX_COMMANDABLE_SPEED		2500                    //MAX_APPLICATION_SPEED_RPM 
//#define SPEED_UP_RAMP_RATE		75		// RPM/Sec
//#define SPEED_DOWN_RAMP_RATE		100		// RPM/Sec
//#define SPEED_CONSIDERED_STOPPED	200		// RPM              

int32_t target_speed = 0;                               //Actural speed to submit into ST motor libaries                        
int16_t act_dir = 1;
/************************ Motor start spinning timing parameter  ********************************************************/
static uint16_t MotSpinPollCount = 0;
//#define MotSpinTimeOut 4                        //max motor spin poll count for time out measurement
//#define SpinPollPeriod 1000                     //time period for checking and sending 0-10V and speed data to motor board
uint64_t tt_SpinPollTime;

/**********************************   Start-up retry  *******************************************/                           
/***  total re-start waiting period = StartRetryPeriod +(StartPeriodInc * numOfStartRetry) ms  **/
//#define numOfStartRetry   6                     //number of retry when start-up fail
//#define StartRetryPeriod  2000                  //fundamental delay time for start-up retry delay
//#define StartPeriodInc    10000                 //Each number of failure additional time delay        
uint16_t StartRetryCounter = 0;
uint64_t tt_StartUpRetryTime;
uint64_t tt_FaultOTFWaitTime_u64 = 0;
#define OTF_WAIT_TIME 30000  // 30 sec wait time to allow motor to coast down to 0 RPM

/************************************* de_rating parameters  *********************************************/
//#define over_current_threshold         1000              //over current theardhold (need convert to actual value!!!!!!!!!!!!)
//#define over_current_rpm_Reduce         10              //rpm reduce in every loop of derating
//#define OvCurrent_derate_period         200             //over current derate poll time duration ms
uint64_t tt_derateCurrentPollTime;
int32_t avrCurrentRd[]={0,0,0,0};
int32_t avrCurrentRdOP = 0;


//#define over_power_threshold           3000
//#define over_power_rpm_Reduce           10              //rpm reduce in every loop of derating
//#define OvPower_derate_period           200             //over power derate poll time duration ms
uint64_t tt_deratePowerPollTime;

//#define over_temperature_threshold      33              //Actural temperature in degree C
//#define over_temperature_rpm_Reduce     10              //rpm reduce in every loop of derating
//#define OvTemp_derate_period            30000             //over power derate poll time duration ms
uint64_t tt_derateTempPollTime;

//RPa: Stop and Brake
uint64_t tt_TurnOnLowSideTime;

//RPa: OTF temporary fix
uint64_t tt_FaultOTFWaitTime;

/****************** local fault status ************************/
/** GMI_FaultStatus => 0x01 = start-up retry error            */
/**                    0x02 = ST_Motor fault error persisting */
uint8_t GMI_FaultStatus = 0; //GMI statemachine Error message status for report to module_err_logHandle      

int16_t tmpryTempature = 0;


uint8_t module_Mc_StateMachine_u32(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8,
                                   uint8_t irq_id_u8) 
{
  uint8_t return_state_u8 = 0;
  /** pre-process the Motor stop speed command or fault status, both get higher priority in the state machine**/
  updateAvrCurrent();
  if((!module_StateMachineControl.motorEnable) && (next_State_u8 != INIT_MODULE)){
    module_StateMachineControl.command_Speed = 0; //disable the motor
    return_state_u8 = IDLE_MODULE;  
  }
  if( MC_GetSTMStateMotor1() == FAULT_NOW) {
    GMI_FaultStatus = 0x02;              //ST_Motor fault error persisting
    next_State_u8 = FAULT_REPORT_MODULE; //report fault and return
  }
  else if(MC_GetSTMStateMotor1() == FAULT_OVER) next_State_u8 = FAULT_PROCESS_MODULE; //after fault over then can process the fault and restart
  else if((next_State_u8 != INIT_MODULE)&& (next_State_u8 != IRQ_MODULE))
  {
	volatile uint8_t	entry_u8 = 0;
    if(module_StateMachineControl.command_Speed == 0) {   // any situation see stop command will change to stop state, unless IRQ and stopping states
      switch (next_State_u8){
	  case PRE_START_MODULE: {
		entry_u8 = 1;
        next_State_u8 = STOP_MOTOR_MODULE;		
		break;
	  }
	  case OTF_STARTUP_MODULE: {
		entry_u8 = 2;
        next_State_u8 = STOP_MOTOR_MODULE;		
		break;
	  }
	  case MOTOR_RUNNING_MODULE: {
		entry_u8 = 3;
        next_State_u8 = STOP_MOTOR_MODULE;		
		break;
	  }
	  case CURRENT_DERATING_MODULE: {
		entry_u8 = 4;
        next_State_u8 = STOP_MOTOR_MODULE;		
		break;
	  }
	  case POWER_DERATING_MODULE: {
		entry_u8 = 5;
        next_State_u8 = STOP_MOTOR_MODULE;		
		break;
	  }
	  case TEMPERATURE_DERATING_MODULE: {
		entry_u8 = 6;
        next_State_u8 = STOP_MOTOR_MODULE;		
		break;
	  }
      default:
        break;
      } 
    }
    else {  
      if(module_StateMachineControl.command_Speed < MIN_COMMANDABLE_SPEED) module_StateMachineControl.command_Speed = MIN_COMMANDABLE_SPEED; //check command speed lower than min speed
      if(module_StateMachineControl.command_Speed > MAX_COMMANDABLE_SPEED) module_StateMachineControl.command_Speed = MAX_COMMANDABLE_SPEED; //check command speed lower than min speed       
    }
  }
  module_StateMachineControl.current_State = (ModuleMotorStates)next_State_u8;
  /** pre-process the Motor stop speed command or fault status, both get higher priority in the state machine end**/
  
  switch (next_State_u8) {
  case INIT_MODULE: {          
    //init this module 
    Mc_StateMachineStructMem_u32 =  StructMem_CreateInstance(MODULE_MC_STATEMACHINE, sizeof(Module_StateMachineControl), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a structured memory for this driver [should map it back to this driver local struct]
    (*Mc_StateMachineStructMem_u32).p_ramBuf_u8 = (uint8_t *)&module_StateMachineControl ; //map the generated module's control memory into the structured memory
    uint8_t module_Mc_StateMachine_Index = getProcessInfoIndex(MODULE_MC_STATEMACHINE);
    processInfoTable[module_Mc_StateMachine_Index].Sched_ModuleData.p_masterSharedMem_u32 = Mc_StateMachineStructMem_u32; //also map it back to module_paramters under kernel 
    
    //init motor from boot 
    /*** check the motor setting and init all motor setting ***/            
    module_StateMachineControl.command_Speed = 0;            
    module_StateMachineControl.errorCode_u8 = 0;
    module_StateMachineControl.motorDir = 1; //CW
    module_StateMachineControl.motorEnable = TRUE;
    //init motor flag
    return_state_u8 = IDLE_MODULE;
    break;
  }
  case IDLE_MODULE: {
    if(module_StateMachineControl.command_Speed != 0)
    {
      return_state_u8 = PRE_START_MODULE;
      break;
    }
    StartRetryCounter = 0;        //this variable combine with StartRetryPeriod to form the timeout of start retry
    MotSpinPollCount = 0;         //this value combine the startRetryTime delay to form the timeout 
    return_state_u8 = IDLE_MODULE;
    break;
  }
  case PRE_START_MODULE: {        //calculate the actual execution time according to the current motor speed and the target speed in RPM
    target_speed = module_StateMachineControl.command_Speed;
            setSpeed(module_StateMachineControl.command_Speed * (int32_t) module_StateMachineControl.motorDir);                                                                 //Command ST motor libraries for running the speed of module_StateMachineControl.targetSpeed
            MC_StartMotor1();
            act_dir = MC_GetImposedDirectionMotor1();
            // Temporary integrator limits to avoid regen when decelerating, a dc bus control method will be implemented soon
            if (act_dir == 1)
            {
              PIDSpeedHandle_M1.hLowerOutputLimit=0;
              PIDSpeedHandle_M1.wLowerIntegralLimit=0;
              PIDSpeedHandle_M1.wUpperIntegralLimit = (int32_t)A_IQMAX * (int32_t)SP_KIDIV;
              PIDSpeedHandle_M1.hUpperOutputLimit = (int16_t)A_IQMAX;              
            }
            else if (act_dir == -1)
            {
              PIDSpeedHandle_M1.hUpperOutputLimit=0;
              PIDSpeedHandle_M1.wUpperIntegralLimit=0;          
              PIDSpeedHandle_M1.wLowerIntegralLimit = -(int32_t)A_IQMAX * (int32_t)SP_KIDIV;
              PIDSpeedHandle_M1.hLowerOutputLimit = -(int16_t)A_IQMAX;
            }
    MotSpinPollCount = 0;                               //Reset motor spinning loop count as timeout counter
    tt_SpinPollTime = getSysCount() + SpinPollPeriod;   //prepare next time tick value for OTF_STARTUP_MODULE
    return_state_u8 = OTF_STARTUP_MODULE;
    break;
  }
  case OTF_STARTUP_MODULE: {      //monitor the startup process and check motor is running successfully
    // startup successfully proof 
    //ST motor state in either START_RUN or RUN, no fault state, measured speed normal, 
    if(MC_GetOccurredFaultsMotor1()){  //if any fault happen
      MC_StopMotor1();                 //stop the motor first
      MC_AcknowledgeFaultMotor1();     //clear the fault
      MotSpinPollCount = 0;            //this value combine the startRetryTime delay to form the timeout 
      tt_StartUpRetryTime = getSysCount() + StartRetryPeriod +(StartPeriodInc * StartRetryCounter);
      return_state_u8 = MOTOR_START_RETRY_MODULE;
      break;
    }    
    State_t tmpryMC_State = MC_GetSTMStateMotor1();
    if (getSysCount() >= tt_SpinPollTime) {        //wait for motor spin up to speed   
      if(MotSpinPollCount++ >= MotSpinTimeOut){    //motor spin-up time-out, timeout period = SpinPollPeriod * MotSpinTimeOut
        MC_StopMotor1();                           //stop the motor first
        MC_AcknowledgeFaultMotor1();               //clear the fault
        MotSpinPollCount = 0;                      //this value combine the startRetryTime delay to form the timeout 
        tt_StartUpRetryTime = getSysCount() + StartRetryPeriod +(StartPeriodInc * StartRetryCounter);
        return_state_u8 = MOTOR_START_RETRY_MODULE;   
        break;
      } 
      tt_SpinPollTime = getSysCount() + SpinPollPeriod;           //update next time tick value 
    }
    if((tmpryMC_State == START_RUN) || ( tmpryMC_State == RUN)){  //ST motor libries in correct state?
      if(abs((MC_GetMecSpeedAverageMotor1() * 6 * act_dir) - target_speed) <= ((abs(target_speed) *8)/10)) {                 //check motor actual running at more than +-20% of target speed
        MotSpinPollCount = 0;    
        StartRetryCounter = 0;    
        return_state_u8 = MOTOR_RUNNING_MODULE;
        break;   
      }   
    }
    return_state_u8 = OTF_STARTUP_MODULE;
    break;
  }
  
  case MOTOR_RUNNING_MODULE: {                  //keep tracking over_temperature, over_current, over_power
    uint8_t recent_derateCheck = deratingCheck();
    return_state_u8 = MOTOR_RUNNING_MODULE;
    if (recent_derateCheck == CURRENT_DERATING_MODULE){
      tt_derateCurrentPollTime = getSysCount(); //prepare next time tick value for CURRENT_DERATING_MODULE
      return_state_u8 = CURRENT_DERATING_MODULE;
    }
    if (recent_derateCheck ==  POWER_DERATING_MODULE){
      tt_deratePowerPollTime = getSysCount();  //prepare next time tick value for POWER_DERATING_MODULE
      return_state_u8 = POWER_DERATING_MODULE;
    }
    if (recent_derateCheck == TEMPERATURE_DERATING_MODULE){
      tt_derateTempPollTime = getSysCount();  //prepare next time tick value for TEMPATURE_DERATING_MODULE  
      return_state_u8 = TEMPERATURE_DERATING_MODULE;
    } 
    
    // May need to check motor running status where  running properly 
    /*        if(abs((MC_GetMecSpeedAverageMotor1() * 6) - target_speed) > ((target_speed *5)/10)) { //check motor actual running lower than within +-50% of target speed
    module_StateMachineControl.command_Speed = 0;
    next_State_u8 = STOP_MOTOR_MODULE;   //assume this is an abnormal operation, only stop motor at this stage.
    break;   
  }   */
    //if can run to this line means no de-rating happen, then will update the new speed 
          if (module_StateMachineControl.motorDir != act_dir)
          {
            module_StateMachineControl.command_Speed = 0;
            return_state_u8 = STOP_MOTOR_MODULE;           
          }
          else
          {
            if(target_speed != module_StateMachineControl.command_Speed){   
              
              target_speed = module_StateMachineControl.command_Speed;
              setSpeed(module_StateMachineControl.command_Speed * (int32_t) act_dir);   
            }
          }
    break;
  }
  
  
  //derating feature/s will use the higher priority one to follow (over-temperature, over-current, or fire-mode)
  case CURRENT_DERATING_MODULE: { 
    uint32_t tmpryDrate = -(uint32_t)over_current_rpm_Reduce;
    return_state_u8 = CURRENT_DERATING_MODULE;
    if (target_speed > module_StateMachineControl.command_Speed)
    {
        if (module_StateMachineControl.motorDir == act_dir)//collinear
        {
          target_speed = module_StateMachineControl.command_Speed;
          setSpeed(module_StateMachineControl.command_Speed * (int32_t) act_dir);   
        }
        else
        {
          module_StateMachineControl.command_Speed = 0;
          return_state_u8 = STOP_MOTOR_MODULE;
        }   
      }
    
    if(deratingCheck() != CURRENT_DERATING_MODULE){ //check the same de-rating is presist 
      if( target_speed < module_StateMachineControl.command_Speed) tmpryDrate = -tmpryDrate; //increase speed after de_rate condition over
      else {
          if (module_StateMachineControl.motorDir == act_dir)//collinear
          {
            target_speed = module_StateMachineControl.command_Speed;
            setSpeed(module_StateMachineControl.command_Speed * (int32_t) act_dir);  
            return_state_u8 = MOTOR_RUNNING_MODULE;
          }
          else
          {
            module_StateMachineControl.command_Speed = 0;
            return_state_u8 = STOP_MOTOR_MODULE;
          }                                             //after speed get back to command then jump out       
        }
      }
      if (getSysCount() >= tt_derateCurrentPollTime) {                                                    //wait for motor spin down to reach the lower threshold
        target_speed += tmpryDrate;                                                                         //reduce the speed 
        if (target_speed < MIN_COMMANDABLE_SPEED)
        {
          module_StateMachineControl.command_Speed = 0;   
          return_state_u8 = STOP_MOTOR_MODULE;// Stop the motor and go back to IDLE
        }            
        setSpeed(target_speed * (int32_t) act_dir);                                                                           //Command ST motor libraries for running the speed of target_speed
      tt_derateCurrentPollTime = getSysCount() + OvCurrent_derate_period; //prepare next time tick value for CURRENT_DERATING_MODULE 
    }
    break;
  }
  //derating feature/s will use the higher priority one to follow (over-temperature, over-current, or fire-mode)
  case POWER_DERATING_MODULE: { 
    uint32_t tmpryDrate = -(uint32_t)over_power_rpm_Reduce;
    return_state_u8 = POWER_DERATING_MODULE;
    if (target_speed > module_StateMachineControl.command_Speed)
    {
        if (module_StateMachineControl.motorDir == act_dir)//collinear
        {
          target_speed = module_StateMachineControl.command_Speed;
          setSpeed(module_StateMachineControl.command_Speed * (int32_t) act_dir);   
        }
        else
        {
          module_StateMachineControl.command_Speed = 0;
          return_state_u8 = STOP_MOTOR_MODULE;
        }   
      }
      
      if(deratingCheck() != POWER_DERATING_MODULE){                                                     //check the same de-rating is presist 
        if( target_speed < module_StateMachineControl.command_Speed) tmpryDrate = -tmpryDrate;               //increase speed after de_rate condition over
        else {
          if (module_StateMachineControl.motorDir == act_dir)//collinear
          {
            target_speed = module_StateMachineControl.command_Speed;
            setSpeed(module_StateMachineControl.command_Speed * (int32_t) act_dir);  
            return_state_u8 = MOTOR_RUNNING_MODULE;
          }
          else
          {
            module_StateMachineControl.command_Speed = 0;
            return_state_u8 = STOP_MOTOR_MODULE;
          }                                             //after speed get back to command then jump out
          
        }
      }
      if (getSysCount() >= tt_deratePowerPollTime) {                                                        //wait for motor spin down to reach the lower threshold  
        target_speed += tmpryDrate;                                                                         //reduce the speed 
        if (target_speed < MIN_COMMANDABLE_SPEED)
        {
          module_StateMachineControl.command_Speed = 0;   
          return_state_u8 = STOP_MOTOR_MODULE;// Stop the motor and go back to IDLE
        }
        setSpeed(target_speed * (int32_t) act_dir);    
        tt_deratePowerPollTime = getSysCount() + OvPower_derate_period;                                     //prepare next time tick value for POWER_DERATING_MODULE
      }
      break;
    }
    //derating feature/s will use the higher priority one to follow (over-temperature, over-current, or fire-mode)
    case TEMPERATURE_DERATING_MODULE: {  
      uint32_t tmpryDrate = -(uint32_t)over_temperature_rpm_Reduce;
      return_state_u8 = TEMPERATURE_DERATING_MODULE;
      if ((target_speed > module_StateMachineControl.command_Speed))
      {
        if(module_StateMachineControl.motorDir == act_dir)
        {
          target_speed = module_StateMachineControl.command_Speed;
          setSpeed(module_StateMachineControl.command_Speed * (int32_t) act_dir); 
        }
        else
        {
          module_StateMachineControl.command_Speed = 0;
          return_state_u8 = STOP_MOTOR_MODULE;
        } 
      }
      
      if(deratingCheck() != TEMPERATURE_DERATING_MODULE) {                                                     //check the same de-rating is presist 
        if( target_speed < module_StateMachineControl.command_Speed) tmpryDrate = -tmpryDrate;               //increase speed after de_rate condition over
        else {
          if (module_StateMachineControl.motorDir == act_dir)//collinear
          {
            target_speed = module_StateMachineControl.command_Speed;
            setSpeed(module_StateMachineControl.command_Speed * (int32_t) act_dir); 
            return_state_u8 = MOTOR_RUNNING_MODULE;
          }
          else
          {
            module_StateMachineControl.command_Speed = 0;
            return_state_u8 = STOP_MOTOR_MODULE;
          }                                         //after speed get back to command then jump out
          
        }
      }
      if (getSysCount() >= tt_derateTempPollTime) {                                                         //wait for motor spin down to reach the lower threshold           
        target_speed += tmpryDrate;                                                                         //reduce the speed 
        if ((target_speed < MIN_COMMANDABLE_SPEED))
        {
          module_StateMachineControl.command_Speed = 0;   
          return_state_u8 = STOP_MOTOR_MODULE;// Stop the motor and go back to IDLE
        }
        setSpeed(target_speed * (int32_t) act_dir);   
        tt_derateTempPollTime = getSysCount() + OvTemp_derate_period;                                     //prepare next time tick value for TEMPATURE_DERATING_MODULE  ;    
      }          
    break;
  }
  
  //Error retry handling
  case MOTOR_START_RETRY_MODULE: {
    if (getSysCount() >= tt_StartUpRetryTime) {  //wait for motor spin up to speed
      if( StartRetryCounter++ < numOfStartRetry -1){
        return_state_u8 = PRE_START_MODULE;
        break;
      }  
      else
      {
        StartRetryCounter = 0;
        module_StateMachineControl.command_Speed = 0;
        GMI_FaultStatus = 0x01;   //start-up retry error
        return_state_u8 = FAULT_PROCESS_MODULE;
        break;     
      }
    }      
    return_state_u8 = MOTOR_START_RETRY_MODULE;
    break;
  }
  
  //Error report 
  case FAULT_PROCESS_MODULE: {
    //     if( setting status ){  //Fault command will be issued according to user setting!!
    MC_AcknowledgeFaultMotor1();  //CLear ST motor libraries fault status
    //     }
    MotSpinPollCount = 0;         //Reset motor spinning loop count as timeout counter
    return_state_u8 = FAULT_REPORT_MODULE;
    break;
  }
  
  case FAULT_REPORT_MODULE: {
    setupSoftwareIRQ(module_id_u8, MODULE_ERR_LOGHANDLE, 0xEF, GMI_FaultStatus, 0x00, NULL);  //Note: in FAULT_NOW situation will continue to issue fault to error-module 
    tt_FaultOTFWaitTime_u64 = getSysCount() + OTF_WAIT_TIME; // Motor need to coast down to 0RPM before retry
    return_state_u8 = FAULT_WAIT_MODULE;
    break;
  }      
  
  case FAULT_WAIT_MODULE: { // Wait for fan to stop before restart
    if(getSysCount() >= tt_FaultOTFWaitTime_u64)
    {
      return_state_u8 = IDLE_MODULE;
    } else
    {
      return_state_u8 = FAULT_WAIT_MODULE; 
    }
    break;    
  }
  
  //Motor stop
  case STOP_MOTOR_MODULE: {
    //check motor current situation to perform motor stop sequency
    MC_StopMotor1();
    return_state_u8 = MOTOR_STOPPING_MODULE;
    break;
  }
  case MOTOR_STOPPING_MODULE: {
    if (BrakeHandle_M1.BrakingPhase == STARTRAMP)
    {
        tt_TurnOnLowSideTime = getSysCount() + 2000;
        return_state_u8 = MOTOR_BRAKING_MODULE;
        break;
      }   
      else{
        return_state_u8 = MOTOR_STOPPING_MODULE;
        break;
      }
      
      break;
    }
    case MOTOR_BRAKING_MODULE: {
      if (getSysCount()>= tt_TurnOnLowSideTime)
      {
        return_state_u8 = IDLE_MODULE;
        break;
      }
      else{
        return_state_u8 = MOTOR_BRAKING_MODULE;
        break;
      }
      break; 
    }             
  
  case IRQ_MODULE: {
    return_state_u8 = IDLE_MODULE;
    break;
  }
  case STOP_MODULE: {
    return_state_u8 = IDLE_MODULE;
    break;
  }
  default: {
    return_state_u8 = IDLE_MODULE;
    break;
  }
  }
  return return_state_u8;
}

void setSpeed(int32_t target_speed){
  int32_t current_speed = MC_GetMecSpeedAverageMotor1() * 6;   //convert to rpm
  uint32_t speed_ramp_up_duration = (abs(current_speed - target_speed ) * 1000) / SPEED_UP_RAMP_RATE;
  MC_ProgramSpeedRampMotor1( target_speed / 6, speed_ramp_up_duration );    
}

//uint8_t AvrCurrentIndx = 1;
uint8_t AvrCurrentIndx = 0;

void updateAvrCurrent(void){  //average current for fault monitor 
  qd_t Iqd = pMPM[M1]->pFOCVars->Iqd;
  avrCurrentRd[AvrCurrentIndx] = ( int32_t )(sqrt((( int32_t )Iqd.q * (int32_t) Iqd.q)+ (( int32_t )Iqd.d * (int32_t) Iqd.d)));
  AvrCurrentIndx++;
  if(AvrCurrentIndx == 4) {
    avrCurrentRdOP = abs((avrCurrentRd[0] + avrCurrentRd[1] + avrCurrentRd[2] + avrCurrentRd[3])/4);
    AvrCurrentIndx = 0;
  }
}

ModuleMotorStates deratingCheck(void){
  if( avrCurrentRdOP >= over_current_threshold) return (CURRENT_DERATING_MODULE);                        //check for over current de-rating
  else if( MPM_GetAvrgElMotorPowerW(pMCT->pMPM) >= over_power_threshold) return (POWER_DERATING_MODULE); //check for over power de-rating
  else if( (tmpryTempature = NTC_GetAvTemp_C(pMCT->pTemperatureSensor)) >= over_temperature_threshold) return (TEMPERATURE_DERATING_MODULE); //check for over temperature de-rating
  return ((ModuleMotorStates)IDLE_MODULE);
}

