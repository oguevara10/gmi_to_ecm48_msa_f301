/**
  ********************************************************************************************************************************
  * @file    ab_module_Mc_StateMachine.h 
  * @author  Pamela Lee
  * @brief   The main motor interface with ST motor libraries.
  * @details This module will keep track with the motor start/stop and running status, and perform startup fail retry, active breaking 
  *          , on the fly startup and derating control for the motor
  ********************************************************************************************************************************
  *     motor settings for this module: MIN_COMMANDABLE_SPEED		300     	// RPM
  *                                     MAX_COMMANDABLE_SPEED		2500                    //MAX_APPLICATION_SPEED_RPM 
  *                                     SPEED_UP_RAMP_RATE		75		// RPM/Sec
  *                                     SPEED_DOWN_RAMP_RATE		100		// RPM/Sec
  *                                     SPEED_CONSIDERED_STOPPED	200		// RPM    
  *                                     
  *
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _AB_MODULE_MC_STATEMACHINE_H_
#define _AB_MODULE_MC_STATEMACHINE_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "main.h"
#include "typedef.h"

#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"
#include "scheduler.h"
#include "ring_buffer.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Setup -----------------------------------------------------------------------------------------------------------------------*/
//static Ram_Buf_Handle Mc_StateMachineStructMem_u32;

typedef enum {
    INIT_MODULE,
    
    IDLE_MODULE,
    
    PRE_START_MODULE,
    OTF_STARTUP_MODULE,
    MOTOR_RUNNING_MODULE,
    
    STOP_MOTOR_MODULE,
    MOTOR_STOPPING_MODULE,
    MOTOR_BRAKING_MODULE,
    
    CURRENT_DERATING_MODULE,
    POWER_DERATING_MODULE,
    TEMPERATURE_DERATING_MODULE,
    
    MOTOR_START_RETRY_MODULE,
    FAULT_REPORT_MODULE,  
    FAULT_PROCESS_MODULE,
    FAULT_WAIT_MODULE,
      
    // additional states to be added here as necessary.
    IRQ_MODULE = DEFAULT_IRQ_STATE,
    STOP_MODULE = KILL_APP
}ModuleMotorStates;



/**
  ********************************************************************************************************************************
  * @brief   Module Mc_StateMachine (inside shared memory)
  * @details 
  * @param   
  * @param   errorCode_u8       Error code of this app module.
  ********************************************************************************************************************************
  */
typedef struct {
   // Ring_Buf_Handle internalPipe_u32;
    int32_t command_Speed;                     //write only     //as percentage 100.000 = 100000 (CW) or -100.000 = -100000 (CCW)will be convert back to rpm
    ModuleMotorStates current_State;                     //read only      //Motor control state machine is currenly in what state 
    uint8_t motorEnable;
    uint8_t errorCode_u8;
    int8_t motorDir;                                   //write only, commanded direction coming from the app-side
} Module_StateMachineControl;

void setSpeed(int32_t target_speed);
void updateAvrCurrent(void);
ModuleMotorStates deratingCheck(void);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _AB_MODULE_MC_STATEMACHINE_H_ */