/**
  ********************************************************************************************************************************
  * @file    regal_mc_lib.c
  * @author  Roel Pantonial
  * @brief   This source file contains the regal motor control library
  * @details This file has declarations for motor control algorithms such as on-the-fly and non-regenerative braking
  ********************************************************************************************************************************
  */

#include "drive_parameters.h"
#include "regal_mc_lib.h"
#include "bus_voltage_sensor.h"
#include "mc_math.h"
#include "mc_api.h"
#include "mc_config.h"

/*Private Variables */
static uint8_t Imax_count = 0;
static uint16_t LowSide_count = 0;
/**
  * @brief  PI / PID Bus Voltage parameters Motor 1
  */
//RPa
PID_Handle_t PIDBkHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_BRAKE_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_BRAKE_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)100 * (int32_t)BK_KIDIV, //GMI 200
  .wLowerIntegralLimit = -(int32_t)100 * (int32_t)BK_KIDIV, //GMI: -200
  .hUpperOutputLimit       = 0, //Rpa: a factor of the OVP, temp is for 60V
  .hLowerOutputLimit       = -2000,//RPa: a factor of the UVP, temp is for 60V, GMI -3000
  .hKpDivisor          = (uint16_t)BK_KPDIV,
  .hKiDivisor          = (uint16_t)BK_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)BK_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)BK_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Imax Controller parameters Motor 1
  */
//RPa 
PID_Handle_t PIDImHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_IMAX_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_IMAX_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)2000 * (int32_t)IMAX_KIDIV,
  .wLowerIntegralLimit = -(int32_t)2000 * (int32_t)IMAX_KIDIV,
  .hUpperOutputLimit       = 500,//INT16_MAX,
  .hLowerOutputLimit       = -500,//RPa: make sure that this is more than -Flux/Ld,
  .hKpDivisor          = (uint16_t)IMAX_KPDIV,
  .hKiDivisor          = (uint16_t)IMAX_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)IMAX_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)IMAX_KIDIV_LOG,
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  Brake parametrs Motor 1
  */
Braking_Handle_t BrakeHandle_M1 =
{
  .Nbar = 205,
  .BrakingPhase = STARTRAMP,
  .IMax_Ref = 500,// GMI: 2000
  .FeedForward_term = 0,
  .Vbus_Add = 20, //GMI: 60, Blender: 40.
};


/**
  * @brief Execution of DC Bus Voltage Control
*/
int32_t FOC_BusVoltageControlM1(Braking_Handle_t * pHandle, PID_Handle_t * pPIDHandle, RDivider_Handle_t * pBSHandle)
{
  int32_t hVBusError;
  uint16_t BusVoltCheck_16bit;
  int32_t BrakeTorque_q;
  
  // Compute Error given BusVoltageRef and measured bus voltage in 16bit representation
  BusVoltCheck_16bit = VBS_GetAvBusVoltage_d(&(pBSHandle->_Super));
  hVBusError = (int32_t) BusVoltCheck_16bit - (int32_t) (((uint32_t)pHandle->Adapt_BusVoltageRef*FP16)/((uint32_t)(pBSHandle->_Super.ConversionFactor)));
  
  // Check Direction
    int16_t dir = MC_GetImposedDirectionMotor1();
  // PI Control
  BrakeTorque_q = (int32_t)PI_Controller(pPIDHandle, hVBusError) * dir;
    
  return(BrakeTorque_q);
}

/**
  * @brief Execution of maximum current control
*/
int32_t FOC_ImaxCurrentControllerM1(Braking_Handle_t * pHandle, PID_Handle_t * pPIDHandle, FOCVars_t * pFOCHandle_t)
{
  int32_t CurrentLoss;
  int32_t hCurrentError;
  uint32_t current_meas;
  
  // Compute Error given with current reference and the magnitude of the current vector
  current_meas = (uint32_t)(pFOCHandle_t->Iqd.d * pFOCHandle_t->Iqd.d) + (uint32_t) (pFOCHandle_t->Iqd.q * pFOCHandle_t->Iqd.q);
  current_meas = MCM_Sqrt( (int32_t) current_meas);
  hCurrentError =   (int32_t) current_meas - (int32_t) pHandle->IMax_Ref;
  // PI Control
  CurrentLoss = PI_Controller(pPIDHandle, hCurrentError);
  
  //Feedforward term
  pHandle->FeedForward_term = ((int32_t)pHandle->Nbar * (int32_t) pHandle->IMax_Ref)>>BYTE_SHIFT;
  
  //Output_calculation
  CurrentLoss = CurrentLoss - pHandle->FeedForward_term;
  
  //RPa: an insurance that d-current will never get to the IV quadrant
  if (CurrentLoss > 0)
    CurrentLoss = 0;
  
  return(CurrentLoss);
}

/**
  * @brief Initialisation of Braking structure
*/
void BrakingStruct_Init(Braking_Handle_t * pHandle, SpeednTorqCtrl_Handle_t * pSTCHandle)
{
  pHandle->rMeasuredSpeed = SPD_GetAvrgMecSpeedUnit( pSTCHandle->SPD );
  //RPa: take the absolute value of speed measure
  pHandle->Adapt_IMax = (int32_t)((RAMP_a * (int32_t) pHandle->rMeasuredSpeed * (int32_t) pHandle->rMeasuredSpeed)>>BYTE_SHIFT) + \
    (int32_t)(RAMP_b * (int32_t)pHandle->rMeasuredSpeed) + RAMP_c;
}

/**
  * @brief Motor Braking State Machine
*/
void MotorBraking_StateMachine(Braking_Handle_t * pBkHandle, PID_Handle_t * pPIDBusHandle, PID_Handle_t * pPIDImHandle, SpeednTorqCtrl_Handle_t * pSTCHandle, FOCVars_t * pFOCHandle, RDivider_Handle_t * pBSHandle)
{
     //RPa controlled way to set the next Iq and Id
    pBkHandle->rMeasuredSpeed = SPD_GetAvrgMecSpeedUnit( pSTCHandle->SPD );
    //RPa: take the absolute value of speed measure  
    int16_t MeasuredSpeedAbsValue = (pBkHandle->rMeasuredSpeed < 0 ? (-pBkHandle->rMeasuredSpeed): (pBkHandle->rMeasuredSpeed));
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //RPa: IMax trajectory is controlled to always be within the motor loss ellipse (copper+iron losses)
  if (Imax_count >= 2) //RPa: does an IMax trajectory sampling every 2msec but sampling can be increased
  {
    pBkHandle->Adapt_IMax = (int32_t)((RAMP_a * (int32_t) MeasuredSpeedAbsValue * (int32_t) MeasuredSpeedAbsValue)>>BYTE_SHIFT) + \
      (int32_t)(RAMP_b * (int32_t)MeasuredSpeedAbsValue) + RAMP_c;
    Imax_count = 0;
  }
  Imax_count++;
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // RPa: State Machine for the d current injection
  switch(pBkHandle->BrakingPhase)
  {
  case STARTRAMP:   
    if (MeasuredSpeedAbsValue >= SPEED_TRANSITION)
      pBkHandle->BrakingPhase = RAMPUP;
    else
      pBkHandle->BrakingPhase = RAMPDOWN;
     Imax_count = 0;   
    FOCStop_CalcCurrRef( pBkHandle, pPIDBusHandle, pPIDImHandle, pFOCHandle, pBSHandle); 
    break;
    
  case RAMPUP:   
    if (pBkHandle->IMax_Ref >=  pBkHandle->Adapt_IMax)
    {
      // Always compute the I-maximum reference to be within the copper loss ellipse
      pBkHandle->IMax_Ref =  pBkHandle->Adapt_IMax;
      pBkHandle->BrakingPhase = STEADYSTATE;
    }
    else
    {
      pBkHandle->IMax_Ref += RAMP_STEP;
    }
    //RPa: can be placed the next lines into a function as this is common with steadystate
    if (MeasuredSpeedAbsValue < SPEED_TRANSITION)
    {
      pBkHandle->BrakingPhase = RAMPDOWN;
      pPIDImHandle->hLowerOutputLimit = -5;
      pPIDImHandle->hUpperOutputLimit = 5;
    }   
    //Calling the Iq and Id injection for controlled braking
    FOCStop_CalcCurrRef( pBkHandle, pPIDBusHandle, pPIDImHandle, pFOCHandle, pBSHandle); 
    break;
    
  case STEADYSTATE:
    pBkHandle->IMax_Ref =  pBkHandle->Adapt_IMax;
    
    //RPa: can be placed the next lines into a function as this is common with rampup
    if (MeasuredSpeedAbsValue < SPEED_TRANSITION)
    {
      pBkHandle->BrakingPhase = RAMPDOWN;
      // Limit the boundaries of the Id injection when ramping down
      pPIDImHandle->hLowerOutputLimit = -5;
      pPIDImHandle->hUpperOutputLimit = 5;
    } 
    //Calling the Iq and Id injection for controlled braking
    FOCStop_CalcCurrRef( pBkHandle, pPIDBusHandle, pPIDImHandle, pFOCHandle, pBSHandle);  
    break;
    
  case RAMPDOWN:
      if (pBkHandle->IMax_Ref <=  RAMPEND_CURRENT) // go to Iq Hold state when Id injection is at the end of a specified minimum
      {
        pBkHandle->FilteredSpeed = MeasuredSpeedAbsValue;   
        pBkHandle->IMax_Ref =  RAMPEND_CURRENT;
        pBkHandle->BrakingPhase = LOWSPEED_IQHOLD;
      }
      else
      {
        pBkHandle->IMax_Ref -= RAMP_STEP ;
      }
      //Calling the Iq and Id injection for controlled braking
      FOCStop_CalcCurrRef( pBkHandle, pPIDBusHandle, pPIDImHandle, pFOCHandle, pBSHandle); 
    break;
    
  case LOWSPEED_IQHOLD:
    //RPa: Low-Pass Filtering of Speed measurement
    pBkHandle->FilteredSpeed = (int16_t)((((256 - (int32_t)alpha_br)*(int32_t)MeasuredSpeedAbsValue) + ((int32_t)alpha_br*(int32_t)pBkHandle->FilteredSpeed) + 128)>>BYTE_SHIFT) ;
    //Calling the Iq and Id injection for controlled braking
    FOCStop_CalcCurrRef( pBkHandle, pPIDBusHandle, pPIDImHandle, pFOCHandle, pBSHandle); 
    
    if (pBkHandle->FilteredSpeed < BRAKING_ENDSPEED)//RPa: (MeasuredSpeedAbsValue < BRAKING_ENDSPEED)
    {
      LowSide_count = 0;
      R3_1_SwitchOffPWM( &PWM_Handle_M1._Super );
      R3_1_TurnOnLowSides( &PWM_Handle_M1._Super );// Turning the low-side on when speed is lower than a specified threshold
      pBkHandle->BrakingPhase=TURNONLOWSIDE;
    }   
    break;   

  case TURNONLOWSIDE:
    
    if (LowSide_count>1000)//RPa: Low-side turned on for a specified amount of time; this can be tuned later on for the application (or not even turned off at all)
    {
      pBkHandle->BrakingPhase=STARTRAMP;
    }
    LowSide_count++;
    break;
    
  default:
    
    break;
  }
}

/**
  * @brief Current Reference calculation for non-regenerative braking
*/
void FOCStop_CalcCurrRef(Braking_Handle_t * pBrakeHandle, PID_Handle_t * pPIDBusHandle, PID_Handle_t * pPIDImHandle, FOCVars_t * pFOCHandle_t, RDivider_Handle_t * pBSHandle)
{
  if(pFOCHandle_t->bDriveInput == INTERNAL)
  {
    pFOCHandle_t->Iqdref.q = FOC_BusVoltageControlM1(pBrakeHandle, pPIDBusHandle, pBSHandle); 
    pFOCHandle_t->Iqdref.d = FOC_ImaxCurrentControllerM1(pBrakeHandle, pPIDImHandle, pFOCHandle_t );
  }
}

/**
  * @brief non-regenerative braking when motor is at running state
*/
void RegenControlM1(Braking_Handle_t * pBkHandle, PID_Handle_t * pPIDBusHandle, PID_Handle_t * pPIDSpeedHandle, SpeednTorqCtrl_Handle_t * pSTCHandle, RDivider_Handle_t * pBSHandle)
{
    if ((MC_GetImposedDirectionMotor1()==1)&&((pSTCHandle->SPD->hAvrMecSpeedUnit - pSTCHandle->TargetFinal) > 5))
    {//FWD direction
      pPIDSpeedHandle->hLowerOutputLimit = FOC_BusVoltageControlM1(pBkHandle, pPIDBusHandle, pBSHandle); 
      pPIDSpeedHandle->wLowerIntegralLimit = (int32_t) pPIDSpeedHandle->hLowerOutputLimit * (int32_t)SP_KIDIV;
    }
    else if ((MC_GetImposedDirectionMotor1()==-1)&&((pSTCHandle->SPD->hAvrMecSpeedUnit - pSTCHandle->TargetFinal) < -5))
    {//REV direction
      pPIDSpeedHandle->hUpperOutputLimit = FOC_BusVoltageControlM1(pBkHandle, pPIDBusHandle, pBSHandle);
      pPIDSpeedHandle->wUpperIntegralLimit = (int32_t) pPIDSpeedHandle->hUpperOutputLimit * (int32_t)SP_KIDIV;
    }  
}