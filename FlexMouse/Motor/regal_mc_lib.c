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
#include "revup_ctrl.h"

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
  .wUpperIntegralLimit = (int32_t)100 * (int32_t)BK_KIDIV,
  .wLowerIntegralLimit = -(int32_t)100 * (int32_t)BK_KIDIV,
  .hUpperOutputLimit       = 0, //Rpa: a factor of the OVP, temp is for 60V
  .hLowerOutputLimit       = -2000,//RPa: a factor of the UVP, temp is for 60V
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
  .IMax_Ref = 2500,
  .FeedForward_term = 0,
  .Vbus_Add = BK_VBUS_ADD
};

/**
  * @brief  OTF Parameters Motor 1
  */
OTF_Handle_t OTFHandle_M1 =
{
  .hdir = 1, /* check for collinearity during detection phase */
  .hSyncTRef = TREF_SYNC, /* Reference Iq during transition to speed control, can be set to the final ramp current in the drive parameters (or 70% of it) */
  .CoilShortSpeed = 10, /* speed in the syncrhonisation phase when turning the low-side is safe and be done to put motor to stationary position */
  .MaxSyncSpeed = OTF_MAX_SYNC_SPEED, /* Maximum speed for motor to synchronise */
  .MinSyncSpeed = OTF_MIN_SYNC_SPEED, /* Minimum speed for motor to synchronise */
  .seamless_transfer = 0, /* tracking the different phase of revup and used for integrator seeding */
  .bemfg_alpha = 256, /* alpha gain of the bemf */
  .bemfg_beta = 256, /* beta gain of the bemf */
  .detect_bemfg = OTF_DBEMFG, /* bemf gain for good detection */
  .max_bemfg = OTF_MAX_BEMFG, /* maximum gain to clip vbus without causing instability to the observer or OCP tripping */
  .min_bemfg = OTF_MIN_BEMFG /* minimum gain for vbus clipping without causing OVP tripping, at the nominal voltage, can be set to 256 to flatten out dc bus */
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
  pHandle->Adapt_IMax = (int32_t)((BK_RAMP_a * (int32_t) pHandle->rMeasuredSpeed * (int32_t) pHandle->rMeasuredSpeed)>>BYTE_SHIFT) + \
    (int32_t)(BK_RAMP_b * (int32_t)pHandle->rMeasuredSpeed) + BK_RAMP_c;
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
    pBkHandle->Adapt_IMax = (int32_t)((BK_RAMP_a * (int32_t) MeasuredSpeedAbsValue * (int32_t) MeasuredSpeedAbsValue)>>BYTE_SHIFT) + \
      (int32_t)(BK_RAMP_b * (int32_t)MeasuredSpeedAbsValue) + BK_RAMP_c;
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
    if ((MC_GetImposedDirectionMotor1()==1)&&((pSTCHandle->SPD->hAvrMecSpeedUnit - pSTCHandle->TargetFinal) > 1))
    {//FWD direction
      pPIDSpeedHandle->hLowerOutputLimit = FOC_BusVoltageControlM1(pBkHandle, pPIDBusHandle, pBSHandle); 
      pPIDSpeedHandle->wLowerIntegralLimit = (int32_t) pPIDSpeedHandle->hLowerOutputLimit * (int32_t)SP_KIDIV;
    }
    else if ((MC_GetImposedDirectionMotor1()==-1)&&((pSTCHandle->SPD->hAvrMecSpeedUnit - pSTCHandle->TargetFinal) < -1))
    {//REV direction
      pPIDSpeedHandle->hUpperOutputLimit = FOC_BusVoltageControlM1(pBkHandle, pPIDBusHandle, pBSHandle);
      pPIDSpeedHandle->wUpperIntegralLimit = (int32_t) pPIDSpeedHandle->hUpperOutputLimit * (int32_t)SP_KIDIV;
    }  
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  Main revup controller procedure executing overall programmed phases and
  *         on-the-fly startup handling.
  */
__weak bool Regal_OTF_Exec( RevUpCtrl_Handle_t * pHandle, OTF_Handle_t *pOTFHandle )
{
  bool IsSpeedReliable;
  bool retVal = true;
  bool condition = false;

  if ( pHandle->hPhaseRemainingTicks > 0u )
  {
    /* Decrease the hPhaseRemainingTicks.*/
    pHandle->hPhaseRemainingTicks--;

    /* OTF start-up */
    if ( pHandle->bStageCnt == 0u )
    {
      if ( pHandle->EnteredZone1 == false )
      {
        if ( pHandle->pSNSL->pFctStoOtfResetPLL != MC_NULL )
        {
          pHandle->bResetPLLCnt++;
          if ( pHandle->bResetPLLCnt > pHandle->bResetPLLTh )
          {
            pHandle->pSNSL->pFctStoOtfResetPLL( pHandle->pSNSL );
            pHandle->bOTFRelCounter = 0u;
            pHandle->bResetPLLCnt = 0u;
          }
        }

        IsSpeedReliable = pHandle->pSNSL->pFctSTO_SpeedReliabilityCheck( pHandle->pSNSL );

        if ( IsSpeedReliable )
        {
          if ( pHandle->bOTFRelCounter < 127u ) pHandle->bOTFRelCounter++;
        }
        else
        {
          pHandle->bOTFRelCounter = 0u;
        }

        if ( pHandle->pSNSL->pFctStoOtfResetPLL != MC_NULL )
        {
          if ( pHandle->bOTFRelCounter == ( pHandle->bResetPLLTh >> 1 ) ) condition = true;
        }
        else
        {
          if ( pHandle->bOTFRelCounter == 127 ) condition = true;
        }

        if ( condition == true )
        {
          bool bCollinearSpeed = false;
          int16_t hObsSpeedUnit = SPD_GetAvrgMecSpeedUnit( pHandle->pSNSL->_Super );
          int16_t hObsSpeedUnitAbsValue =
                  ( hObsSpeedUnit < 0 ? ( -hObsSpeedUnit ) : ( hObsSpeedUnit ) ); /* hObsSpeedUnit absolute value */

          if ( pHandle->hDirection > 0 )
          {
            if ( hObsSpeedUnit > 0 ) bCollinearSpeed = true; /* actual and reference speed are collinear*/
          }
          else
          {
            if ( hObsSpeedUnit < 0 ) bCollinearSpeed = true; /* actual and reference speed are collinear*/
          }

          if ( bCollinearSpeed == false )
          {
            /*reverse speed management*/
            pHandle->bOTFRelCounter = 0u;
            pOTFHandle->hdir = -1;
          }
          else /*speeds are collinear*/
          {
            if ( ( uint16_t )( hObsSpeedUnitAbsValue ) > pHandle->hMinStartUpFlySpeed )
            {
              pHandle->hPhaseRemainingTicks = 0u;          
              pOTFHandle->seamless_transfer = 1;
              pHandle->bStageCnt = 1u;
              pOTFHandle->hdir = 1;
            } /* speed > MinStartupFly */
            else
            {
              pOTFHandle->hdir = -1; /* this would also be the case when speed is lower than the hMinStartUpFlySpeed */
            }
          } /* speeds are collinear */
        } /* speed is reliable */
      }/*EnteredZone1 1 is false */
      else
      {
        pHandle->pSNSL->pFctForceConvergency1( pHandle->pSNSL );
      }
    } /*stage 0: detection stage*/
    
    if ( pHandle->bStageCnt == 1u )
    {     
      int16_t hObsSpeedUnit = SPD_GetAvrgMecSpeedUnit( pHandle->pSNSL->_Super );
      int16_t hObsSpeedUnitAbsValue =
        ( hObsSpeedUnit < 0 ? ( -hObsSpeedUnit ) : ( hObsSpeedUnit ) ); /* hObsSpeedUnit absolute value */    
            
      if (pOTFHandle->hdir == -1)
      {
        if (hObsSpeedUnitAbsValue > pOTFHandle->CoilShortSpeed)
        {
          pHandle->hPhaseRemainingTicks++;
        }
        else
        {
          if (pHandle->OTFSCLowside == false)
          {
            PWMC_SwitchOffPWM( pHandle->pPWM );
            pHandle->OTFSCLowside = true;
            PWMC_TurnOnLowSides( pHandle->pPWM );         
          } /* check of coil shorting */
        } /* check of absolute speed */
      }/* hdir check of reversal */
      else if (pOTFHandle->hdir == 1)
      {
        if ((hObsSpeedUnitAbsValue > pOTFHandle->MaxSyncSpeed)||((hObsSpeedUnitAbsValue < pOTFHandle->MinSyncSpeed)&&(hObsSpeedUnitAbsValue >= pOTFHandle->CoilShortSpeed)))
        {
          pHandle->hPhaseRemainingTicks++;
        }
        else if ((hObsSpeedUnitAbsValue <= pOTFHandle->MaxSyncSpeed)&&(hObsSpeedUnitAbsValue >= pOTFHandle->MinSyncSpeed))
        {   
          VSS_SetCopyObserver( pHandle->pVSS );
          pHandle->pSNSL->pFctForceConvergency2( pHandle->pSNSL );      
          
          STC_ExecRamp( pHandle->pSTC, pHandle->hDirection * pOTFHandle->hSyncTRef, 0u );
          
          pHandle->hPhaseRemainingTicks = 1u;
          
          pHandle->pCurrentPhaseParams = &pHandle->OTFPhaseParams;       
          
          pHandle->bStageCnt = 6u;               
        }
        else if (hObsSpeedUnitAbsValue < pOTFHandle->CoilShortSpeed)
        {
          if (pHandle->OTFSCLowside == false)
          {
            PWMC_SwitchOffPWM( pHandle->pPWM );
            pHandle->OTFSCLowside = true;
            PWMC_TurnOnLowSides( pHandle->pPWM );         
          } /* check of coil shorting */
        } /* check of absolute speed */
      }/*hdir check for forward*/
    }/*stage 1: synchronization phase*/
    
  } /* hPhaseRemainingTicks > 0 */
  
  
  
  if ( pHandle->hPhaseRemainingTicks == 0u )
  {
    if ( pHandle->pCurrentPhaseParams != MC_NULL )
    {
      if ( pHandle->bStageCnt == 0u )
      {
        /*end of OTF*/
        pHandle->bOTFRelCounter = 0u;
      }
      else if ( ( pHandle->bStageCnt == 1u ) )
      {
        PWMC_SwitchOnPWM( pHandle->pPWM );
        pHandle->OTFSCLowside = false;
      }
      else
      {
      }

      /* If it becomes zero the current phase has been completed.*/
      /* Gives the next command to STC and VSS.*/
      STC_ExecRamp( pHandle->pSTC, pHandle->pCurrentPhaseParams->hFinalTorque * pHandle->hDirection,
                    ( uint32_t )( pHandle->pCurrentPhaseParams->hDurationms ) );

      VSS_SetMecAcceleration( pHandle->pVSS,
                              pHandle->pCurrentPhaseParams->hFinalMecSpeedUnit * pHandle->hDirection,
                              pHandle->pCurrentPhaseParams->hDurationms );

      /* Compute hPhaseRemainingTicks.*/
      pHandle->hPhaseRemainingTicks =
              ( uint16_t )( ( ( uint32_t )pHandle->pCurrentPhaseParams->hDurationms *
                      ( uint32_t )pHandle->hRUCFrequencyHz ) / 1000u );
      pHandle->hPhaseRemainingTicks++;

      /*Set the next phases parameter pointer.*/
      pHandle->pCurrentPhaseParams = pHandle->pCurrentPhaseParams->pNext;

      /*Increases the rev up stages counter.*/
      pHandle->bStageCnt++;
    }
    else
    {
      if ( pHandle->bStageCnt == pHandle->bPhaseNbr - 1 ) /* End of user programmed revup */
      {
        retVal = false;
      }
      else if ( pHandle->bStageCnt == 7u ) /* End of first OTF runs */
      {
        pHandle->bStageCnt = 0u; /* Breaking state */
        pHandle->hPhaseRemainingTicks = 0u;
      }
      else
      {
      }
    }
  }
  return retVal;
}