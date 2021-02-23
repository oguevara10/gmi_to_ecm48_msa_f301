/**
  ********************************************************************************************************************************
  * @file    regal_mc_lib.h
  * @author  Roel Pantonial
  * @brief   header file for the Regal Motor Control Library
  * @details Definitions for On-the-Fly, Non-regenerative braking and other Regal Motor Control Algorithms
  ********************************************************************************************************************************
  *                                     
  *
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _REGAL_MC_LIB_H_
#define _REGAL_MC_LIB_H_

/* Includes ------------------------------------------------------------------*/
#include "mc_tuning.h"

#define CONTROLLED_BRAKING 0u // set to 0u if not using controlled braking
#define REGAL_OTF 1u

////////////////////////////////////////////////////////////////////////////
// RPa: Reference Bus Voltage Settings for non-regenerative braking
#define TORQUE2IQ_CONVERSION           338 //1.3229 shifted-
#define BRAKING_ENDSPEED                10
#define BRAKING_CURRENTSEEDING          500
#define FP16 65536
#define FP8  256

// RPa: Bus Voltage Control loop
#define PID_BRAKE_KP_DEFAULT          45//545
#define PID_BRAKE_KI_DEFAULT          10//160
#define PID_BRAKE_KD_DEFAULT          0
/* Brake PID parameter dividers */
#define BK_KPDIV                      16
#define BK_KIDIV                      256
#define BK_KDDIV                      16
#define BK_KPDIV_LOG                  LOG2(16)
#define BK_KIDIV_LOG                  LOG2(256)
#define BK_KDDIV_LOG                  LOG2(16)

////////////////////////////////////////////////////////////////////////////
// RPa: Imax Controller Settings for non-regenerative braking
#define RAMPEND_CURRENT              (int32_t) 10
#define RAMP_STEP                       10  
#define SPEED_TRANSITION                65
#define alpha_br                         (int16_t) (FP8*0.7)
#define BYTE_SHIFT                      8                                  
// RPa: the following hash defines are for the IMax trajectories to always be within the motor loss ellipse
//      This has to be adapted for each motor class; for conservative setting: a=13, b=-6, c=1220
#define RAMP_a          (int32_t) 13    //GMI: 17
#define RAMP_b          (int32_t) -6   // GMI: -9
#define RAMP_c          (int32_t) 1220 //GMI: 1285

// RPa: Imax Control loop
#define PID_IMAX_KP_DEFAULT          55
#define PID_IMAX_KI_DEFAULT          10
#define PID_IMAX_KD_DEFAULT          0
/* Brake PID parameter dividers */
#define IMAX_KPDIV                      16
#define IMAX_KIDIV                      256
#define IMAX_KDDIV                      16
#define IMAX_KPDIV_LOG                  LOG2(16)
#define IMAX_KIDIV_LOG                  LOG2(256)
#define IMAX_KDDIV_LOG                  LOG2(16)


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef enum IMAX_PHASE {
    STARTRAMP = 0,
    RAMPUP,
    STEADYSTATE,
    RAMPDOWN,
    LOWSPEED_IQHOLD,
    TURNONLOWSIDE,    
    MAX_PHASE
}Imax_t;

typedef struct Braking_Handle
{
    Imax_t BrakingPhase;
    int16_t FilteredSpeed;
    int16_t rMeasuredSpeed;
    int32_t IMax_Ref;
    int16_t Nbar;
    int32_t FeedForward_term;
    int32_t Adapt_IMax;
    uint16_t Adapt_BusVoltageRef;
    uint16_t Vbus_Add;
}
Braking_Handle_t;


int32_t FOC_BusVoltageControlM1(Braking_Handle_t * pHandle, PID_Handle_t * pPIDHandle, RDivider_Handle_t * pBSHandle);//RPa
int32_t FOC_ImaxCurrentControllerM1(Braking_Handle_t * pHandle, PID_Handle_t * pPIDHandle, FOCVars_t * pFOCHandle_t);//RPa
void BrakingStruct_Init(Braking_Handle_t * pHandle, SpeednTorqCtrl_Handle_t * pSTCHandle);//RPa
void MotorBraking_StateMachine(Braking_Handle_t * pBkHandle, PID_Handle_t * pPIDBusHandle, PID_Handle_t * pPIDImHandle, SpeednTorqCtrl_Handle_t * pSTCHandle, FOCVars_t * pFOCHandle_t, RDivider_Handle_t * pBSHandle);
void FOCStop_CalcCurrRef(Braking_Handle_t * pBrakeHandle, PID_Handle_t * pPIDBusHandle, PID_Handle_t * pPIDImHandle, FOCVars_t * pFOCHandle, RDivider_Handle_t * pBSHandle);
void RegenControlM1(Braking_Handle_t * pBkHandle, PID_Handle_t * pPIDBusHandle, PID_Handle_t * pPIDSpeedHandle, SpeednTorqCtrl_Handle_t * pSTCHandle, RDivider_Handle_t * pBSHandle);
extern PID_Handle_t PIDBkHandle_M1;//RPa
extern PID_Handle_t PIDImHandle_M1;//RPa
extern Braking_Handle_t BrakeHandle_M1;

#endif /* _REGAL_MC_LIB_H_ */