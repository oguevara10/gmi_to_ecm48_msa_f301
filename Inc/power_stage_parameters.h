
/**
  ******************************************************************************
  * @file    power_stage_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a power stage.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __POWER_STAGE_PARAMETERS_H
#define __POWER_STAGE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/************* PWM Driving signals section **************/

#define HW_DEAD_TIME_NS               100 /*!< Dead-time inserted 
                                                         by HW if low side signals 
                                                         are not used */
														 
/*********** Bus voltage sensing section ****************/
#define VBUS_PARTITIONING_FACTOR      0.0063 /*!< It expresses how 
                                                       much the Vbus is attenuated  
                                                       before being converted into 
                                                       digital value */
#define NOMINAL_BUS_VOLTAGE_V         320 
/******** Current reading parameters section ******/
/*** Topology ***/
#define THREE_SHUNT

#define RSHUNT                        0.03000 

/*  ICSs gains in case of isolated current sensors,
        amplification gain for shunts based sensing */
#define AMPLIFICATION_GAIN            6.00 

/*** Noise parameters ***/
#define TNOISE_NS                     2500
#define TRISE_NS                      2500 
#define MAX_TNTR_NS TRISE_NS
   
/************ Temperature sensing section ***************/
#define HARDWARE_VERSION_1p3KW_230V 1
#define HARDWARE_VERSION_1p3KW_460V 2

#define HARDWARE_VERSION HARDWARE_VERSION_1p3KW_230V

#if HARDWARE_VERSION == HARDWARE_VERSION_1p3KW_460V
/* V[V]=V0+dV/dT[V/Celsius]*(T-T0)[Celsius]*/
#define V0_V                          2.935 /*!< in Volts */
#define T0_C                          30 /*!< in Celsius degrees */
#define dV_dT                         0.023 /*!< V/Celsius degrees */
#define T_MAX                         125 /*!< Sensor measured 
                                                       temperature at maximum 
                                                       power stage working 
                                                       temperature, Celsius degrees */
#endif

#if HARDWARE_VERSION == HARDWARE_VERSION_1p3KW_230V
#define V0_V                          2.935 /*!< in Volts */
#define T0_C                          30 /*!< in Celsius degrees */
#define dV_dT                         0.023 /*!< V/Celsius degrees */
#define T_MAX                         125 /*!< Sensor measured 
                                                       temperature at maximum 
                                                       power stage working 
                                                       temperature, Celsius degrees */
#endif

#endif /*__POWER_STAGE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/