/**
  ********************************************************************************************************************************
  * @file    module_ecm_icl.c 
  * @author  Myron Mychal
  * @brief   This is based off the module_ap.c template
  * @details This module controls the behavior of the inrush current limiting (ICL) circuit in the ECM 48 frame
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
//#include "driver_usart1.h"
#include "module_ecm_icl.h"
#include "bus_voltage_sensor.h"
#include "pqd_motor_power_measurement.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
extern ProcessInfo processInfoTable[];
extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1;
#define	TIME_TO_TURN_GPIO_ON	100
#define	TIME_TO_TURN_GPIO_OFF	600
#define VBUS_TO_REMOVE_ICL		180
#define	VBUS_TO_ENGAGE_ICL		150

#define DemandPollPeriod 2000   
uint64_t tt_DemandECMICLTime;

//Usart1_Control* usart1Control_AppLocal;
uint8_t testECMICLCounter = 0;
uint64_t errorECMICLCounter = 0;
uint16_t		voltage1_u16 = 0;
uint16_t		voltage2_u16 = 0;
bool			isTiming = FALSE;

uint8_t tmpry4ECMICLTest = true; /** this is only for testing the error/log data exchange !!!!!!!!!!!!! please delete these line for production version **/
#define tmpDelayPeriod 100                                                       //time period for checking and sending 0-10V and speed data to motor board
uint64_t tt_DemandtmpECMICLDelayTime;


enum AppStates {
    INIT_ECM_ICL,
	WAIT_FOR_BUS_TO_RISE,
	BUS_IS_LOW_ICL_ON,
	WAIT_FOR_BUS_TO_FALL,
	BUS_IS_HIGH_ICL_OFF,
    // additional states to be added here as necessary.
    IRQ_APP = DEFAULT_IRQ_STATE,
    STOP_APP = KILL_APP
};

uint8_t p_moduleECM_ICL_u32(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8,
                        uint8_t irq_id_u8) {
    uint8_t return_state_u8 = 0;
	uint64_t		time_now_u64 = 0;
	static uint64_t	time_gpio_on_u64 = 0;
	static uint64_t	time_gpio_off_u64 = 0;
	
    switch (next_State_u8) {
        case INIT_ECM_ICL: {  
			LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
			
			/* GPIO Ports Clock Enable */
			//LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

			// initialization for relay output pin
			GPIO_InitStruct.Pin 			= LL_GPIO_PIN_5;
			GPIO_InitStruct.Mode 			= LL_GPIO_MODE_OUTPUT;
			GPIO_InitStruct.Speed 			= LL_GPIO_SPEED_FREQ_LOW;
			GPIO_InitStruct.OutputType 		= LL_GPIO_OUTPUT_PUSHPULL;
			GPIO_InitStruct.Pull 			= LL_GPIO_PULL_NO;
			LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
			//time_now_u64 = getSysCount();
            //time_gpio_off_u64 = time_now_u64 + TIME_TO_TURN_GPIO_OFF;                     		  			
			//isTiming = TRUE;
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);	// ICL is on/engaged			
            return_state_u8 = WAIT_FOR_BUS_TO_RISE;
            break;
        }
        case WAIT_FOR_BUS_TO_RISE: {
		  	// ICL is engaged because bus is not high enough
		  	voltage1_u16 = VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS);		  
			if((voltage1_u16 > VBUS_TO_REMOVE_ICL) && (!isTiming)) {
				isTiming = TRUE;
				time_gpio_off_u64 = getSysCount() + TIME_TO_TURN_GPIO_OFF;
				return_state_u8 = BUS_IS_HIGH_ICL_OFF;
			}
			else
				return_state_u8 = WAIT_FOR_BUS_TO_RISE;
            break;
		}
		case BUS_IS_HIGH_ICL_OFF: {
		  	// as long as bus stays over threshold, keep timing
		  	time_now_u64 = getSysCount();
			voltage2_u16 = VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS);
			if(voltage2_u16 < VBUS_TO_ENGAGE_ICL) {
				// bus has fallen and need to re-engage ICL
				LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);	// ICL is on/engaged			
			 	time_gpio_on_u64 = 0;
				time_gpio_off_u64 = 0;
				isTiming = FALSE;
				return_state_u8 = WAIT_FOR_BUS_TO_RISE;
			}
		  	else if ((time_now_u64 >= time_gpio_off_u64) && (isTiming)) {
			  	// bus has been high for long enough and can remove ICL
				time_gpio_off_u64 = 0;
				isTiming = FALSE;
				LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5); // remove ICL from circuit
            	return_state_u8 = WAIT_FOR_BUS_TO_FALL;
            }
			else
            	return_state_u8 = BUS_IS_HIGH_ICL_OFF;		  	
		  	break;			
        }
		case WAIT_FOR_BUS_TO_FALL: {
		  	// ICL is not engaged because bus is high enough
		  	voltage1_u16 = VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS);		  
			if((voltage1_u16 < VBUS_TO_ENGAGE_ICL) && (!isTiming)) {
				isTiming = TRUE;
				time_gpio_on_u64 = getSysCount() + TIME_TO_TURN_GPIO_ON;
				return_state_u8 = BUS_IS_LOW_ICL_ON;
			}
			else
				return_state_u8 = WAIT_FOR_BUS_TO_FALL;
            break;
        }
		case BUS_IS_LOW_ICL_ON: {
		  	// as long as bus stays under threshold, keep timing
		  	time_now_u64 = getSysCount();
			voltage2_u16 = VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS);
			if(voltage2_u16 > VBUS_TO_REMOVE_ICL) {
				// bus has risen and need to remove ICL
				LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);	// ICL is on/engaged			
			 	time_gpio_on_u64 = 0;
				time_gpio_off_u64 = 0;
				isTiming = FALSE;
				return_state_u8 = WAIT_FOR_BUS_TO_FALL;
			}
		  	else if ((time_now_u64 >= time_gpio_on_u64) && (isTiming)) {
			  	// bus has been low for long enough and can re-engage ICL
				time_gpio_on_u64 = 0;
				isTiming = FALSE;
				LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5); // remove ICL from circuit
            	return_state_u8 = WAIT_FOR_BUS_TO_RISE;
            }
			else
            	return_state_u8 = BUS_IS_LOW_ICL_ON;		  	
		  	break;			
        }
        case IRQ_APP: {
            errorECMICLCounter++;
            return_state_u8 = WAIT_FOR_BUS_TO_RISE;
            break;
        }
        case STOP_APP: {
            return_state_u8 = INIT_ECM_ICL;
            break;
        }
        default: {
            return_state_u8 = STOP_APP;
            break;
        }
    }
    return return_state_u8;
}
/** pam procedure #10 of Module insertion  :  add the module execution function end **/