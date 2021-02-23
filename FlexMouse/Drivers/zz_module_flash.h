/**
  ********************************************************************************************************************************
  * @file    zz_module_flash.h 
  * @author  Pamela Lee
  * @brief   Main driver module for flash storgae management.
  * @details This module initializes the flash 
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _ZZ_MODULE_FLASH_H_
#define _ZZ_MODULE_FLASH_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "main.h"
#include "typedef.h"

#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"
/* Includes for default settings -----------------------------------------------------------------------------------------------*/
#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"
#include "mc_stm_types.h"
#include "mc_type.h"
#include "hall_speed_pos_fdbk.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Base address of the Flash sectors */
/* Base address of the Flash pages */
#define ADDR_FLASH_PAGE_0     ((uint32_t)0x08000000) // Base address of Page 0, 2 Kbytes //
#define ADDR_FLASH_PAGE_1     ((uint32_t)0x08000800) // Base address of Page 1, 2 Kbytes //
#define ADDR_FLASH_PAGE_2     ((uint32_t)0x08001000) // Base address of Page 2, 2 Kbytes //
#define ADDR_FLASH_PAGE_3     ((uint32_t)0x08001800) // Base address of Page 3, 2 Kbytes //
#define ADDR_FLASH_PAGE_4     ((uint32_t)0x08002000) // Base address of Page 4, 2 Kbytes //
#define ADDR_FLASH_PAGE_5     ((uint32_t)0x08002800) // Base address of Page 5, 2 Kbytes //
#define ADDR_FLASH_PAGE_6     ((uint32_t)0x08003000) // Base address of Page 6, 2 Kbytes //
#define ADDR_FLASH_PAGE_7     ((uint32_t)0x08003800) // Base address of Page 7, 2 Kbytes //
#define ADDR_FLASH_PAGE_8     ((uint32_t)0x08004000) // Base address of Page 8, 2 Kbytes //
#define ADDR_FLASH_PAGE_9     ((uint32_t)0x08004800) // Base address of Page 9, 2 Kbytes //
#define ADDR_FLASH_PAGE_10    ((uint32_t)0x08005000) // Base address of Page 10, 2 Kbytes //
#define ADDR_FLASH_PAGE_11    ((uint32_t)0x08005800) // Base address of Page 11, 2 Kbytes //
#define ADDR_FLASH_PAGE_12    ((uint32_t)0x08006000) // Base address of Page 12, 2 Kbytes //
#define ADDR_FLASH_PAGE_13    ((uint32_t)0x08006800) // Base address of Page 13, 2 Kbytes //
#define ADDR_FLASH_PAGE_14    ((uint32_t)0x08007000) // Base address of Page 14, 2 Kbytes //
#define ADDR_FLASH_PAGE_15    ((uint32_t)0x08007800) // Base address of Page 15, 2 Kbytes //
#define ADDR_FLASH_PAGE_16    ((uint32_t)0x08008000) // Base address of Page 16, 2 Kbytes //
#define ADDR_FLASH_PAGE_17    ((uint32_t)0x08008800) // Base address of Page 17, 2 Kbytes //
#define ADDR_FLASH_PAGE_18    ((uint32_t)0x08009000) // Base address of Page 18, 2 Kbytes //
#define ADDR_FLASH_PAGE_19    ((uint32_t)0x08009800) // Base address of Page 19, 2 Kbytes //
#define ADDR_FLASH_PAGE_20    ((uint32_t)0x0800A000) // Base address of Page 20, 2 Kbytes //
#define ADDR_FLASH_PAGE_21    ((uint32_t)0x0800A800) // Base address of Page 21, 2 Kbytes //
#define ADDR_FLASH_PAGE_22    ((uint32_t)0x0800B000) // Base address of Page 22, 2 Kbytes //
#define ADDR_FLASH_PAGE_23    ((uint32_t)0x0800B800) // Base address of Page 23, 2 Kbytes //
#define ADDR_FLASH_PAGE_24    ((uint32_t)0x0800C000) // Base address of Page 24, 2 Kbytes //
#define ADDR_FLASH_PAGE_25    ((uint32_t)0x0800C800) // Base address of Page 25, 2 Kbytes //
#define ADDR_FLASH_PAGE_26    ((uint32_t)0x0800D000) // Base address of Page 26, 2 Kbytes //
#define ADDR_FLASH_PAGE_27    ((uint32_t)0x0800D800) // Base address of Page 27, 2 Kbytes /
#define ADDR_FLASH_PAGE_28    ((uint32_t)0x0800E000) // Base address of Page 28, 2 Kbytes //
#define ADDR_FLASH_PAGE_29    ((uint32_t)0x0800E800) // Base address of Page 29, 2 Kbytes //
#define ADDR_FLASH_PAGE_30    ((uint32_t)0x0800F000) // Base address of Page 30, 2 Kbytes //
#define ADDR_FLASH_PAGE_31    ((uint32_t)0x0800F800) // Base address of Page 31, 2 Kbytes //
  
  
  
#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_30   // Start @ of user Flash area //
#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_31   // End @ of user Flash area //
#define NumOfPage 1                                  // page size of setting // 
 
/** @Caution :All the settings offset/register number will be assigned in serquential by this enum 
    Please make sure MODBUS Universal-protocol and this flash assignment are lineup together **/
typedef enum                                     
{ 
  //ST_motor libraries flash parameters in flash table
  Index_A_POLE_PAIR_NUM                ,     // reg0  _1
  Index_A_RS                           ,     // reg1   _ 
  Index_A_LS                           ,     // reg2   _ 
  Index_A_NOMINAL_CURRENT              ,     // reg3   _1
  Index_A_MAX_APPLICATION_SPEED_RPM    ,     // reg4   _ 
  Index_A_MIN_APPLICATION_SPEED_RPM    ,     // reg5   _ 
  Index_A_PLL_KP_GAIN                  ,     // reg6   _1	MC_PROTOCOL_REG_PLL_KP   
  Index_A_PLL_KI_GAIN                  ,     // reg7   _1	MC_PROTOCOL_REG_PLL_KI
  Index_A_PWM_FREQUENCY                ,     // reg8   _ 
  Index_A_PID_TORQUE_KP_DEFAULT        ,     // reg9   _1	MC_PROTOCOL_REG_TORQUE_KP
  Index_A_PID_TORQUE_KI_DEFAULT        ,     // reg10  _1	MC_PROTOCOL_REG_TORQUE_KI
  Index_A_PID_FLUX_KP_DEFAULT          ,     // reg11  _1	MC_PROTOCOL_REG_FLUX_KP
  Index_A_PID_FLUX_KI_DEFAULT          ,     // reg12  _1	MC_PROTOCOL_REG_FLUX_KI
  Index_A_PID_SPEED_KP_DEFAULT         ,     // reg13  _1	MC_PROTOCOL_REG_SPEED_KP
  Index_A_PID_SPEED_KI_DEFAULT         ,     // reg14  _1	MC_PROTOCOL_REG_SPEED_KI
  Index_A_IQMAX                        ,     // reg15  _1
  Index_A_DEFAULT_CONTROL_MODE         ,     // reg16  _1       MC_PROTOCOL_REG_CONTROL_MODE
  Index_A_OV_VOLTAGE_THRESHOLD_V       ,     // reg17  _ 
  Index_A_UD_VOLTAGE_THRESHOLD_V       ,     // reg18  _ 
  Index_A_OV_TEMPERATURE_THRESHOLD_C   ,     // reg19  _ 
  Index_A_OV_TEMPERATURE_HYSTERESIS_C  ,     // reg20  _ 
  Index_A_PHASE1_DURATION              ,     // reg21  _1       MC_PROTOCOL_CODE_SET_REVUP_DATA:
  Index_A_PHASE1_FINAL_SPEED_UNIT      ,     // reg22  _1       MC_PROTOCOL_CODE_SET_REVUP_DATA:
  Index_A_PHASE1_FINAL_CURRENT         ,     // reg23  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
  Index_A_PHASE2_DURATION              ,     // reg24  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
  Index_A_PHASE2_FINAL_SPEED_UNIT      ,     // reg25  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
  Index_A_PHASE2_FINAL_CURRENT         ,     // reg26  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
  Index_A_PHASE3_DURATION              ,     // reg27  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
  Index_A_PHASE3_FINAL_SPEED_UNIT      ,     // reg28  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
  Index_A_PHASE3_FINAL_CURRENT         ,     // reg29  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
  Index_A_PHASE4_DURATION              ,     // reg30  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
  Index_A_PHASE4_FINAL_SPEED_UNIT      ,     // reg31  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
  Index_A_PHASE4_FINAL_CURRENT         ,     // reg32  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
  Index_A_PHASE5_DURATION              ,     // reg33  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
  Index_A_PHASE5_FINAL_SPEED_UNIT      ,     // reg34  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
  Index_A_PHASE5_FINAL_CURRENT         ,     // reg35  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
  Index_A_TRANSITION_DURATION          ,     // reg36  _1	MC_PROTOCOL_CODE_SET_REVUP_DATA:
  // below are future configurable data      
  Index_D_HALL_SENSORS_PLACEMENT       ,     // reg37  _1
  Index_D_HALL_PHASE_SHIFT             ,     // reg38  _1
  Index_D_M1_ENCODER_PPR               ,     // reg39  _ 
#if GAIN1 != 0   //pll or cord
  Index_A_GAIN1                        ,     // reg40  _ 
  Index_A_GAIN2                        ,     // reg41  _ 
#else
  Index_D_CORD_GAIN1                   ,     // reg40  _ 
  Index_D_CORD_GAIN2                   ,     // reg41  _ 
  Index_D_CORD_MAX_ACCEL_DPPP          ,     // reg42  _1
#endif //GAIN1   //pll or cord
  Index_ST_MOT_LIB_PARAMETERS_END      ,      
  //Module setting data should append below here and group as each module

#ifdef _AB_MODULE_MC_STATEMACHINE_H_
  Index_MIN_COMMANDABLE_SPEED	  =   Index_ST_MOT_LIB_PARAMETERS_END,                  //<---- please use the index above this line for the start of index for this module
  Index_MAX_COMMANDABLE_SPEED           ,                                               /** @caution1: if you change the sequence of module/s please make sure you follow the same logic)**/
  Index_SPEED_UP_RAMP_RATE              ,                                               /** @caution2: All parameter directly write into flash and active instantly !!!!! **/
  Index_SPEED_DOWN_RAMP_RATE            ,
  Index_SPEED_CONSIDERED_STOPPED        , 
  Index_MotSpinTimeOut                  , 
  Index_SpinPollPeriod                  , 
  Index_numOfStartRetry                 , 
  Index_StartRetryPeriod                , 
  Index_StartPeriodInc                  , 
  Index_over_current_threshold          , 
  Index_over_current_rpm_Reduce         , 
  Index_OvCurrent_derate_period         , 
  Index_over_power_threshold            , 
  Index_over_power_rpm_Reduce           , 
  Index_OvPower_derate_period           , 
  Index_over_temperature_threshold      , 
  Index_over_temperature_rpm_Reduce     , 
  Index_OvTemp_derate_period            , 
  Index_MODULE_MC_STATEMACHINE_END      ,
#else
  Index_MODULE_MC_STATEMACHINE_END =  Index_ST_MOT_LIB_PARAMETERS_END,                   //<----- If this module is not installed in this system, Flash index system will resume the last index of former module
#endif //_MODULE_MC_STATEMACHINE_H_

  index_Blk_Flash_CRC   = (FLASH_PAGE_SIZE/2)                                           //always reserve the last word for CRC         
      
}FlashOffsetIndex; 

__weak const uint16_t A_POLE_PAIR_NUM               @(FLASH_USER_START_ADDR + (2 * Index_A_POLE_PAIR_NUM               )  ) = POLE_PAIR_NUM            		; 
__weak const uint16_t A_RS                          @(FLASH_USER_START_ADDR + (2 * Index_A_RS                          )  ) = (RS * 1000)              		; //in mOhm
__weak const uint16_t A_LS                          @(FLASH_USER_START_ADDR + (2 * Index_A_LS                          )  ) = (LS * 1000)              		; //in mH
__weak const uint16_t A_NOMINAL_CURRENT             @(FLASH_USER_START_ADDR + (2 * Index_A_NOMINAL_CURRENT             )  ) = NOMINAL_CURRENT          		; 
__weak const uint16_t A_MAX_APPLICATION_SPEED_RPM   @(FLASH_USER_START_ADDR + (2 * Index_A_MAX_APPLICATION_SPEED_RPM   )  ) = MAX_APPLICATION_SPEED_RPM         ; //this parameter have to takecare of its dependance //MAX_BEMF_VOLTAGE parameter dependance => C3
                                                                                                                                                                                                                      //MAX_APPLICATION_SPEED_RPM parameter dependance => MAX_APPLICATION_SPEED_UNIT
__weak const uint16_t A_MIN_APPLICATION_SPEED_RPM   @(FLASH_USER_START_ADDR + (2 * Index_A_MIN_APPLICATION_SPEED_RPM   )  ) = MIN_APPLICATION_SPEED_RPM         ; //this parameter have to takecare of its dependance //MIN_APPLICATION_SPEED_RPM parameter dependance => MIN_APPLICATION_SPEED_UNIT
__weak const uint16_t A_PLL_KP_GAIN                 @(FLASH_USER_START_ADDR + (2 * Index_A_PLL_KP_GAIN                 )  ) = PLL_KP_GAIN                	; 
__weak const uint16_t A_PLL_KI_GAIN                 @(FLASH_USER_START_ADDR + (2 * Index_A_PLL_KI_GAIN                 )  ) = PLL_KI_GAIN                	; 
__weak const uint16_t A_PWM_FREQUENCY               @(FLASH_USER_START_ADDR + (2 * Index_A_PWM_FREQUENCY               )  ) = PWM_FREQUENCY              	; //this parameter have to takecare of its dependance //PWM_FREQUENCY parameter dependance => TF_REGULATION_RATE 
__weak const uint16_t A_PID_TORQUE_KP_DEFAULT       @(FLASH_USER_START_ADDR + (2 * Index_A_PID_TORQUE_KP_DEFAULT       )  ) = PID_TORQUE_KP_DEFAULT      	; 
__weak const uint16_t A_PID_TORQUE_KI_DEFAULT       @(FLASH_USER_START_ADDR + (2 * Index_A_PID_TORQUE_KI_DEFAULT       )  ) = PID_TORQUE_KI_DEFAULT      	; 
__weak const uint16_t A_PID_FLUX_KP_DEFAULT         @(FLASH_USER_START_ADDR + (2 * Index_A_PID_FLUX_KP_DEFAULT         )  ) = PID_FLUX_KP_DEFAULT        	; 
__weak const uint16_t A_PID_FLUX_KI_DEFAULT         @(FLASH_USER_START_ADDR + (2 * Index_A_PID_FLUX_KI_DEFAULT         )  ) = PID_FLUX_KI_DEFAULT        	; 
__weak const uint16_t A_PID_SPEED_KP_DEFAULT        @(FLASH_USER_START_ADDR + (2 * Index_A_PID_SPEED_KP_DEFAULT        )  ) = PID_SPEED_KP_DEFAULT       	; 
__weak const uint16_t A_PID_SPEED_KI_DEFAULT        @(FLASH_USER_START_ADDR + (2 * Index_A_PID_SPEED_KI_DEFAULT        )  ) = PID_SPEED_KI_DEFAULT       	; 
__weak const uint16_t A_IQMAX                       @(FLASH_USER_START_ADDR + (2 * Index_A_IQMAX                       )  ) = IQMAX                      	; 
__weak const uint16_t A_DEFAULT_CONTROL_MODE        @(FLASH_USER_START_ADDR + (2 * Index_A_DEFAULT_CONTROL_MODE        )  ) = DEFAULT_CONTROL_MODE       	;
__weak const uint16_t A_OV_VOLTAGE_THRESHOLD_V      @(FLASH_USER_START_ADDR + (2 * Index_A_OV_VOLTAGE_THRESHOLD_V      )  ) = OV_VOLTAGE_THRESHOLD_V     	;
__weak const uint16_t A_UD_VOLTAGE_THRESHOLD_V      @(FLASH_USER_START_ADDR + (2 * Index_A_UD_VOLTAGE_THRESHOLD_V      )  ) = UD_VOLTAGE_THRESHOLD_V     	; 
__weak const uint16_t A_OV_TEMPERATURE_THRESHOLD_C  @(FLASH_USER_START_ADDR + (2 * Index_A_OV_TEMPERATURE_THRESHOLD_C  )  ) = OV_TEMPERATURE_THRESHOLD_C 	; 
__weak const uint16_t A_OV_TEMPERATURE_HYSTERESIS_C @(FLASH_USER_START_ADDR + (2 * Index_A_OV_TEMPERATURE_HYSTERESIS_C )  ) = OV_TEMPERATURE_HYSTERESIS_C	; 
__weak const uint16_t A_PHASE1_DURATION             @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE1_DURATION             )  ) = PHASE1_DURATION            	; 
__weak const uint16_t A_PHASE1_FINAL_SPEED_UNIT     @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE1_FINAL_SPEED_UNIT     )  ) = PHASE1_FINAL_SPEED_UNIT    	; 
__weak const uint16_t A_PHASE1_FINAL_CURRENT        @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE1_FINAL_CURRENT        )  ) = PHASE1_FINAL_CURRENT       	; 
__weak const uint16_t A_PHASE2_DURATION             @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE2_DURATION             )  ) = PHASE2_DURATION            	; 
__weak const uint16_t A_PHASE2_FINAL_SPEED_UNIT     @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE2_FINAL_SPEED_UNIT     )  ) = PHASE2_FINAL_SPEED_UNIT    	; 
__weak const uint16_t A_PHASE2_FINAL_CURRENT        @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE2_FINAL_CURRENT        )  ) = PHASE2_FINAL_CURRENT       	; 
__weak const uint16_t A_PHASE3_DURATION             @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE3_DURATION             )  ) = PHASE3_DURATION            	; 
__weak const uint16_t A_PHASE3_FINAL_SPEED_UNIT     @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE3_FINAL_SPEED_UNIT     )  ) = PHASE3_FINAL_SPEED_UNIT    	; 
__weak const uint16_t A_PHASE3_FINAL_CURRENT        @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE3_FINAL_CURRENT        )  ) = PHASE3_FINAL_CURRENT       	; 
__weak const uint16_t A_PHASE4_DURATION             @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE4_DURATION             )  ) = PHASE4_DURATION            	; 
__weak const uint16_t A_PHASE4_FINAL_SPEED_UNIT     @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE4_FINAL_SPEED_UNIT     )  ) = PHASE4_FINAL_SPEED_UNIT    	; 
__weak const uint16_t A_PHASE4_FINAL_CURRENT        @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE4_FINAL_CURRENT        )  ) = PHASE4_FINAL_CURRENT       	; 
__weak const uint16_t A_PHASE5_DURATION             @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE5_DURATION             )  ) = PHASE5_DURATION            	; 
__weak const uint16_t A_PHASE5_FINAL_SPEED_UNIT     @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE5_FINAL_SPEED_UNIT     )  ) = PHASE5_FINAL_SPEED_UNIT    	; 
__weak const uint16_t A_PHASE5_FINAL_CURRENT        @(FLASH_USER_START_ADDR + (2 * Index_A_PHASE5_FINAL_CURRENT        )  ) = PHASE5_FINAL_CURRENT       	; 
__weak const uint16_t A_TRANSITION_DURATION         @(FLASH_USER_START_ADDR + (2 * Index_A_TRANSITION_DURATION         )  ) = TRANSITION_DURATION        	; 
__weak const uint16_t D_HALL_SENSORS_PLACEMENT      @(FLASH_USER_START_ADDR + (2 * Index_D_HALL_SENSORS_PLACEMENT      )  ) = HALL_SENSORS_PLACEMENT     	; 
__weak const uint16_t D_HALL_PHASE_SHIFT            @(FLASH_USER_START_ADDR + (2 * Index_D_HALL_PHASE_SHIFT            )  ) = HALL_PHASE_SHIFT           	; 
__weak const uint16_t D_M1_ENCODER_PPR              @(FLASH_USER_START_ADDR + (2 * Index_D_M1_ENCODER_PPR              )  ) = M1_ENCODER_PPR             	; 



#ifdef GAIN1 
  __weak const int16_t  A_GAIN1                       @(FLASH_USER_START_ADDR + (2 * Index_A_GAIN1                       )  ) = GAIN1				; //!!!!!!!!!!!!!!!!!!
  __weak const int16_t  A_GAIN2                       @(FLASH_USER_START_ADDR + (2 * Index_A_GAIN2                       )  ) = GAIN2                      	; //!!!!!!!!!!!!!!!!!!
#else
__weak const uint16_t D_CORD_GAIN1                  @(FLASH_USER_START_ADDR + (2 * Index_D_CORD_GAIN1                  )  ) = CORD_GAIN1         		; //!!!!!!!!!!!!!!!!
__weak const uint16_t D_CORD_GAIN2                  @(FLASH_USER_START_ADDR + (2 * Index_D_CORD_GAIN2                  )  ) = CORD_GAIN2         		; //!!!!!!!!!!!!!!!!
__weak const uint16_t D_CORD_MAX_ACCEL_DPPP         @(FLASH_USER_START_ADDR + (2 * Index_D_CORD_MAX_ACCEL_DPPP         )  ) = CORD_MAX_ACCEL_DPPP	        ;  //!!!!!!!!!!!!!!!!!!
#endif //GAIN1   //pll or cord


 
uint8_t flashPageErase(uint8_t drv_id_u8, uint32_t pageAddress);
uint8_t flashPageUpdate(uint8_t drv_id_u8, uint32_t _FrompageAddress, uint32_t _TopageAddress);
uint8_t flashBlockProgram(uint8_t drv_id_u8, uint32_t _TopageAddress, uint8_t* _buf, uint32_t _length);
uint8_t flashPageCopy(uint8_t drv_id_u8, uint32_t _FrompageAddress, uint32_t _TopageAddress);
uint8_t isFlashCRCValid(uint32_t _FLASH_START_ADDR, uint16_t _NumOfPage);
uint16_t FlashRead(unsigned char* aDataBuf, uint16_t offsetByte);
uint16_t FlashBufDeRegistered(uint16_t _offset, uint16_t* _returnBuf);
 

//internal buffer management function
void FlashBufInit(void); 
void FlashFlushBuf(void);
uint8_t FlashDatSet(uint16_t _offset, uint16_t _flashDat);

uint8_t IsFlashBufFull(void);
uint8_t Reg2Ram(uint32_t _RegNum, uint16_t _Value);
 /*
0x800f000  0x0f,  0x00,  0xff,  0xff,  0xff,  0xff,  0x64,  0x15,  0xd0,  0x07,  0x00,  0x00,  0x3c,  0x06,  0x71,  0x00,  
0x800f010  0x10,  0x27,  0x61,  0x08,  0xa9,  0x07,  0x61,  0x08,  0xa9,  0x07,  0x4c,  0x1d,  0xf4,  0x01,  0x64,  0x15, 
 
0x800f020  0x00,  0x00,  0x26,  0x02,  0x23,  0x00,  0x5a,  0x00,  0xff,  0xff,  0xe8,  0x03,  0x00,  0x00,  0x8e,  0x08,  
0x800f030  0x88,  0x13,  0xa6,  0x00,  0x8e,  0x08,  0x00,  0x00,  0xa6,  0x00,  0x8e,  0x08,  0x00,  0x00,  0xa6,  0x00,  

0x800f040  0x8e,  0x08,  0x00,  0x00,  0xa6,  0x00,  0x8e,  0x08,  0x19,  0x00,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  
0x800f050  0x18,  0xa1,  0xb3,  0x6a,  0x2c,  0x01,  0xc4,  0x09,  0x4b,  0x00,  0xff,  0xff,  0xff,  0xff,  0x04,  0x00, 
 
0x800f060  0xe8,  0x03,  0x06,  0x00,  0xd0,  0x07,  0x10,  0x27,  0xe8,  0x03,  0x0a,  0x00,  0xc8,  0x00,  0xb8,  0x0b,  
0x800f070  0x0a,  0x00,  0xc8,  0x00,  0x21,  0x00,  0x0a,  0x00,  0x30,  0x75,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,

0x800f7ff  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff, 
0x800f7f0  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0xff,  0x90,  0x08, 
*/ 
  

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _ZZ_MODULE_FLASH_H_ */