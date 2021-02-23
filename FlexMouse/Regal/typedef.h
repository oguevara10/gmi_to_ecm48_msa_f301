/**
  ********************************************************************************************************************************
  * @file    typedef.h 
  * @author  Kyle Mcbrady
  * @brief   Standard type definitions
  * @details 
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _TYPEDEF_H_
#define _TYPEDEF_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include <stdint.h>

/* Content ---------------------------------------------------------------------------------------------------------------------*/
// no float type

#define TRUE  1
#define FALSE 0

// ERROR has a conflict in the stm32f3xx.h file from ST's sdk.
// #define NOERROR 0
// #define ERROR   1

#define NOT_DONE           0
#define DONE               1
#define TIMEOUT            2
#define ADDRESS_ERROR      3
#define ADDRESS_REASSIGNED 4

#endif // _TYPEDEF_H_