/**
  ********************************************************************************************************************************
  * @file    ram_buffer.h 
  * @author  Pamela Lee
  * @brief   This file declares the interface for the RAM buffer data structure.
  * @details This file contains declarations for constants, macros, global variables, and function prototypes required to
  *          implement a single RAM buffer (consequitive RAM area allocation) data structure.
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _RAM_BUF_H_
#define _RAM_BUF_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "typedef.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* User parameters -------------------------------------------------------------------------------------------------------------*/

/* global parameters -----------------------------------------------------------------------------------------------------------*/  
static uint8_t Ram_mallocError = 0;                             //Heap memory alocation error counter               //house keeping code


/* API Functions ---------------------------------------------------------------------------------------------------------------*/
/**
  ********************************************************************************************************************************
  * @brief   Structure definition for a RAM buffer instance.
  * @details Ring_Buf is a forward structure declaration of the RAM buffer instance, implemented within the source library.
  *             Maintaining this framework ensures that users of the ram_buffer library work directly with this API instead of 
  *             modifying the structure directly.
  ********************************************************************************************************************************
  */
typedef struct Ram_Buf Ram_Buf;

/**
  ********************************************************************************************************************************
  * @brief   Handle for a ring buffer instance.
  * @details This handle provides a method for accessing the Ram_Buf pointer indirectly, preventing the user from attempting
  *             to de-reference the value. Additionally, the handle prevents the need to cast the pointer within the following
  *             function implementations.
  ********************************************************************************************************************************
  */
typedef Ram_Buf *Ram_Buf_Handle;

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   p_ramBuf_u8             Pointer to this instance of the RAM buffer's location of memory.
  * @param   totalNumOfElements_u32  Declared total size of available memory for this instance of the RAM buffer.
  * @param   owner_u8                   The index of the process that owns this buffer instance.
  *                                         If the process is killed, this RAM buffer instance is also killed.
  * @param   accessMode_u8              Access mode to this instance of the RAM buffer.
  *                                         (0 = broadcast, 1= read only, 2 = write only)
  * @param   p_userList_u8              Pointer to a list of users with access to this instance of the RAM buffer.
  * @param   userListSize_u8            Size of the list of users with access to this instance of the RAM buffer.
  * @param   processInstanceIndex_u8    Index for this particular RAM buffer instance that the parent process uses to
  *                                         uniquely identify this instance.
  ********************************************************************************************************************************
  */
struct Ram_Buf {
    uint8_t *p_ramBuf_u8;
    uint32_t totalNumOfElements_u32;
    uint8_t owner_u8;
    uint8_t accessMode_u8;
    uint8_t *p_userList_u8;
    uint8_t userListSize_u8;
    uint8_t processInstanceIndex_u8;
};

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   index
  * @param   owner_u8
  * @param   ram_buf_size_u8
  * @param   access_mode_u8
  * @param   p_user_list_u8
  * @param   user_list_size_u8
  * @return  
  ********************************************************************************************************************************
  */
void RamBuf_Initialize(Ram_Buf_Handle this_ram_buf_u32, uint8_t owner_u8, uint8_t ram_buf_size_u8,
                          uint8_t access_mode_u8, uint8_t *p_user_list_u8, uint8_t user_list_size_u8);

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ram_buf_u32   The RAM buffer handle referencing the RAM buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
void RamBuf_Release(Ram_Buf_Handle this_ram_buf_u32);

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ram_buf_u32   The RAM buffer handle referencing the RAM buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
uint8_t RamBuf_GetOwner(Ram_Buf_Handle this_ram_buf_u32);

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ram_buf_u32   The RAM buffer handle referencing the RAM buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
uint8_t RamBuf_SetOwner(Ram_Buf_Handle this_ram_buf_u32, uint8_t owner_u8);

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ram_buf_u32   The RAM buffer handle referencing the RAM buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
uint8_t RamBuf_GetProcessInstanceIndex(Ram_Buf_Handle this_ram_buf_u32);

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ram_buf_u32   The RAM buffer handle referencing the RAM buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
uint8_t RamBuf_SetProcessInstanceIndex(Ram_Buf_Handle this_ram_buf_u32, uint8_t valueToSet);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _RAM_BUF_H_ */