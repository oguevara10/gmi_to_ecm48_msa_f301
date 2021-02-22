/**
  ********************************************************************************************************************************
  * @file    sequential_memory.h 
  * @author  Oscar Guevara
  * @brief   This file exposes the memory abstraction API for the FlexMouse sequential memory data structure.
  * @details FlexMouse has two types of available memory: sequential and structured memory. Sequential memory, declared in this
  *             file, allows the user to store and retrieve data. As the name implies, data can only be accessed in the same
  *             order as it was stored. 
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _SEQ_MEM_H_
#define _SEQ_MEM_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "../Memory/ring_buffer.h"

/* User parameters -------------------------------------------------------------------------------------------------------------*/
// #define TOTAL_NUM_OF_SEQ_MEM_INSTANCES 4
#define NO_OWNER                                    255
#define NO_OWNER_ID                                 255
#define EMPTY_LIST 0

#define ACCESS_MODE_BROADCAST  0
#define ACCESS_MODE_READ_ONLY  1
#define ACCESS_MODE_WRITE_ONLY 2

/**
  ********************************************************************************************************************************
  * @brief   Creates an instance of sequential memory.
  * @details 
  * @param   owner_u8
  * @param   ring_buf_size_u8
  * @param   access_mode_u8 (0 = broadcast, 1= read only, 2 = write only)
  * @param   p_user_list_u8
  * @param   user_list_size_u8
  * @return  Returns a pointer to a new sequential memory instance location. If no instance is available then return's NULL.
  ********************************************************************************************************************************
  */
Ring_Buf_Handle SeqMem_CreateInstance(uint8_t owner_u8, uint8_t ring_buf_size_u8, uint8_t access_mode_u8,
                                                   uint8_t *p_user_list_u8, uint8_t user_list_size_u8);

/**
  ********************************************************************************************************************************
  * @brief   Frees memory of a previously allocated instance of sequential memory.
  * @details System will claim back most of the previously used memory resource.
  * @param   owner_u8
  * @param   this_ring_buf_u32 If the value is NULL, kill any 
  * @return  TRUE if the instance was destroyed, or FALSE if the instance was not found.
  ********************************************************************************************************************************
  */
uint8_t SeqMem_DestroyInstance(uint8_t owner_u8, Ring_Buf_Handle this_ring_buf_u32);

/**
  ********************************************************************************************************************************
  * @brief   Resets all sequential memory buffers
  * @details 
  * @return  
  ********************************************************************************************************************************
  */
uint8_t SeqMem_InitBufs(void);

#endif /* _SEQ_MEM_H_ */