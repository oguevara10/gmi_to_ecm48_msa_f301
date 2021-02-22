/**
  ********************************************************************************************************************************
  * @file    structured_memory.h 
  * @author  Oscar Guevara
  * @brief   This file exposes the memory abstraction API for the FlexMouse structured memory data structure.
  * @details FlexMouse has two types of available memory: sequential and structured memory. Structured memory, declared in this
  *             file, allows the user to store and retrieve data. Unlike sequential memory, structured memory allows the user to
  *             retrieve any data previously stored at any time, regardless of in what order the data was stored.
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _STRUCT_MEM_H_
#define _STRUCT_MEM_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "../Memory/ram_buffer.h"

/* User parameters -------------------------------------------------------------------------------------------------------------*/
//#define TOTAL_NUM_OF_STRUCT_MEM_INSTANCES 2
#define NO_OWNER                                    255
#define NO_OWNER_ID                                 255

/**
  ********************************************************************************************************************************
  * @brief   Creates an instance of structured memory
  * @details 
  * @param   owner_u8
  * @param   ram_buf_size_u8
  * @param   access_mode_u8 (0 = broadcast, 1= read only, 2 = write only)
  * @param   p_user_list_u8
  * @param   user_list_size_u8 
  * @return  Returns a pointer to new structured memory instance location.
  ********************************************************************************************************************************
  */
Ram_Buf_Handle StructMem_CreateInstance(uint8_t owner_u8, uint8_t ram_buf_size_u8, uint8_t access_mode_u8,
                                                  uint8_t *p_user_list_u8, uint8_t user_list_size_u8);

/**
  ********************************************************************************************************************************
  * @brief   Frees memory from a previously allocated instance of structured memory.
  * @details System will calim back most of the previously used memory resource.
  * @param   owner_u8
  * @param   this_ram_buf_u32
  * @return  
  ********************************************************************************************************************************
  */
uint8_t StructMem_DestroyInstance(uint8_t owner_u8, Ram_Buf_Handle this_ram_buf_u32);

/**
  ********************************************************************************************************************************
  * @brief   Resets all structured memory buffers
  * @details 
  * @return  
  ********************************************************************************************************************************
  */
uint8_t StructMem_InitBufs(void);

#endif /* _STRUCT_MEM_H_ */