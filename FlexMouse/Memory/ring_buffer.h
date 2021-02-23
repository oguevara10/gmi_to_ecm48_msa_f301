/**
  ********************************************************************************************************************************
  * @file    ring_buffer.h 
  * @author  Pamela Lee
  * @brief   This file declares the interface for the ring buffer data structure.
  * @details This file contains declarations for constants, macros, global variables, and function prototypes required to
  *          implement a ring buffer (first in first out queue) data structure.
  *          The ring buffer is a fixed size structure that can be initialized with size ranging from 2 to (2^32)-1 bytes.
  *          New data is entered at the location where the ring buffer head pointer indicates.
  *          Stored data is accessed at the location that the ring buffer tail pointer indicates.
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _RING_BUF_H_
#define _RING_BUF_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include <stdlib.h>

#include "typedef.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* User parameters -------------------------------------------------------------------------------------------------------------*/
#define NONE_AVAILABLE 255

#define OFFSET_ERROR 0xffff
#define NOT_FOUND    0xf000

/* global parameters -----------------------------------------------------------------------------------------------------------*/  
static uint8_t Ring_mallocError = 0;                    //Heap memory alocation error counter                //house keeping code

/* API Functions ---------------------------------------------------------------------------------------------------------------*/
/**
  ********************************************************************************************************************************
  * @brief   Structure definition for a ring buffer instance.
  * @details Ring_Buf is a forward structure declaration of the ring buffer instance, implemented within the source library.
  *             Maintaining this framework ensures that users of the ring_buffer library work directly with this API instead of 
  *             modifying the structure directly.
  ********************************************************************************************************************************
  */
typedef struct Ring_Buf Ring_Buf;

/**
  ********************************************************************************************************************************
  * @brief   Handle for a ring buffer instance.
  * @details This handle provides a method for accessing the Ring_Buf pointer indirectly, preventing the user from attempting
  *             to de-reference the value. Additionally, the handle prevents the need to cast the pointer within the following
  *             function implementations.
  ********************************************************************************************************************************
  */
typedef Ring_Buf *Ring_Buf_Handle;

/**
  ********************************************************************************************************************************
  * @brief                                  Structure definition for a ring buffer instance.
  * @details 
  * @param   p_ringBuf_u8                Pointer to this instance of the ring buffer's location in memory.
  * @param   head_s32                       Pointer to the first byte of used memory in this instance of the ring buffer.
  * @param   tail_s32                       Pointer to the last byte of used memory in this instance of the ring buffer.
  * @param   usedNumOfElements_s32       Number of bytes of used memory in this instance of the ring buffer.
  * @param   totalNumOfElements_u32      Declared total size of available memory for this instance of the ring buffer.
  * @param   is_OverwrittingAllowed_u8       Flag for determining over-writting on buffer overflow. (0 = not allowed, 1 = allowed)
  * @param   owner_u8                       The index of the process that owns this buffer instance.
  *                                             If the process is killed, this ring buffer instance is also killed.
  * @param   accessMode_u8                  Access mode to this instance of the ring buffer.
  *                                             (0 = broadcast, 1= read only, 2 = write only)
  * @param   p_userList_u8                  Pointer to a list of users with access to this instance of the ring buffer.
  * @param   userListSize_u8                Size of the list of users with access to this instance of the ring buffer.
  * @param   processInstanceIndex_u8        Index for this particular ring buffer instance that the parent process uses to
  *                                             uniquely identify this instance.
  * @param   systemInstanceIndex_u8         Index for this particular ring buffer instance that the kernel uses to uniquely
  *                                             identify this instance.
  ********************************************************************************************************************************
  */
struct Ring_Buf {
    uint8_t *p_ringBuf_u8;
    int32_t head_s32;
    int32_t tail_s32;
    int32_t usedNumOfElements_s32;
    uint32_t totalNumOfElements_u32;
    uint8_t is_OverwrittingAllowed_u8;
    uint8_t owner_u8;
    uint8_t accessMode_u8;
    uint8_t *p_userList_u8;
    uint8_t userListSize_u8;
    uint8_t processInstanceIndex_u8;
    uint8_t systemInstanceIndex_u8;
};

/**
  ********************************************************************************************************************************
  * @brief   Initialize ring buffer instance to the default values.
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   owner_u8               This value is the index that represents the parent process that owns this instance.
  * @param   ring_buf_size_u8    This value represents the total number of elements this ring buffer instance can store.
  * @param   access_mode_u8         This value determines the read/write access level for this ring buffer instance.
  * @param   p_user_list_u8         This value is a pointer to where the list of processes that use this instance are stored.
  * @param   user_list_size_u8      This value is the number of processes that use this ring buffer instance.
  ********************************************************************************************************************************
  */
void RingBuf_Initialize(Ring_Buf_Handle this_ring_buf_u32, uint8_t owner_u8, uint8_t ring_buf_size_u8,
                           uint8_t access_mode_u8, uint8_t *p_user_list_u8, uint8_t user_list_size_u8);

/**
  ********************************************************************************************************************************
  * @brief   Set the value of this ring buffer instance to the default values.
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   owner_u8               This value is the index that represents the parent process that owns this instance.
  * @param   ring_buf_size_u8    This value represents the total number of elements this ring buffer instance can store.
  * @param   access_mode_u8         This value determines the read/write access level for this ring buffer instance.
  * @param   p_user_list_u8         This value is a pointer to where the list of processes that use this instance are stored.
  * @param   user_list_size_u8      This value is the number of processes that use this ring buffer instance.
  ********************************************************************************************************************************
  */
void RingBuf_ClearContents(Ring_Buf_Handle this_ring_buf_u32);

/**
  ********************************************************************************************************************************
  * @brief   Writes a single character to the ring buffer instance.
  * @details Writes a single character to the address that tail_s32 currently point to if the ring buffer is not full. Updates
  *                                 the number of used elements and increments the tail_s32 pointer.
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   character_u8           The array containing the character that will be written
  * @return  Returns TRUE if the operation was successful, FALSE otherwise.
  ********************************************************************************************************************************
  */
uint8_t RingBuf_WriteCharacter(Ring_Buf_Handle this_ring_buf_u32, uint8_t character_u8[]);

/**
  ********************************************************************************************************************************
  * @brief   Writes a block of characters to the ring buffer instance.
  * @details Writes a single character to the address that tail_s32 currently point to if the ring buffer is not full. Updates
  *                                 the number of used elements and increments the tail_s32 pointer by the length of characters to
  *                                 write. If the ring buffer becomes full during the write operation and overwrittin is not
  *                                 allowed it returns FALSE.
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   character_array_u8	    The array containing the characters that will be written
  * @param   p_len_u32	        The number of characters to be written
  * @return  Returns TRUE if the operation was successful, FALSE otherwise.
  ********************************************************************************************************************************
  */
uint8_t RingBuf_WriteBlock(Ring_Buf_Handle this_ring_buf_u32, uint8_t character_array_u8[], uint32_t *p_len_u32);

/**
  ********************************************************************************************************************************
  * @brief   Reads the specified number of bytes from the ring buffer instance.
  * @details Reads the specified number of bytes from the buffer. After reading, the bytes are removed from the buffer.
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   character_array_u8	    The array used to store the characters that will be read
  * @param   p_len_u32	        The number of characters to be read
  * @return  Returns TRUE if the operation was successful, FALSE otherwise.
  ********************************************************************************************************************************
  */
uint8_t RingBuf_ReadBlock(Ring_Buf_Handle this_ring_buf_u32, uint8_t character_array_u8[], uint32_t *p_len_u32);

/**
  ********************************************************************************************************************************
  * @brief   Reads the specified number of bytes from the ring buffer instance.
  * @details Reads the specified number of bytes from the buffer. After reading, the bytes are still available in the buffer.
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   character_array_u8     The array used to store the characters that will be read
  * @param   offset_u32	            The number of bytes to skip, starting from the head, before the read begins
  * @param   p_len_u32	        The number of characters to be read. Holds the number of bytes actually read after function call.
  * @return  Returns TRUE if the operation was successful, FALSE otherwise.
  ********************************************************************************************************************************
  */
uint8_t RingBuf_Observe(Ring_Buf_Handle this_ring_buf_u32, uint8_t character_array_u8[], uint32_t offset_u32,
                           uint32_t *p_len_u32);

/**
  ********************************************************************************************************************************
  * @brief   Iterates through the ring buffer and returns the index of a specified character if it exists.
  * @details Returns the index of the first instance of a specified character that is found on the ring buffer starting from the 
  *             head.
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   character_array_u8     The specified character to search for
  * @param   offset_u32	            The number of bytes to skip, starting from the head, before the search begins
  * @param   length_u32	            The number of characters to be read. Holds the number of bytes actually read after function call.
  * @return  Returns the index of the first instance of the character if found. or 0xf000 == not found, or 0xffff == offset error
  ********************************************************************************************************************************
  */
uint32_t RingBuf_Search(Ring_Buf_Handle this_ring_buf_u32, uint8_t character_array_u8, uint32_t offset_u32);

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
uint8_t *RingBuf_GetPointerToRingBuf(Ring_Buf_Handle this_ring_buf_u32);

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
uint8_t RingBuf_GetUsedNumOfElements(Ring_Buf_Handle this_ring_buf_u32);


/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
int32_t RingBuf_GetCalculateAvailableSpace(Ring_Buf_Handle this_ring_buf_u32);

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
uint8_t RingBuf_GetOwner(Ring_Buf_Handle this_ring_buf_u32);

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   owner_u8               The index of the process that should be made the owner of this buffer instance.
  * @return  Returns 
  ********************************************************************************************************************************
  */
uint8_t RingBuf_SetOwner(Ring_Buf_Handle this_ring_buf_u32, uint8_t owner_u8);

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
uint8_t RingBuf_GetProcessInstanceIndex(Ring_Buf_Handle this_ring_buf_u32);

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32       The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   processInstanceIndex_u8    The index of this ring buffer relative to the number of ring buffers the owner has.
  * @return  Returns 
  ********************************************************************************************************************************
  */
uint8_t RingBuf_SetProcessInstanceIndex(Ring_Buf_Handle this_ring_buf_u32, uint8_t processInstanceIndex_u8);

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   systemInstanceIndex_u8 The index of this ring buffer relative to all the ring buffers the kernel has.
  * @return  Returns 
  ********************************************************************************************************************************
  */
uint8_t RingBuf_SetSystemInstanceIndex(Ring_Buf_Handle this_ring_buf_u32, uint8_t systemInstanceIndex_u8);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _RING_BUF_H_ */