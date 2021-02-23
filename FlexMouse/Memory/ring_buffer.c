/**
  ********************************************************************************************************************************
  * @file    ring_buffer.c
  * @author  Pamela Lee
  * @brief   This file implements the interface for the ring buffer data structure.
  * @details This file contains function definitions implementing a ring buffer (first in first out queue) data structure.
  *          The ring buffer is a fixed size structure that can be initialized with size ranging from 2 to (2^32)-1 bytes.
  *          New data is entered at the location where the ring buffer head pointer indicates.
  *          Stored data is accessed at the location that the ring buffer tail pointer indicates.
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "ring_buffer.h"

/* Private Function Prototypes -------------------------------------------------------------------------------------------------*/
static uint8_t RingBuf_GetValueOfHead(Ring_Buf_Handle this_ring_buf_u32);
static uint8_t RingBuf_SetValueOfHead(Ring_Buf_Handle this_ring_buf_u32, uint8_t head_s32);
static uint8_t RingBuf_GetValueOfTail(Ring_Buf_Handle this_ring_buf_u32);
static uint8_t RingBuf_SetValueOfTail(Ring_Buf_Handle this_ring_buf_u32, uint8_t tail_s32);
static uint8_t RingBuf_IncHead(Ring_Buf_Handle this_ring_buf_u32);
static uint8_t RingBuf_IncTail(Ring_Buf_Handle this_ring_buf_u32);
static uint8_t *RingBuf_GetPhysicalAddressOfHeadPlusOffset(Ring_Buf_Handle this_ring_buf_u32, uint32_t offset_u32);
static uint8_t RingBuf_IsEmpty(Ring_Buf_Handle this_ring_buf_u32);
static uint8_t RingBuf_IsFull(Ring_Buf_Handle this_ring_buf_u32);
static int32_t RingBuf_CalculateAvailableSpace(Ring_Buf_Handle this_ring_buf_u32);
static uint8_t *RingBuf_SetPointerToRingBuf(Ring_Buf_Handle this_ring_buf_u32, uint8_t *p_ringBuf_u8);
static uint8_t RingBuf_SetUsedNumOfElements(Ring_Buf_Handle this_ring_buf_u32, uint32_t usedNumOfElements_s32);
static uint8_t RingBuf_GetTotalNumOfElements(Ring_Buf_Handle this_ring_buf_u32);
static uint8_t RingBuf_SetTotalNumOfElements(Ring_Buf_Handle this_ring_buf_u32, uint32_t totalNumOfElements_u32);
static uint8_t RingBuf_GetIsOverwrittingAllowed(Ring_Buf_Handle this_ring_buf_u32);
static uint8_t RingBuf_SetIsOverwrittingAllowed(Ring_Buf_Handle this_ring_buf_u32, uint8_t is_OverwrittingAllowed_u8);
static uint8_t RingBuf_SetAccessMode(Ring_Buf_Handle this_ring_buf_u32, uint8_t access_mode_u8);
static uint8_t *RingBuf_SetUserList(Ring_Buf_Handle this_ring_buf_u32, uint8_t *p_user_list_u8);
static uint8_t RingBuf_SetUserListSize(Ring_Buf_Handle this_ring_buf_u32, uint8_t user_list_size_u8);

/* Public Functions ------------------------------------------------------------------------------------------------------------*/
void RingBuf_Initialize(Ring_Buf_Handle this_ring_buf_u32, uint8_t owner_u8, uint8_t ring_buf_size_u8,
                           uint8_t access_mode_u8, uint8_t *p_user_list_u8, uint8_t user_list_size_u8) {
    uint8_t* tmpryBuf;                                                          //check any Heap memory alocation error                 //house keeping code
    if( (tmpryBuf = malloc(ring_buf_size_u8)) == NULL) Ring_mallocError++;      //check any Heap memory alocation error                 //house keeping code
    RingBuf_SetPointerToRingBuf(this_ring_buf_u32, tmpryBuf);
    RingBuf_SetValueOfHead(this_ring_buf_u32, 0);
    RingBuf_SetValueOfTail(this_ring_buf_u32, 0);
    RingBuf_SetUsedNumOfElements(this_ring_buf_u32, 0);
    RingBuf_SetTotalNumOfElements(this_ring_buf_u32, ring_buf_size_u8);
    RingBuf_SetIsOverwrittingAllowed(this_ring_buf_u32, TRUE);
    RingBuf_SetOwner(this_ring_buf_u32, owner_u8);
    RingBuf_SetAccessMode(this_ring_buf_u32, access_mode_u8);
    RingBuf_SetUserList(this_ring_buf_u32, p_user_list_u8);
    RingBuf_SetUserListSize(this_ring_buf_u32, user_list_size_u8);
    RingBuf_SetProcessInstanceIndex(this_ring_buf_u32, NONE_AVAILABLE);
    RingBuf_SetSystemInstanceIndex(this_ring_buf_u32, NONE_AVAILABLE);
}

void RingBuf_ClearContents(Ring_Buf_Handle this_ring_buf_u32) {
    RingBuf_SetValueOfHead(this_ring_buf_u32, 0);
    RingBuf_SetValueOfTail(this_ring_buf_u32, 0);
    RingBuf_SetUsedNumOfElements(this_ring_buf_u32, 0);
}

uint8_t RingBuf_WriteCharacter(Ring_Buf_Handle this_ring_buf_u32, uint8_t c[]) {
    uint8_t *TailPtr = RingBuf_GetPointerToRingBuf(this_ring_buf_u32) + RingBuf_GetValueOfTail(this_ring_buf_u32);
    if (RingBuf_IsFull(this_ring_buf_u32) && !RingBuf_GetIsOverwrittingAllowed(this_ring_buf_u32)) {
        return FALSE;
    }
    *TailPtr = c[0];
    RingBuf_SetUsedNumOfElements(this_ring_buf_u32, RingBuf_GetUsedNumOfElements(this_ring_buf_u32) + 1);
    return RingBuf_IncTail(this_ring_buf_u32);
}

uint8_t RingBuf_WriteBlock(Ring_Buf_Handle this_ring_buf_u32, uint8_t c[], uint32_t *length) {
    uint8_t *TailPtr;
      for (uint32_t i = 0; i < *length; i++) {
          if ((RingBuf_CalculateAvailableSpace(this_ring_buf_u32) < *length) && !RingBuf_GetIsOverwrittingAllowed(this_ring_buf_u32)) {
              return FALSE;
          }
          TailPtr = RingBuf_GetPointerToRingBuf(this_ring_buf_u32) + RingBuf_GetValueOfTail(this_ring_buf_u32);
          *TailPtr = c[i];
          RingBuf_SetUsedNumOfElements(this_ring_buf_u32, RingBuf_GetUsedNumOfElements(this_ring_buf_u32) + 1);
          RingBuf_IncTail(this_ring_buf_u32);
      }
      return TRUE;
}

uint8_t RingBuf_ReadBlock(Ring_Buf_Handle this_ring_buf_u32, uint8_t c[], uint32_t *length) {
    uint32_t i = 0;
    for (; i < *length; i++) {
        if (!RingBuf_IsEmpty(this_ring_buf_u32)) {
            *(c++) = *(RingBuf_GetPointerToRingBuf(this_ring_buf_u32) + RingBuf_GetValueOfHead(this_ring_buf_u32));
            RingBuf_SetUsedNumOfElements(this_ring_buf_u32,
                                               RingBuf_GetUsedNumOfElements(this_ring_buf_u32) - 1);
            RingBuf_IncHead(this_ring_buf_u32);
        } else {
            *length = i;
            return FALSE;
        }
    }
    *length = i;
    return TRUE;
}

uint8_t RingBuf_Observe(Ring_Buf_Handle this_ring_buf_u32, uint8_t c[], uint32_t offset_u32, uint32_t *length) {
    uint8_t *ptr;
    RingBuf_CalculateAvailableSpace(this_ring_buf_u32);
    if (RingBuf_GetUsedNumOfElements(this_ring_buf_u32) <= offset_u32) {
        *length = OFFSET_ERROR;
        return FALSE;
    }
    uint32_t i = 0;
    for (; i < *length; i++) {
        ptr = RingBuf_GetPhysicalAddressOfHeadPlusOffset(this_ring_buf_u32, offset_u32 + i);
        if (ptr != 0) 
        {
            *(c++) = *ptr;
        } else {
            *length = i;
            return FALSE;
        }
    }
    *length = i;
    return TRUE;
}

uint32_t RingBuf_Search(Ring_Buf_Handle this_ring_buf_u32, uint8_t c, uint32_t offset_u32) {
    uint8_t *ptr;
    RingBuf_CalculateAvailableSpace(this_ring_buf_u32);
    if (RingBuf_GetUsedNumOfElements(this_ring_buf_u32) <= offset_u32) {
        return OFFSET_ERROR;
    }
    uint8_t i = 0;
    for (; i < RingBuf_GetUsedNumOfElements(this_ring_buf_u32); i++) {
        ptr = RingBuf_GetPhysicalAddressOfHeadPlusOffset(this_ring_buf_u32, offset_u32 + i);
        if (*ptr == c) {
            return i;
        }
    }
    return NOT_FOUND;
}

uint8_t *RingBuf_GetPointerToRingBuf(Ring_Buf_Handle this_ring_buf_u32) {
    return (*this_ring_buf_u32).p_ringBuf_u8;
}

uint8_t RingBuf_GetUsedNumOfElements(Ring_Buf_Handle this_ring_buf_u32) {
    return (*this_ring_buf_u32).usedNumOfElements_s32;
}
    
int32_t RingBuf_GetCalculateAvailableSpace(Ring_Buf_Handle this_ring_buf_u32) {
    return (RingBuf_CalculateAvailableSpace(this_ring_buf_u32));
}


uint8_t RingBuf_GetOwner(Ring_Buf_Handle this_ring_buf_u32) {
    return (*this_ring_buf_u32).owner_u8;
}

uint8_t RingBuf_SetOwner(Ring_Buf_Handle this_ring_buf_u32, uint8_t owner_u8) {
    (*this_ring_buf_u32).owner_u8 = owner_u8;
    return RingBuf_GetOwner(this_ring_buf_u32) == owner_u8;
}

uint8_t RingBuf_SetProcessInstanceIndex(Ring_Buf_Handle this_ring_buf_u32, uint8_t processInstanceIndex_u8) {
    return (*this_ring_buf_u32).processInstanceIndex_u8 = processInstanceIndex_u8;
}

uint8_t RingBuf_GetProcessInstanceIndex(Ring_Buf_Handle this_ring_buf_u32) {
    return (*this_ring_buf_u32).processInstanceIndex_u8;
}

uint8_t RingBuf_SetSystemInstanceIndex(Ring_Buf_Handle this_ring_buf_u32, uint8_t systemInstanceIndex_u8) {
    return (*this_ring_buf_u32).systemInstanceIndex_u8 = systemInstanceIndex_u8;
}

/* Private Functions -----------------------------------------------------------------------------------------------------------*/
/**
  ********************************************************************************************************************************
  * @brief   Private observer function for obtaining a copy of the value of head_s32. 
  * @details 
  * @param   *this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t RingBuf_GetValueOfHead(Ring_Buf_Handle this_ring_buf_u32) {
    return (*this_ring_buf_u32).head_s32;
}

/**
  ********************************************************************************************************************************
  * @brief   Private function for updating the value of head_s32.
  * @details 
  * @param   *this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t RingBuf_SetValueOfHead(Ring_Buf_Handle this_ring_buf_u32, uint8_t head_s32) {
    return (*this_ring_buf_u32).head_s32 = head_s32;
}

/**
  ********************************************************************************************************************************
  * @brief   Private observer function for obtaining a copy of the value of tail_s32.  
  * @details 
  * @param   *this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t RingBuf_GetValueOfTail(Ring_Buf_Handle this_ring_buf_u32) {
    return (*this_ring_buf_u32).tail_s32;
}

/**
  ********************************************************************************************************************************
  * @brief   Private function for updating the value of tail_s32. 
  * @details 
  * @param   *this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t RingBuf_SetValueOfTail(Ring_Buf_Handle this_ring_buf_u32, uint8_t tail_s32) {
    return (*this_ring_buf_u32).tail_s32 = tail_s32;
}

/**
  ********************************************************************************************************************************
  * @brief   Private function for incrementing head_s32 by 1.
  * @details This function takes care of incrementing the pointer to the head of the ring buffer by 1 taking into consideration
  *             the linear implementation in physical memory.
  * @param   this_ring_buf_u32	The index that the kernel uses to uniquely track of this instance (systemInstanceIndex_u8).
  * @return  Returns TRUE if the operation was successful, FALSE otherwise.
  ********************************************************************************************************************************
  */
static uint8_t RingBuf_IncHead(Ring_Buf_Handle this_ring_buf_u32) {
    if (RingBuf_IsEmpty(this_ring_buf_u32)) {
        return FALSE;
    } else {
        if (RingBuf_GetTotalNumOfElements(this_ring_buf_u32) - RingBuf_GetValueOfHead(this_ring_buf_u32) <= 1) {
            RingBuf_SetValueOfHead(this_ring_buf_u32, 0);
        } else {
            RingBuf_SetValueOfHead(this_ring_buf_u32, RingBuf_GetValueOfHead(this_ring_buf_u32) + 1);
        }
    }
    return TRUE;
}

/**
  ********************************************************************************************************************************
  * @brief   Private function for incrementing tail_s32 by 1.
  * @details This function takes care of incrementing the pointer to the head of the ring buffer by 1 taking into consideration
  *             the linear implementation in physical memory.
  * @param   this_ring_buf_u32	The index that the kernel uses to uniquely track of this instance (systemInstanceIndex_u8).
  * @return  Returns TRUE if the operation was successful, FALSE otherwise.
  ********************************************************************************************************************************
  */
static uint8_t RingBuf_IncTail(Ring_Buf_Handle this_ring_buf_u32) {
    if (RingBuf_IsFull(this_ring_buf_u32)) {
        if (RingBuf_GetIsOverwrittingAllowed(this_ring_buf_u32)) {
            RingBuf_IncHead(this_ring_buf_u32);
            if (RingBuf_GetTotalNumOfElements(this_ring_buf_u32) - RingBuf_GetValueOfTail(this_ring_buf_u32) <=
                1) {
                RingBuf_SetValueOfTail(this_ring_buf_u32, 0);
            } else {
                RingBuf_SetValueOfTail(this_ring_buf_u32, RingBuf_GetValueOfTail(this_ring_buf_u32) + 1);
            }
            return TRUE;
        }
        return FALSE;
    } else {
        if (RingBuf_GetTotalNumOfElements(this_ring_buf_u32) - RingBuf_GetValueOfTail(this_ring_buf_u32) <= 1) {
            RingBuf_SetValueOfTail(this_ring_buf_u32, 0);
        } else {
            RingBuf_SetValueOfTail(this_ring_buf_u32, RingBuf_GetValueOfTail(this_ring_buf_u32) + 1);
        }
        return TRUE;
    }
}

/**
  ********************************************************************************************************************************
  * @brief   Private function for obtaining a copy of the physial address of a particular offset relative to head_s32.
  * @details This function takes care of offsets that would return addresses outside of the buffer's address range allocation.
  * @param   this_ring_buf_u32	The index that the kernel uses to uniquely track of this instance (systemInstanceIndex_u8).
  * @param   offset_u32	            This value specifies the distance to be added to the head_s32 physical address location.
  * @return  Returns the physial address location corresponding this ring buffer instance's head_s32 address plus an offset.
  ********************************************************************************************************************************
  */
static uint8_t *RingBuf_GetPhysicalAddressOfHeadPlusOffset(Ring_Buf_Handle this_ring_buf_u32, uint32_t offset_u32) {
    uint32_t diff;
    RingBuf_CalculateAvailableSpace(this_ring_buf_u32);
    if ((RingBuf_IsEmpty(this_ring_buf_u32)) || (RingBuf_GetUsedNumOfElements(this_ring_buf_u32) < offset_u32)) 
    {
        return 0;
    } 
    else if (RingBuf_CalculateAvailableSpace(this_ring_buf_u32) > 0) 
    {
        if (RingBuf_GetUsedNumOfElements(this_ring_buf_u32) > offset_u32) 
        {
          uint8_t wrapAroundAns_u8 = RingBuf_GetValueOfHead(this_ring_buf_u32) + offset_u32;
          if(wrapAroundAns_u8 >= RingBuf_GetTotalNumOfElements(this_ring_buf_u32))
          {
            wrapAroundAns_u8 -= (RingBuf_GetTotalNumOfElements(this_ring_buf_u32) ) ;   
          }   
          uint8_t* temp = RingBuf_GetPointerToRingBuf(this_ring_buf_u32);
          return (temp + wrapAroundAns_u8)  ;
        } 
        else 
        {
            return 0;
        }
    } 
    else 
    {
        if ((diff = RingBuf_GetTotalNumOfElements(this_ring_buf_u32) -
                    (RingBuf_GetValueOfHead(this_ring_buf_u32) + offset_u32)) <= 0) {
            return (RingBuf_GetPointerToRingBuf(this_ring_buf_u32) + abs(diff));
        } else {
            return (RingBuf_GetPointerToRingBuf(this_ring_buf_u32) + RingBuf_GetValueOfHead(this_ring_buf_u32) +
                    offset_u32);
        }
    }
}

/**
  ********************************************************************************************************************************
  * @brief   Returns TRUE if there are no used elements in the ring buffer instance, falses otherwise.
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @return  eturns TRUE if there are no used elements in the ring buffer instance, falses otherwise.
  ********************************************************************************************************************************
  */
static uint8_t RingBuf_IsEmpty(Ring_Buf_Handle this_ring_buf_u32) {
    return RingBuf_GetValueOfHead(this_ring_buf_u32) == RingBuf_GetValueOfTail(this_ring_buf_u32);
}

/**
  ********************************************************************************************************************************
  * @brief   Returns TRUE if the number of used elements is equal to the total number of elements in the ring buffer instance.
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on. 
  * @return  Returns TRUE if the number of used elements is equal to the total number of elements in the ring buffer instance.
  ********************************************************************************************************************************
  */
static uint8_t RingBuf_IsFull(Ring_Buf_Handle this_ring_buf_u32) {
    return (RingBuf_GetValueOfTail(this_ring_buf_u32) - RingBuf_GetValueOfHead(this_ring_buf_u32) ==
            (RingBuf_GetTotalNumOfElements(this_ring_buf_u32) - 1)) ||
           (RingBuf_GetValueOfTail(this_ring_buf_u32) - RingBuf_GetValueOfHead(this_ring_buf_u32) == -1);
}

/**
  ********************************************************************************************************************************
  * @brief   Returns the number of unused elements in the ring buffer instance.
  * @details Returns the number of unused elements in the ring buffer accounting for cases where the head is in equal to, in front
  *                                 of, or behind the tail. Automatically updates usedNumOfElements_s32 when called.
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @return  Returns the number of unused elements in the ring buffer instance.
  * @Tested by PL 27OCT2020
  ********************************************************************************************************************************
  */
static int32_t RingBuf_CalculateAvailableSpace(Ring_Buf_Handle this_ring_buf_u32) {
    int32_t head_minus_tail = 0;
    if ((head_minus_tail = RingBuf_GetValueOfTail(this_ring_buf_u32) - RingBuf_GetValueOfHead(this_ring_buf_u32)) >
        0) {
        RingBuf_SetUsedNumOfElements(this_ring_buf_u32, abs(head_minus_tail));
        return RingBuf_GetTotalNumOfElements(this_ring_buf_u32) - head_minus_tail;
    } else {
      if(head_minus_tail != 0){
        RingBuf_SetUsedNumOfElements(this_ring_buf_u32,
                                           RingBuf_GetTotalNumOfElements(this_ring_buf_u32) - abs(head_minus_tail));
        return abs(head_minus_tail);
      }
      else
      { 
        RingBuf_SetUsedNumOfElements(this_ring_buf_u32, RingBuf_GetTotalNumOfElements(this_ring_buf_u32) - 1);
        return RingBuf_GetTotalNumOfElements(this_ring_buf_u32);
      }
    }  
}

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   p_ringBuf_u8        A pointer to the memory location where thie ring buffer instance stores it's values.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t *RingBuf_SetPointerToRingBuf(Ring_Buf_Handle this_ring_buf_u32, uint8_t *p_ringBuf_u8) {
    return (*this_ring_buf_u32).p_ringBuf_u8 = p_ringBuf_u8;
}

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32       The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   usedNumOfElements_s32   TODO: This function should not be accessible from outside the source file.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t RingBuf_SetUsedNumOfElements(Ring_Buf_Handle this_ring_buf_u32, uint32_t usedNumOfElements_s32) {
    return (*this_ring_buf_u32).usedNumOfElements_s32 = usedNumOfElements_s32;
}

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t RingBuf_GetTotalNumOfElements(Ring_Buf_Handle this_ring_buf_u32) {
    return (*this_ring_buf_u32).totalNumOfElements_u32;
}

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32       The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   totalNumOfElements_u32  TODO: This function should not be accessible from outside the source file.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t RingBuf_SetTotalNumOfElements(Ring_Buf_Handle this_ring_buf_u32, uint32_t totalNumOfElements_u32) {
    (*this_ring_buf_u32).totalNumOfElements_u32 = totalNumOfElements_u32;
    return RingBuf_GetTotalNumOfElements(this_ring_buf_u32) == totalNumOfElements_u32;
}

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t RingBuf_GetIsOverwrittingAllowed(Ring_Buf_Handle this_ring_buf_u32) {
    return (*this_ring_buf_u32).is_OverwrittingAllowed_u8;
}

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32       The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   is_OverwrittingAllowed_u8  Flag for tracking whether it is allowed for a full ring buffer to be overwritten.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t RingBuf_SetIsOverwrittingAllowed(Ring_Buf_Handle this_ring_buf_u32, uint8_t is_OverwrittingAllowed_u8) {
    return (*this_ring_buf_u32).is_OverwrittingAllowed_u8 = is_OverwrittingAllowed_u8;
}

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   access_mode_u8         This flag tracks whether the ring buffer instance is read-only, write-only, or broadcasted.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t RingBuf_SetAccessMode(Ring_Buf_Handle this_ring_buf_u32, uint8_t access_mode_u8) {
    return (*this_ring_buf_u32).accessMode_u8 = access_mode_u8;
}

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   p_user_list_u8         A pointer to the memory location that stores the list of processes that use this ring buffer
  *                                     instance.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t *RingBuf_SetUserList(Ring_Buf_Handle this_ring_buf_u32, uint8_t *p_user_list_u8) {
    return (*this_ring_buf_u32).p_userList_u8 = p_user_list_u8;
}

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ring_buf_u32   The ring buffer handle referencing the ring buffer that the function must operate on.
  * @param   user_list_size_u8      TODO: This function should not be accessible ALONE from outside the source file.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t RingBuf_SetUserListSize(Ring_Buf_Handle this_ring_buf_u32, uint8_t user_list_size_u8) {
    return (*this_ring_buf_u32).userListSize_u8 = user_list_size_u8;
}