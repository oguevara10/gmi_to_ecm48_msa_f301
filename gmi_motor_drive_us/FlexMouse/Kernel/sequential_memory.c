/**
  ********************************************************************************************************************************
  * @file    sequential_memory.c
  * @author  Oscar Guevara
  * @brief   This file implements the memory abstraction API for the FlexMouse sequential memory data structure.
  * @details FlexMouse has two types of available memory: sequential and structured memory. Sequential memory, defined in this
  *             file, allows the user to store and retrieve data. As the name implies, data can only be accessed in the same
  *             order as it was stored. 
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "scheduler.h"
#include "sequential_memory.h"

Ring_Buf seqMemArray[SEQ_MEM_ARRAY_SIZE]; // Allocate space for sequential memory instances

Ring_Buf_Handle SeqMem_CreateInstance(uint8_t owner_u8, uint8_t ring_buf_size_u8, uint8_t access_mode_u8,
                                                   uint8_t *p_user_list_u8, uint8_t user_list_size_u8) {
    Ring_Buf_Handle this_ring_buf_u32;
    uint8_t instance_index_u8 = 0;
    uint8_t max_process_index_u8 = 0;
    for (; instance_index_u8 < TOTAL_NUM_OF_SEQ_MEM_INSTANCES; instance_index_u8++) {
        this_ring_buf_u32 = &seqMemArray[instance_index_u8];
        if (RingBuf_GetOwner(this_ring_buf_u32) == owner_u8) {
            if (RingBuf_GetProcessInstanceIndex(this_ring_buf_u32) > max_process_index_u8) {
                max_process_index_u8 = RingBuf_GetProcessInstanceIndex(this_ring_buf_u32);
            }
        }
    }
    instance_index_u8 = 0;
    for (; instance_index_u8 < TOTAL_NUM_OF_SEQ_MEM_INSTANCES; instance_index_u8++) {
        this_ring_buf_u32 = &seqMemArray[instance_index_u8];
        if (RingBuf_GetOwner(this_ring_buf_u32) == 255) {
            RingBuf_Initialize(this_ring_buf_u32, owner_u8, ring_buf_size_u8, 0, 0, 0);
            RingBuf_SetProcessInstanceIndex(this_ring_buf_u32, max_process_index_u8 + 1);
            RingBuf_SetSystemInstanceIndex(this_ring_buf_u32, instance_index_u8);
            return (&seqMemArray[instance_index_u8]);
        }
    }
    return (0);
}

uint8_t SeqMem_DestroyInstance(uint8_t owner_u8, Ring_Buf_Handle this_ring_buf_u32) {
    Ring_Buf_Handle temp_ring_buf_u32;
    uint8_t instance_index_u8 = 0;
    for (; instance_index_u8 < TOTAL_NUM_OF_SEQ_MEM_INSTANCES; instance_index_u8++) {
        temp_ring_buf_u32 = &seqMemArray[instance_index_u8];
        if (RingBuf_GetOwner(temp_ring_buf_u32) == owner_u8) {
            if ((this_ring_buf_u32 == NULL) || (this_ring_buf_u32 == temp_ring_buf_u32)) {
                RingBuf_ClearContents(this_ring_buf_u32);
                return (TRUE);
            }
        }
    }
    return (FALSE);
}

uint8_t SeqMem_InitBufs(void) {
    Ring_Buf_Handle this_ring_buf_u32;
    for (uint8_t instance_index_u8 = 0; instance_index_u8 < TOTAL_NUM_OF_SEQ_MEM_INSTANCES; instance_index_u8++) {
        this_ring_buf_u32 = &seqMemArray[instance_index_u8];
        RingBuf_SetOwner(this_ring_buf_u32, NO_OWNER);
        RingBuf_SetProcessInstanceIndex(this_ring_buf_u32, NO_OWNER_ID);
    }
    return TRUE;
}