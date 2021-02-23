/**
  ********************************************************************************************************************************
  * @file    struct_memory.c 
  * @author  Oscar Guevara
  * @brief   This file implements the memory abstraction API for the FlexMouse structured memory data structure.
  * @details FlexMouse has two types of available memory: sequential and structured memory. Structured memory, defined in this
  *             file, allows the user to store and retrieve data. Unlike sequential memory, structured memory allows the user to
  *             retrieve any data previously stored at any time, regardless of in what order the data was stored.
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "scheduler.h"
#include "structured_memory.h"

Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE]; // Allocate space for structured memory instances

Ram_Buf_Handle StructMem_CreateInstance(uint8_t owner_u8, uint8_t ram_buf_size_u8, uint8_t access_mode_u8,
                                                  uint8_t *p_user_list_u8, uint8_t user_list_size_u8) {
    Ram_Buf_Handle this_ram_buf_u32;
    uint8_t instance_index_u8 = 0;
    uint8_t max_process_index_u8 = 0;
    for (; instance_index_u8 < TOTAL_NUM_OF_STRUCT_MEM_INSTANCES; instance_index_u8++) {
        this_ram_buf_u32 = &sharedMemArray[instance_index_u8];
        if (RamBuf_GetOwner(this_ram_buf_u32) == owner_u8) {
            if (RamBuf_GetProcessInstanceIndex(this_ram_buf_u32) > max_process_index_u8) {
                max_process_index_u8 = RamBuf_GetProcessInstanceIndex(this_ram_buf_u32);
            }
        }
    }
    for (instance_index_u8 = 0; instance_index_u8 < TOTAL_NUM_OF_STRUCT_MEM_INSTANCES; instance_index_u8++) {
        this_ram_buf_u32 = &sharedMemArray[instance_index_u8];
        if (RamBuf_GetOwner(this_ram_buf_u32) == 255) {
            RamBuf_Initialize(this_ram_buf_u32, owner_u8, ram_buf_size_u8, 0, 0, 0);
            RamBuf_SetProcessInstanceIndex(this_ram_buf_u32, max_process_index_u8 + 1);
            return (&sharedMemArray[instance_index_u8]);
        }
    }
    return (0);
}

uint8_t StructMem_DestroyInstance(uint8_t owner_u8, Ram_Buf_Handle this_ram_buf_u32) {
    Ram_Buf_Handle temp_ram_buf_u32;
    uint8_t instance_index_u8 = 0;
    for (; instance_index_u8 < TOTAL_NUM_OF_STRUCT_MEM_INSTANCES; instance_index_u8++) {
        temp_ram_buf_u32 = &sharedMemArray[instance_index_u8];
        if (RamBuf_GetOwner(temp_ram_buf_u32) == owner_u8) {
            if ((this_ram_buf_u32 == 0) || (this_ram_buf_u32 == temp_ram_buf_u32)) {
                RamBuf_Release(this_ram_buf_u32);
                return (TRUE);
            }
        }
    }
    return (FALSE);
}

uint8_t StructMem_InitBufs(void) {
    Ram_Buf_Handle this_ram_buf_u32;
    for (uint8_t instance_index_u8 = 0; instance_index_u8 < TOTAL_NUM_OF_STRUCT_MEM_INSTANCES; instance_index_u8++) {
        this_ram_buf_u32 = &sharedMemArray[instance_index_u8];
        RamBuf_SetOwner(this_ram_buf_u32, NO_OWNER);
        RamBuf_SetProcessInstanceIndex(this_ram_buf_u32, NO_OWNER_ID);
    }
    return TRUE;
}