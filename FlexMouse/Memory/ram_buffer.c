/**
  ********************************************************************************************************************************
  * @file    ram_buffer.c 
  * @author  Pamela Lee
  * @brief   This file implements the interface for the RAM buffer data structure.
  * @details This file contains definitions for constants, macros, global variables, and function prototypes required to
  *          implement a single RAM buffer (consequitive RAM area allocation) data structure.
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "ram_buffer.h"

#include <stdlib.h>
#include <string.h>

/* Content ---------------------------------------------------------------------------------------------------------------------*/

/* Private Function Prototypes -------------------------------------------------------------------------------------------------*/
static uint8_t *RamBuf_SetPointerToRamBuf(Ram_Buf_Handle this_ram_buf_u32, uint8_t *p_RamBuf_u8);
static uint8_t RamBuf_GetTotalNumOfElements(Ram_Buf_Handle this_ram_buf_u32);
static uint8_t RamBuf_SetTotalNumOfElements(Ram_Buf_Handle this_ram_buf_u32, uint32_t totalNumOfElements_u32);
static uint8_t RamBuf_SetAccessMode(Ram_Buf_Handle this_ram_buf_u32, uint8_t access_mode_u8);
static uint8_t *RamBuf_SetUserList(Ram_Buf_Handle this_ram_buf_u32, uint8_t *p_user_list_u8);
static uint8_t RamBuf_SetUserListSize(Ram_Buf_Handle this_ram_buf_u32, uint8_t user_list_size_u8);

/* Public Functions ------------------------------------------------------------------------------------------------------------*/
void RamBuf_Initialize(Ram_Buf_Handle this_ram_buf_u32, uint8_t owner_u8, uint8_t ram_buf_size_u8,
                          uint8_t access_mode_u8, uint8_t *p_user_list_u8, uint8_t user_list_size_u8) {
    uint8_t* tmpryBuf;                                                                  //check any Heap memory alocation error                 //house keeping code
    if( (tmpryBuf = malloc(ram_buf_size_u8)) == NULL) Ram_mallocError++;                //check any Heap memory alocation error                 //house keeping code
    RamBuf_SetPointerToRamBuf(this_ram_buf_u32, tmpryBuf);
    RamBuf_SetOwner(this_ram_buf_u32, owner_u8);
    RamBuf_SetAccessMode(this_ram_buf_u32, access_mode_u8);
    RamBuf_SetUserList(this_ram_buf_u32, p_user_list_u8);
    RamBuf_SetUserListSize(this_ram_buf_u32, user_list_size_u8);
    RamBuf_SetTotalNumOfElements(this_ram_buf_u32, ram_buf_size_u8);
    RamBuf_SetProcessInstanceIndex(this_ram_buf_u32, 255);
}

void RamBuf_Release(Ram_Buf_Handle this_ram_buf_u32) {
    RamBuf_SetOwner(this_ram_buf_u32, 255);
    RamBuf_SetAccessMode(this_ram_buf_u32, 1);
    RamBuf_SetUserList(this_ram_buf_u32, NULL);
    RamBuf_SetUserListSize(this_ram_buf_u32, 0);
    RamBuf_SetTotalNumOfElements(this_ram_buf_u32, 0);
    RamBuf_SetProcessInstanceIndex(this_ram_buf_u32, 255);
}

uint8_t RamBuf_GetOwner(Ram_Buf_Handle this_ram_buf_u32) {
    return (*this_ram_buf_u32).owner_u8;
}

uint8_t RamBuf_SetOwner(Ram_Buf_Handle this_ram_buf_u32, uint8_t owner_u8) {
    (*this_ram_buf_u32).owner_u8 = owner_u8;
    return RamBuf_GetOwner(this_ram_buf_u32) == owner_u8;
}

uint8_t RamBuf_GetProcessInstanceIndex(Ram_Buf_Handle this_ram_buf_u32) {
    return (*this_ram_buf_u32).processInstanceIndex_u8;
}

uint8_t RamBuf_SetProcessInstanceIndex(Ram_Buf_Handle this_ram_buf_u32, uint8_t processInstanceIndex_u8) {
    return (*this_ram_buf_u32).processInstanceIndex_u8 = processInstanceIndex_u8;
}

/* Private Functions -----------------------------------------------------------------------------------------------------------*/

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ram_buf_u32   The RAM buffer handle referencing the RAM buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t *RamBuf_SetPointerToRamBuf(Ram_Buf_Handle this_ram_buf_u32, uint8_t *p_ramBuf_u8) {
    return (*this_ram_buf_u32).p_ramBuf_u8 = p_ramBuf_u8;
}

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ram_buf_u32   The RAM buffer handle referencing the RAM buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t RamBuf_GetTotalNumOfElements(Ram_Buf_Handle this_ram_buf_u32) {
    return (*this_ram_buf_u32).totalNumOfElements_u32;
}

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ram_buf_u32   The RAM buffer handle referencing the RAM buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t RamBuf_SetTotalNumOfElements(Ram_Buf_Handle this_ram_buf_u32, uint32_t totalNumOfElements_u32) {
    (*this_ram_buf_u32).totalNumOfElements_u32 = totalNumOfElements_u32;
    return RamBuf_GetTotalNumOfElements(this_ram_buf_u32) == totalNumOfElements_u32;
}

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ram_buf_u32   The RAM buffer handle referencing the RAM buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t RamBuf_SetAccessMode(Ram_Buf_Handle this_ram_buf_u32, uint8_t access_mode_u8) {
    return (*this_ram_buf_u32).accessMode_u8 = access_mode_u8;
}

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ram_buf_u32   The RAM buffer handle referencing the RAM buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t *RamBuf_SetUserList(Ram_Buf_Handle this_ram_buf_u32, uint8_t *p_user_list_u8) {
    return (*this_ram_buf_u32).p_userList_u8 = p_user_list_u8;
}

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details 
  * @param   this_ram_buf_u32   The RAM buffer handle referencing the RAM buffer that the function must operate on.
  * @return  Returns 
  ********************************************************************************************************************************
  */
static uint8_t RamBuf_SetUserListSize(Ram_Buf_Handle this_ram_buf_u32, uint8_t user_list_size_u8) {
    return (*this_ram_buf_u32).userListSize_u8 = user_list_size_u8;
}