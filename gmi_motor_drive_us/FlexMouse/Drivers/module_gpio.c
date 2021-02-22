/**
  ********************************************************************************************************************************
  * @file    module_gpio.c 
  * @author  Pamela Lee
  * @brief   Main driver module for GPIO
  * @details    
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "driver_gpio.h"

/* scheduler handle declaration */
#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];

enum {
    INIT_MODULE,
    RUN_MODULE,
    // additional states to be added here as necessary.
    IRQ_MODULE = DEFAULT_IRQ_STATE,
    KILL_MODULE = KILL_APP
};

uint8_t moduleGPIO_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8,
                         uint8_t irq_id_u8) {
    uint8_t return_state_u8 = INIT_MODULE;
    Ram_Buf_Handle gpio_shared_mem_location_u32;
    switch (next_state_u8) {
        case INIT_MODULE: {
            // Initialize GPIO.
            GPIOInit();

            // Find the structured memory for the GPIO driver module, by searching for the GPIO onwer id.
            Ram_Buf_Handle this_ram_buf_u32;
            for (uint8_t i = 0; i < TOTAL_NUM_OF_STRUCT_MEM_INSTANCES; i++) {
                this_ram_buf_u32 = &sharedMemArray[i];
                if (RamBuf_GetOwner(this_ram_buf_u32) == drv_id_u8) {
                    gpio_shared_mem_location_u32 = &sharedMemArray[i];
                }
            }

            // Attach the structured memory to the process's master shared memory.
            uint8_t index = getProcessInfoIndex(drv_id_u8);
            if (index != INDEX_NOT_FOUND) {
                processInfoTable[index].Sched_DrvData.irqState_u8 = PROCESS_STATUS_KILLED;
                processInfoTable[index].Sched_DrvData.p_masterSharedMem_u32 = gpio_shared_mem_location_u32;
//                processInfoTable[index].Sched_DrvData.attachedModuleID = MODULE_GPIO;
            }

            return_state_u8 = RUN_MODULE;
            break;
        }
        case RUN_MODULE: {
            return_state_u8 = KILL_MODULE;
            break;
        }
        case KILL_MODULE: {
            // The GPIO driver module must only be executed once.
            // Setting processStatus_u8 to PROCESS_STATUS_KILLED prevents the scheduler main loop from calling this module again.
            uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
            if (table_index_u8 != INDEX_NOT_FOUND) {
                processInfoTable[table_index_u8].Sched_DrvData.processStatus_u8 = PROCESS_STATUS_KILLED;
            }
            return_state_u8 = KILL_MODULE;
            break;
        }
        default:
            return_state_u8 = 10;
            break;
    }
    return return_state_u8;
}