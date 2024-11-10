/**
 ********************************************************************************
 * @file    device_info.c
 * @author  Mikolaj Pieklo
 * @date    10.11.2023
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "device_info.h"

#include <stdint.h>
#include <stdio.h>

/************************************
 * EXTERN VARIABLES
 ************************************/

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/
#define STM32_REG_FLASH_SIZE  0x1FFFF7E0U
#define STM32_REG_DEVICE_ID_1 0x1FFFF7E8U
#define STM32_REG_DEVICE_ID_2 0x1FFFF7EAU
#define STM32_REG_DEVICE_ID_3 0x1FFFF7ECU
#define STM32_REG_DEVICE_ID_4 0x1FFFF7F0U

/************************************
 * PRIVATE TYPEDEFS
 ************************************/

/************************************
 * STATIC VARIABLES
 ************************************/

/************************************
 * GLOBAL VARIABLES
 ************************************/

/************************************
 * STATIC FUNCTION PROTOTYPES
 ************************************/

/************************************
 * STATIC FUNCTIONS
 ************************************/

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
void Device_Info(void)
{
   printf("#########################################\n");
   uint16_t *u16_info = (uint16_t *)STM32_REG_FLASH_SIZE;
   printf("Flash size 0x%x\n", *u16_info);
   u16_info = (uint16_t *)STM32_REG_DEVICE_ID_1;
   printf("Device ID1 0x%x\n", *u16_info);
   u16_info = (uint16_t *)STM32_REG_DEVICE_ID_2;
   printf("Device ID2 0x%x\n", *u16_info);
   uint32_t *u32_info = (uint32_t *)STM32_REG_DEVICE_ID_3;
   printf("Device ID3 0x%lx\n", *u32_info);
   u32_info = (uint32_t *)STM32_REG_DEVICE_ID_4;
   printf("Device ID4 0x%lx\n", *u32_info);
}
