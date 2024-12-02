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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <stm32f1xx_ll_rcc.h>
#include <stm32f1xx_ll_utils.h>

/************************************
 * EXTERN VARIABLES
 ************************************/

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

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
static void check_restart_issues(void);

/************************************
 * STATIC FUNCTIONS
 ************************************/
static void check_restart_issues(void)
{
   if (true == LL_RCC_IsActiveFlag_HSECSS())
   {
      printf("Reset cause: HSECSS\n");
   }
   if (true == LL_RCC_IsActiveFlag_IWDGRST())
   {
      printf("Reset cause: IWDGRST\n");
   }
   if (true == LL_RCC_IsActiveFlag_LPWRRST())
   {
      printf("Reset cause: LPWRRST\n");
   }
   if (true == LL_RCC_IsActiveFlag_PINRST())
   {
      printf("Reset cause: PINRST\n");
   }
   if (true == LL_RCC_IsActiveFlag_PORRST())
   {
      printf("Reset cause: PORRST\n");
   }
   if (true == LL_RCC_IsActiveFlag_SFTRST())
   {
      printf("Reset cause: SFTRST\n");
   }
   if (true == LL_RCC_IsActiveFlag_WWDGRST())
   {
      printf("Reset cause: WWDGRST\n");
   }
   LL_RCC_ClearResetFlags();
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
void Device_Info(void)
{
   printf("#############################\n");
   check_restart_issues();
   printf("Flash size: 0x%lx\n", LL_GetFlashSize());
   printf("Device ID: 0x%lx 0x%lx 0x%lx\n", LL_GetUID_Word0(), LL_GetUID_Word1(),
          LL_GetUID_Word2());
   printf("#############################\n");
}
