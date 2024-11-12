/**
 ********************************************************************************
 * @file    delay.c
 * @author  Mikolaj Pieklo
 * @date    12.11.2024
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "delay.h"

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
volatile uint32_t SysTickValue;

/************************************
 * STATIC FUNCTION PROTOTYPES
 ************************************/

/************************************
 * STATIC FUNCTIONS
 ************************************/

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
void TS_Delay_ms(uint32_t delay_ms)
{
   uint32_t tickstart = SysTickValue;
   while ((SysTickValue - tickstart) < delay_ms)
   {
   }
}

uint32_t TS_Get_ms(void)
{
   return SysTickValue;
}
