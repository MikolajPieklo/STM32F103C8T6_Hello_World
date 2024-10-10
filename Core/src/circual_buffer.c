/**
 ********************************************************************************
 * @file    circual_buffer.c
 * @author  Mikolaj Pieklo
 * @date    15.12.2023
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include <circual_buffer.h>
#include <stdint.h>
#include <string.h>
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
volatile Circual_Buffer_T cbp = {0, 0};
volatile uint8_t Circual_Buffer[CIRCUAL_BUFFER_SIZE];

/************************************
 * STATIC FUNCTION PROTOTYPES
 ************************************/

/************************************
 * STATIC FUNCTIONS
 ************************************/

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
void Circual_Buffer_Insert_Char(uint8_t c)
{
   if (cbp.head > (CIRCUAL_BUFFER_SIZE - 1))
   {
      cbp.head = 0;
   }
   Circual_Buffer[cbp.head] = c;
   cbp.head++;
   if (cbp.head == CIRCUAL_BUFFER_SIZE)
   {
      cbp.head = 0;
   }
}

void Circual_Buffer_Insert_Text(uint8_t *text, uint32_t len)
{
   if ((cbp.head + len) <= (CIRCUAL_BUFFER_SIZE - 1))
   {
      memcpy(Circual_Buffer + cbp.head, text, len);
      cbp.head += len;
   } else if (cbp.head + len > (CIRCUAL_BUFFER_SIZE - 1))
   {
      uint32_t lenToEndLine = (CIRCUAL_BUFFER_SIZE - 1) - cbp.head;
      memcpy(Circual_Buffer + cbp.head, text, lenToEndLine);
      memcpy(Circual_Buffer, (text + lenToEndLine), (len - lenToEndLine));
      cbp.head = (len - lenToEndLine);
   } else
   {
      // do nothing
   }
}