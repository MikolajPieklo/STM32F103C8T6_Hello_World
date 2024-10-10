/**
 ********************************************************************************
 * @file    circual_buffer.h
 * @author  Mikolaj Pieklo
 * @date    15.12.2023
 * @brief
 ********************************************************************************
 */

#ifndef __CIRCUAL_BUFFER_H__
#define __CIRCUAL_BUFFER_H__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include <stdint.h>

/************************************
 * MACROS AND DEFINES
 ************************************/
#define CIRCUAL_BUFFER_SIZE 1024

/************************************
 * TYPEDEFS
 ************************************/
typedef struct Circual_Buffer_Typedef
{
   uint32_t tail;
   uint32_t head;
} Circual_Buffer_T;

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
void Circual_Buffer_Insert_Char(uint8_t c);

void Circual_Buffer_Insert_Text(uint8_t *text, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif