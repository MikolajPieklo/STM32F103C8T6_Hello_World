/**
 ********************************************************************************
 * @file    crc16.h
 * @author  Mikolaj Pieklo
 * @date    05.11.2024
 * @brief
 ********************************************************************************
 */

#ifndef __CRC16_H__
#define __CRC16_H__

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

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
uint16_t CRC16_Calculate(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif