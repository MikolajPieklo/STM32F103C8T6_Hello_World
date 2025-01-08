/**
 ********************************************************************************
 * @file    lcd12864.h
 * @author  Mikolaj Pieklo
 * @date    20.11.2024
 * @brief
 ********************************************************************************
 */

#ifndef __LCD12864_H__
#define __LCD12864_H__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include <stdbool.h>
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
void LCD12864_Init(void);

void LCD12864_Set_Char(void);

void LCD12864_Graphic_Mode(bool enable);

void LCD12864_Fill_Screen(void);

void LCD12864_Clear_Screen(void);

#ifdef __cplusplus
}
#endif

#endif