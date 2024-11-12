/**
 ********************************************************************************
 * @file    SH1106.c
 * @author  Mikolaj Pieklo
 * @date    13.10.2023
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "sh1106.h"

#include <delay.h>
#include <fonts.h>
#include <i2c.h>
#include <string.h>

/************************************
 * EXTERN VARIABLES
 ************************************/

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/
#define SH1106_ADDRESS 0x78

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
static void sh1106_clear_screen(void);
static void sh1106_test(void);
static void sh1106_set_cursor(uint8_t x, uint8_t y);
static void sh1106_send_char(uint8_t x, uint8_t y, uint8_t ch);

/************************************
 * STATIC FUNCTIONS
 ************************************/
static void sh1106_clear_screen(void)
{
   uint8_t idx, i = 0x00;
   uint8_t j = 0xB0;

   for (j = 0xB0; j < 0xC0; j++)
   {
      I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, j);
      for (i = 0x10; i < 0x20; i++)
      {
         I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, i);
         for (idx = 0; idx < 0x10; idx++)
         {
            I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, idx);
            I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x40, 0x00);
         }
      }
   }
}

static void sh1106_test(void)
{
   I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x00); // Column address LO max 0x0F
   I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x10); // Column address HI max 0x1F
   I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x40); // Start Line max 0x7F
   I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xB0); // Start Page 0xBF
   uint8_t idx, i = 0x00;
   uint8_t j = 0xB0;
   for (j = 0xB0; j < 0xB8; j++)
   {
      I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, j);
      I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x00); // Column address LO max 0x0F
      I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x10); // Column address HI max 0x1F
      for (i = 0x10; i < 0x18; i++)
      {
         I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, i);
         for (idx = 0; idx < 0x10; idx++)
         {
            I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, idx);
            I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x40, 0xFF);
         }
      }
      TS_Delay_ms(1000);
   }
   // I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x40, 0x81);
}

static void sh1106_set_cursor(uint8_t x, uint8_t y)
{
   do
   {
      if ((x > 128) || (y > 64))
      {
         break;
      }

      switch ((uint8_t) (y / 8))
      {
      case 0:
         I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xB0); // Start Page 0xBF
         break;
      case 1:
         I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xB1); // Start Page 0xBF
         break;
      case 2:
         I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xB2); // Start Page 0xBF
         break;
      case 3:
         I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xB3); // Start Page 0xBF
         break;
      case 4:
         I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xB4); // Start Page 0xBF
         break;
      case 5:
         I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xB5); // Start Page 0xBF
         break;
      case 6:
         I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xB6); // Start Page 0xBF
         break;
      case 7:
         I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xB7); // Start Page 0xBF
         break;
      case 8:
         I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xB8); // Start Page 0xBF
         break;
      }

      uint8_t a = (uint8_t) (x / 16);
      uint8_t b = (uint8_t) (x % 16);

      I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, b); // Column address LO max 0x0F
      I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00,
                                    0x10 + a); // Column address HI max 0x1F
   } while (0);
}

static void sh1106_send_char(uint8_t x, uint8_t y, uint8_t ch)
{
   uint8_t i = 0;
   sh1106_set_cursor(x, y);

   I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x40);
   for (i = 0; i < 6; i++)
   {
      I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x40, font_6x8[ch][i]);
   }
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
void SH1106_Init(void)
{
   bool status = true;

   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xAE); // display OFF

   status &=
       I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xD5); // Display clock divide
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x80);
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xA8); // multiplex ratio
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x3F); // 1F
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xD3); // display offset
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x00);
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x40); // Start line
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x8D); // charge pump
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x14);
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x20);
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x00);
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xA1); // remap
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xC8); // SH1106_COMSCANDEC
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xDA); // com pins
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x12); // 02
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x81); // set contrast
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x8F);
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xD9); // precharge period
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x22);
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xDB); // VCOM deselect
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x40);

   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xA4); // display ON
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xA6); // normal display

   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xAF); // display ON
   TS_Delay_ms(1);

   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00,
                                           0x00); // Column address LO max 0x0F
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00,
                                           0x10); // Column address HI max 0x1F
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0x40); // Start Line max 0x7F
   status &= I2C_Master_Reg8_Transmit_Byte(I2C2, SH1106_ADDRESS, 0x00, 0xB0); // Start Page max 0xBF

   sh1106_clear_screen();

   return;
}

void SH1106_Send_Text(uint8_t x, uint8_t y, char *text)
{
   uint8_t size = strlen((const char *) text);
   uint8_t i = 0;

   for (i = 0; i < size; i++)
   {
      sh1106_send_char((6 * i) + x, y, *(text + i));
   }
}
