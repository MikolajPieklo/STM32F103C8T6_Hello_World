/**
 ********************************************************************************
 * @file    lcd12864.c
 * @author  Mikolaj Pieklo
 * @date    20.11.2024
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "lcd12864.h"

#include <stddef.h>

#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_gpio.h>

#include <delay.h>
#include <spi.h>


/************************************
 * EXTERN VARIABLES
 ************************************/

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/
#define LCD12864_RST_PIN LL_GPIO_PIN_4
#define LCD12864_RS_PIN  LL_GPIO_PIN_5
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
static void lcd12864_init_gpio(void);
static void lcd12864_send_byte(uint8_t data);
static void lcd12864_send_command(uint8_t cmd);
static void lcd12864_send_data(uint8_t data);
static void lcd12864_send_string(int row, int col, char *string);

/************************************
 * STATIC FUNCTIONS
 ************************************/
static void lcd12864_init_gpio(void)
{
   LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

   LL_GPIO_SetPinMode(GPIOB, LCD12864_RST_PIN, LL_GPIO_MODE_OUTPUT);
   LL_GPIO_SetOutputPin(GPIOB, LCD12864_RST_PIN);
   LL_GPIO_SetPinMode(GPIOB, LCD12864_RS_PIN, LL_GPIO_MODE_OUTPUT);
   LL_GPIO_ResetOutputPin(GPIOB, LCD12864_RS_PIN);
}

static void lcd12864_send_byte(uint8_t data)
{
   uint8_t x = data;
   SPI_Transfer(SPI2, 0, &x, NULL, 1);
}

static void lcd12864_send_command(uint8_t cmd)
{
   LL_GPIO_SetOutputPin(GPIOB, LCD12864_RS_PIN); // RS = 0 (komenda)
   lcd12864_send_byte(0xF8);
   lcd12864_send_byte((cmd & 0xF0));      // Wysokie 4 bity
   lcd12864_send_byte((cmd & 0x0F) << 4); // Niskie 4 bity
   TS_Delay_ms(1);
   LL_GPIO_ResetOutputPin(GPIOB, LCD12864_RS_PIN); // RS = 0 (komenda)
}

static void lcd12864_send_data(uint8_t data)
{
   LL_GPIO_SetOutputPin(GPIOB, LCD12864_RS_PIN); // RS = 1 (dane)
   lcd12864_send_byte(0xFA);
   lcd12864_send_byte(data & 0xF0);        // Wysokie 4 bity
   lcd12864_send_byte((data & 0x0F) << 4); // Niskie 4 bity
   TS_Delay_ms(1);
   LL_GPIO_ResetOutputPin(GPIOB, LCD12864_RS_PIN); // RS = 0 (komenda)
}

static void lcd12864_send_string(int row, int col, char *string)
{
   switch (row)
   {
   case 0:
      col |= 0x80;
      break;
   case 1:
      col |= 0x90;
      break;
   case 2:
      col |= 0x88;
      break;
   case 3:
      col |= 0x98;
      break;
   default:
      col |= 0x80;
      break;
   }

   lcd12864_send_command(col);

   while (*string)
   {
      lcd12864_send_data(*string++);
   }
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
void LCD12864_Init(void)
{
   lcd12864_init_gpio();
   TS_Delay_ms(10);

   LL_GPIO_ResetOutputPin(GPIOB, LCD12864_RST_PIN);
   TS_Delay_ms(10);
   LL_GPIO_SetOutputPin(GPIOB, LCD12864_RST_PIN);
   TS_Delay_ms(50);

   lcd12864_send_command(0x30); // 8bit mode
   TS_Delay_ms(1);              //  >100us delay

   lcd12864_send_command(0x30); // 8bit mode
   TS_Delay_ms(1);              // >37us delay

   lcd12864_send_command(0x08); // D=0, C=0, B=0
   TS_Delay_ms(1);              // >100us delay

   lcd12864_send_command(0x01); // clear screen
   TS_Delay_ms(12);             // >10 ms delay


   lcd12864_send_command(0x06); // cursor increment right no shift
   TS_Delay_ms(1);              // 1ms delay

   lcd12864_send_command(0x0F); // D=1, C=0, B=0
   TS_Delay_ms(1);              // 1ms delay

   lcd12864_send_command(0x02); // return to home
   TS_Delay_ms(1);              // 1ms delay

   lcd12864_send_data('A');

   // lcd12864_send_string(1, 10, "Test");
}

void LCD12864_Set_Char(void)
{
   lcd12864_send_data('A');
}