/*
 * spi.c
 *
 *  Created on: Dec 10, 2022
 *      Author: mkpk
 */

#include <spi.h>

#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_spi.h>

#define SPI1_NSS_Pin    LL_GPIO_PIN_4
#define SPI1_SCK_Pin    LL_GPIO_PIN_5
#define SPI1_MISO_Pin   LL_GPIO_PIN_6
#define SPI1_MOSI_Pin   LL_GPIO_PIN_7

void SPI_Init(void)
{
   LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

   LL_GPIO_SetPinMode (GPIOA, SPI1_NSS_Pin, LL_GPIO_MODE_OUTPUT);
   LL_GPIO_SetPinSpeed(GPIOA, SPI1_NSS_Pin, LL_GPIO_SPEED_FREQ_HIGH);
   LL_GPIO_SetPinOutputType(GPIOA, SPI1_NSS_Pin, LL_GPIO_OUTPUT_PUSHPULL);
   LL_GPIO_SetPinPull (GPIOA, SPI1_NSS_Pin, LL_GPIO_PULL_DOWN);

   LL_GPIO_SetPinMode (GPIOA, SPI1_SCK_Pin, LL_GPIO_MODE_ALTERNATE);
   LL_GPIO_SetPinSpeed(GPIOA, SPI1_SCK_Pin, LL_GPIO_SPEED_FREQ_HIGH);
   LL_GPIO_SetPinPull (GPIOA, SPI1_SCK_Pin, LL_GPIO_PULL_DOWN);

   LL_GPIO_SetPinMode (GPIOA, SPI1_MISO_Pin, LL_GPIO_MODE_ALTERNATE);
   LL_GPIO_SetPinSpeed(GPIOA, SPI1_MISO_Pin, LL_GPIO_SPEED_FREQ_HIGH);
   LL_GPIO_SetPinPull (GPIOA, SPI1_MISO_Pin, LL_GPIO_PULL_DOWN);

   LL_GPIO_SetPinMode (GPIOA, SPI1_MOSI_Pin, LL_GPIO_MODE_ALTERNATE);
   LL_GPIO_SetPinSpeed(GPIOA, SPI1_MOSI_Pin, LL_GPIO_SPEED_FREQ_HIGH);
   LL_GPIO_SetPinPull (GPIOA, SPI1_MOSI_Pin, LL_GPIO_PULL_DOWN);

   LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

   //NVIC_SetPriority(SPI1_IRQn, 0);    /* Set priority for SPI1_IRQn */
   //NVIC_EnableIRQ(SPI1_IRQn);         /* Enable SPI1_IRQn           */

   /* Configure SPI1 communication */
   LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV64/*LL_SPI_BAUDRATEPRESCALER_DIV256*/);
   LL_SPI_SetTransferDirection(SPI1, LL_SPI_FULL_DUPLEX);
   LL_SPI_SetClockPhase       (SPI1, LL_SPI_PHASE_1EDGE);
   LL_SPI_SetClockPolarity    (SPI1, LL_SPI_POLARITY_LOW);
   /* Reset value is LL_SPI_MSB_FIRST */
   //  LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
   LL_SPI_SetDataWidth        (SPI1, LL_SPI_DATAWIDTH_8BIT);
   LL_SPI_SetNSSMode          (SPI1, LL_SPI_NSS_SOFT);
   LL_SPI_SetMode             (SPI1, LL_SPI_MODE_MASTER);

   //LL_SPI_EnableIT_RXNE(SPI1);        /* Enable RXNE  Interrupt      */
   //LL_SPI_EnableIT_TXE(SPI1);         /* Enable TXE   Interrupt      */
   //LL_SPI_EnableIT_ERR(SPI1);         /* Enable Error Interrupt      */

   LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
   LL_SPI_Enable(SPI1);
}

void SPI_Tx8(uint8_t *data, uint8_t n)
{
   uint8_t i = 0;

   LL_GPIO_ResetOutputPin(GPIOA, SPI1_NSS_Pin);
   while(i < n)
   {
      LL_SPI_TransmitData8(SPI1, *(data + i));
      while(!LL_SPI_IsActiveFlag_TXE(SPI1));
      i++;
   }
   LL_GPIO_SetOutputPin(GPIOA, SPI1_NSS_Pin);
}

void SPI_Rx8(uint8_t *data, uint8_t n)
{
   uint8_t i = 0;

   LL_GPIO_ResetOutputPin(GPIOA, SPI1_NSS_Pin);
   while(i < n)
   {
      *(data + i) = LL_SPI_ReceiveData8(SPI1);
      while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
      i++;
   }
   LL_GPIO_SetOutputPin(GPIOA, SPI1_NSS_Pin);
}
