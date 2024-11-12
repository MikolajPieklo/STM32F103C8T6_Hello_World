/*
 * spi.h
 *
 *  Created on: Dec 10, 2022
 *      Author: mkpk
 */

#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_spi.h>

#define SPI1_CS1_Pin  LL_GPIO_PIN_4
#define SPI1_CS2_Pin  LL_GPIO_PIN_1
#define SPI1_SCK_Pin  LL_GPIO_PIN_5
#define SPI1_MISO_Pin LL_GPIO_PIN_6
#define SPI1_MOSI_Pin LL_GPIO_PIN_7

void    SPI_Init(void);
uint8_t SPI_Transfer(SPI_TypeDef *dev, uint32_t cs_pin, uint8_t *tx_data, uint8_t *rx_data,
                     uint8_t n);

#ifdef __cplusplus
}
#endif
#endif /* __SPI_H__ */
