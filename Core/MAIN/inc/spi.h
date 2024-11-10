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

void SPI_Init(void);
void SPI_Tx8(uint8_t *data, uint8_t n);
void SPI_Rx8(uint8_t *data, uint8_t n);

#ifdef __cplusplus
}
#endif
#endif /* __SPI_H__ */
