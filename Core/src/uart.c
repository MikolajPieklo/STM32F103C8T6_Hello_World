/*
 * uart.c
 *
 *  Created on: Dec 18, 2022
 *      Author: mkpk
 */

#include <stdint.h>
#include <uart.h>

#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_usart.h>

void UART1_Init(void)
{
   //   /* USART1 instance is used. (requires wiring USART1 TX/Rx Pins to USB to UART adapter) */
   //   LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA); /* Enable the peripheral clock of GPIO
   //   Port */ LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);/* Enable USART peripheral
   //   clock */
   //
   //   /* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
   //   LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
   //   LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
   //   LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
   //   LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);
   //
   //   /* Configure Rx Pin as : Input Floating function, High Speed, Pull up */
   //   LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
   //   LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
   //   LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_PUSHPULL);
   //   LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);
   //
   //   /* (2) NVIC Configuration for USART interrupts */
   //   /*  - Set priority for USARTx_IRQn */
   //   /*  - Enable USARTx_IRQn */
   //   //   NVIC_SetPriority(USARTx_IRQn, 0);
   //   //   NVIC_EnableIRQ(USARTx_IRQn);
   //
   //   /* Disable USART prior modifying configuration registers */
   //   /* Note: Commented as corresponding to Reset value */
   //   LL_USART_Disable(USART1);
   //
   //   /* TX/RX direction */
   //   LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
   //
   //   /* 8 data bit, 1 start bit, 1 stop bit, no parity */
   //   LL_USART_ConfigCharacter(USART1, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE,
   //   LL_USART_STOPBITS_1);
   //
   //   /* Set Baudrate to 115200 using APB frequency set to 72000000/APB_Div Hz */
   //   /* Frequency available for USART peripheral can also be calculated through LL RCC macro */
   //   /* Ex :
   //       Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance);
   //       depending on USART/UART instance
   //
   //       In this example, Peripheral Clock is expected to be equal to 72000000/APB_Div Hz =>
   //       equal to SystemCoreClock/APB_Div
   //   */
   //   LL_USART_SetBaudRate(USART1, SystemCoreClock/1, 115200);
   //
   //   /* (4) Enable USART *********************************************************/
   //   LL_USART_Enable(USARTx_INSTANCE);
   //
   //   /* Enable RXNE and Error interrupts */
   ////   LL_USART_EnableIT_RXNE(USARTx_INSTANCE);
   ////   LL_USART_EnableIT_ERROR(USARTx_INSTANCE);
   //   /* USER CODE BEGIN USART1_Init 0 */
   //
   //   /* USER CODE END USART1_Init 0 */

   LL_USART_InitTypeDef USART_InitStruct = {0};
   LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

   /* Peripheral clock enable */
   LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

   LL_AHB1_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
   /**USART1 GPIO Configuration
   PA9   ------> USART1_TX
   PA10   ------> USART1_RX
   */
   GPIO_InitStruct.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
   GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
   GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
   GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
   GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
   // GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
   LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   /* USART1 interrupt Init */
   NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
   NVIC_EnableIRQ(USART1_IRQn);

   /* USER CODE BEGIN USART1_Init 1 */

   /* USER CODE END USART1_Init 1 */
   USART_InitStruct.BaudRate = 115200;
   USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
   USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
   USART_InitStruct.Parity = LL_USART_PARITY_NONE;
   USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
   USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
   USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
   LL_USART_Init(USART1, &USART_InitStruct);
   LL_USART_ConfigAsyncMode(USART1);
   LL_USART_Enable(USART1);
}

void UART1_Tx(uint8_t *data, uint8_t n)
{
}
void UART1_Rx(uint8_t *data, uint8_t n)
{
}

void USART1_IRQHandler(void)
{
}
