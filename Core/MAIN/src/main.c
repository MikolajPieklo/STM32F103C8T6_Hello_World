#include "main.h"

#include <stdio.h>
#include <stdlib.h>

#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_spi.h>

#include <WS25Qxx.h>
#include <cc1101.h>
#include <circual_buffer.h>
#include <delay.h>
#include <device_info.h>
#include <gpio.h>
#include <i2c.h>
#include <lora_e32.h>
#include <nrf.h>
#include <pwm.h>
#include <rtc.h>
#include <sh1106.h>
#include <si4432.h>
#include <spi.h>
#include <uart.h>

#if !defined(CC1101_TX) && !defined(CC1101_RX) && !defined(SI4432_TX) && !defined(SI4432_RX) \
    && !defined(NRF24_TX) && !defined(NRF24_RX) && !defined(LORA_E32_TX) && !defined(LORA_E32_RX)
#error Please define hardware variant!
#endif

uint8_t address[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

volatile uint32_t  irq_aux_nr = 0;
volatile uint32_t  irq_uart_nr = 0;
uint8_t            irq_buff[10];
static uint32_t    old_ts_ms = 0;
volatile CirBuff_T cb_uart1_tx = {.tail = 0,
                                  .head = 0,
                                  .size = CIRCUAL_BUFFER_SIZE,
                                  .USARTx = USART1};

volatile CirBuff_T cb_uart1_rx = {.tail = 0,
                                  .head = 0,
                                  .size = CIRCUAL_BUFFER_SIZE,
                                  .USARTx = USART1};

void SystemClock_Config(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
   /* Reset of all peripherals, Initializes the Flash interface and the Systick.
    */
   LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
   LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

   /* System interrupt init*/
   NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

   /* SysTick_IRQn interrupt configuration */
   NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));

   /* NOJTAG: JTAG-DP Disabled and SW-DP Enabled */
   LL_GPIO_AF_Remap_SWJ_NOJTAG();

   /* Configure the system clock */
   SystemClock_Config();
   SysTick_Config(SystemCoreClock / 1000);
   LL_SYSTICK_EnableIT();

   /* Initialize all configured peripherals */
   MX_GPIO_Init();
   // PWM_Init();
   UART1_Init();
   SPI_Init();
   RTC_Init();
   Device_Info();
   WS25Qxx_Init();
   if (I2C_DRV_STATUS_SUCCESS == I2C_Init(I2C2))
   {
      printf("I2C OK\n");
   }
   else
   {
      printf("I2C NOK\n");
   }
   SH1106_Init();
   SH1106_Send_Text(0, 0, "CPU LOAD: 12%");
   SH1106_Send_Text(0, 14, "MEMORY USAGE: 6%");
   SH1106_Send_Text(0, 28, "IRQ_Aux:");
   SH1106_Send_Text(0, 40, "IRQ_UART:");
   SH1106_Send_Text(0, 56, "LORA_STA:");

#if defined(CC1101_TX)
   printf("CC1101 Tx\n");
   CC1101_Init(CC1101_TX_ADDRESS);
#endif

#if defined(CC1101_RX)
   printf("CC1101 Rx\n");
   CC1101_Init(CC1101_RX_ADDRESS);
#endif

#if defined(SI4432_TX) || defined(SI4432_RX)
   printf("SI4432\n");
   SI4432_Init();
   SI4432_RxMode();
#endif

#if defined(NRF24_TX) || defined(NRF24_RX)
   printf("nRF24\n");
   nRF24_Init();
   NRF24_TxMode(address, 101);
   NRF24_RxMode(address, 101);
   uint8_t data[32];
#endif

   while (1)
   {

#ifdef CC1101_TX
      CC1101_Check_State();
      CC1101_Tx_Debug();
#endif

#ifdef CC1101_RX
      CC1101_Check_State();
      CC1101_Rx_Debug();
#endif

#ifdef SI4432_TX
      SI4432_Tx_Debug();
      TS_Delay_ms(500);
#endif

#ifdef SI4432_RX
      SI4432_Rx_Debug();
#endif

#ifdef NRF24_TX
      NRF_Debug();
      if (nRF24_Tx_Debug() == 1)
      {
         LL_GPIO_TogglePin(LED_Port, LED_Pin);
      }
      TS_Delay_ms(100);
#endif

#ifdef NRF24_RX
      if (1 == nRF24_isDataAvailable(0))
      {
         nRF24_Rx_Debug(data);
         LL_GPIO_TogglePin(LED_Port, LED_Pin);
      }
#endif

      // PWM_Update();

#if defined(LORA_E32_RX) || defined(LORA_E32_TX)
      Lora_Main_Thread();
#endif
      if (TS_Get_ms() >= old_ts_ms + 500)
      {
         old_ts_ms = TS_Get_ms();
         SH1106_Send_Text(60, 28, itoa(irq_aux_nr, (char *) irq_buff, 10));
         SH1106_Send_Text(60, 40, itoa(irq_uart_nr, (char *) irq_buff, 10));
#if defined(LORA_E32_RX) || defined(LORA_E32_TX)
         SH1106_Send_Text(60, 56, itoa(Lora_Get_Machine_State(), (char *) irq_buff, 10));
#ifdef LORA_E32_RX
         SH1106_Send_Text(100, 28, "RX");
         SH1106_Send_Text(100, 56, itoa(Lora_Get_Rx_Counter(), (char *) irq_buff, 10));
#else
         SH1106_Send_Text(100, 28, "TX");
         SH1106_Send_Text(100, 56, itoa(Lora_Get_Tx_Counter(), (char *) irq_buff, 10));
#endif
#endif
         LL_GPIO_TogglePin(LED_Port, LED_Pin);
      }

      // Simple CMD
      if (cb_uart1_rx.head != cb_uart1_rx.tail)
      {
         if (cb_uart1_rx.data[cb_uart1_rx.tail] == 0x00)
         {
            WS25Qxx_Erase_Chip();
         }
         cb_uart1_rx.tail++;
      }
   }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
   LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
   while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
   {
   }
   LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
   while (LL_RCC_HSE_IsReady() != 1)
   {
   }
   LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
   LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
   while (LL_RCC_PLL_IsReady() != 1)
   {
   }
   LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
   LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
   LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
   LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
   while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
   {
   }
   LL_Init1msTick(72000000);
   LL_SetSystemCoreClock(72000000);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
   /* USER CODE BEGIN Error_Handler_Debug */
   /* User can add his own implementation to report the HAL error return state */
   __disable_irq();
   while (1)
   {
   }
   /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
   // printf("Wrong parameters value: file %s on line %ld\r\n", file, line);
}
#endif /* USE_FULL_ASSERT */
