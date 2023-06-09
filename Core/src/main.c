#include <stdio.h>

#include <main.h>

#include <cc1101.h>
#include <delay.h>
#include <gpio.h>
#include <nrf.h>
#include <pwm.h>
#include <si4432.h>
#include <spi.h>
#include <uart.h>
#include <rtc.h>

#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_spi.h>

#define CC1101_TX

#if !defined(CC1101_TX) && !defined(CC1101_RX) && !defined(SI4432_TX) && !defined(SI4432_RX) && !defined(NRF24_TX) && !defined(NRF24_RX)
   #error Please define hardware variant!
#endif

uint8_t address[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

void SystemClock_Config(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
   /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
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
   //PWM_Init();
   UART_Init();
   SPI_Init();
   RTC_Init();

#if defined(CC1101_TX)
   printf ("CC1101 Tx\n");
   CC1101_Init(0x01);
#endif

#if defined(CC1101_RX)
   printf ("CC1101\n");
   CC1101_Init(0x03);
#endif

#ifdef defined(SI4432_TX) || defined(SI4432_RX)
   printf ("SI4432\n");
   SI4432_Init();
   SI4432_RxMode();
#endif

#ifdef defined(NRF24_TX) || defined(NRF24_RX)
   printf ("nRF24\n");
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


//      LL_GPIO_ResetOutputPin(LED_Port, LED_Pin);
//      TS_Delay_ms(500);
//      LL_GPIO_SetOutputPin(LED_Port, LED_Pin);
//      TS_Delay_ms(500);

      //PWM_Update();
      LL_GPIO_TogglePin(LED_Port, LED_Pin);
      TS_Delay_ms(500);
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

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
   //printf("Wrong parameters value: file %s on line %ld\r\n", file, line);
}
#endif /* USE_FULL_ASSERT */
