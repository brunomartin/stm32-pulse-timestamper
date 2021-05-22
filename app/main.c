
#include "stm32l4xx_hal.h"

RCC_OscInitTypeDef RCC_OscInitStruct;
RCC_ClkInitTypeDef RCC_ClkInitStruct;

void SystemClock_Config(void);
void Error_Handler(void);

void delay_ticks(uint32_t ticks)
{
    SysTick->LOAD = ticks*RCC_OscInitStruct.HSICalibrationValue;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
    // SysTick->CTRL &= ~SYSTICK_CLKSOURCE_HCLK_DIV8;
    // SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;

    // COUNTFLAG is a bit that is set to 1 when counter reaches 0.
    // It's automatically cleared when read.
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    SysTick->CTRL = 0;
}

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */

  const uint32_t pulses = 0;
  uint32_t count = 0;

  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Wait 10 ms to let system setup
  HAL_Delay(10);

  for(count=0;count<pulses;count++) {
    // BSP_LedToggle(GREEN);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    // HAL_Delay(1);
    delay_ticks(40);
  }

  return 0;
}

// System Clock Configuration
void SystemClock_Config(void)
{
  // Initializes the CPU, AHB and APB busses clocks
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  // RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  // RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSICalibrationValue = 1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  // Initializes the CPU, AHB and APB busses clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
          |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  // RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  // RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
  
  // Configure the main internal regulator output voltage
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }
  
  // Configure the Systick interrupt time
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  // HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq());
  // Configure the Systick
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  // SysTick_IRQn interrupt configuration
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}

#endif
