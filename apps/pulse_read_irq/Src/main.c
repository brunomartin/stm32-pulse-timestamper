/**
  ******************************************************************************
  * @file    UART/UART_HyperTerminal_IT/Src/main.c
  * @author  MCD Application Team
  * @brief   This sample code shows how to use UART HAL and LL APIs to transmit
  *          and receive a data buffer with a communication process based on IT;
  *          The communication is done with the Hyperterminal PC application;
  *          HAL driver is used to perform UART configuration, 
  *          then TX/RX transfers procedures are based on LL APIs use
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32L4xx_HAL_LL_MIX_Examples
  * @{
  */

/** @addtogroup UART_Hyperterminal_IT
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Timer declaration */
TIM_HandleTypeDef htim = {0};
uint32_t uwPrescalerValue;

/* UART handler declaration */
UART_HandleTypeDef UartHandle;
__IO uint8_t  ubTxComplete = 0;
__IO uint8_t  ubRxComplete = 0;

/* Buffer used for transmission */
uint8_t aTxStartMessage[] = "\n\r ****UART-Hyperterminal communication based on IT (Mixed HAL/LL usage) ****\n\r Enter 10 characters using keyboard :\n\r";
__IO uint32_t uwTxIndex = 0;
uint8_t ubSizeToSend = sizeof(aTxStartMessage);
uint8_t aTxEndMessage[] = "\n\r Example Finished\n\r";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];
__IO uint32_t uwRxIndex = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Timer_Config(void);
static void Error_Handler(void);

static void EXTI9_5_IRQHandler_Config(void);

__IO uint32_t  detected_pulses = 0;

// Set timestamps vector size, be aware of the total RAM size (96kB on this one)
const uint32_t timestamps_size = 20000;
uint32_t* timestamps;

struct Statistics {
  uint32_t count;
  double total;
  double mean;
  double std_dev;
  uint32_t min;
  uint32_t max;
};

static void ComputeStats(const uint32_t* timestamps,
  uint32_t size, struct Statistics* stats);

static void HAL_Delay_us(uint32_t ticks);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 80 MHz */
  SystemClock_Config();

  /* Configure timer */
  Timer_Config();

  /* Configure External line 13 (connected to PC.13 pin) in interrupt mode */
  EXTI9_5_IRQHandler_Config();
  
  /* Configure leds */
  BSP_LED_Init(LED2);

  /*##-1- Configure the UART peripheral using HAL services ###################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits (7 data bit + 1 parity bit) : 
	                  BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
      - Stop Bit    = One Stop bit
      - Parity      = ODD parity
      - BaudRate    = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals)

    To test it on a rpi4:
    > stty 9600 -F /dev/ttyAMA0 parenb parodd cs7 -cstopb -crtscts
    > cat /dev/ttyAMA0
    > echo 1234567890 > /dev/ttyAMA0
    > screen /dev/ttyAMA0 9600,cs7,parenb,parodd,-cstopb,-crtscts
      
  */
  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_ODD;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure UART peripheral for reception process (using LL) ##########*/  
  /* Any data received will be stored "aRxBuffer" buffer : the number max of 
     data received is RXBUFFERSIZE */
  /* Enable RXNE and Error interrupts */  
  LL_USART_EnableIT_RXNE(USARTx);
  LL_USART_EnableIT_ERROR(USARTx);

  timestamps = (uint32_t*) malloc(timestamps_size*sizeof(uint32_t));

  uint32_t pulses_to_detect = 1e6;
  pulses_to_detect = 1e5;
  int pulses_step_size = timestamps_size/2;

  struct Statistics stats;
  int pulses_last_step = 0;
  int pulses_next_step = pulses_step_size;
  uint32_t* step_timestamps = timestamps;

  int send_id = 0;
  uint32_t total_stats_count = 0;

  /* Wait all pulses have been detected */
  while (detected_pulses < pulses_to_detect) {

    // Wait for a part of pulse to be detected and compute stats
    while (detected_pulses < pulses_next_step) {}

    step_timestamps = timestamps;
    step_timestamps += pulses_last_step % timestamps_size;

    // Compute statistics
    ComputeStats(step_timestamps, pulses_step_size, &stats);

    total_stats_count += stats.count;

    pulses_last_step = pulses_next_step;
    pulses_next_step += pulses_step_size;

    // Initialize message to send
    memset(aTxStartMessage, 0, ubSizeToSend);
    
    // Transmit results on serial link
    sprintf(aTxStartMessage, "%d: total: %0.2fs, count: %lu mean: %0.2fus, std: %0.2fus, min: %luus, max: %luus, rate: %0.2fkHz\n\r", send_id, stats.total/1e6, stats.count, stats.mean, stats.std_dev, stats.min, stats.max, 1/stats.mean*1e3);

    send_id++;

    /*##-3- Start the transmission process (using LL) *##########################*/  
    /* While the UART in reception process, user can transmit data from 
      "aTxStartMessage" buffer */
    /* Start USART transmission : Will initiate TXE interrupt after TDR register is empty */
    LL_USART_TransmitData8(USARTx, aTxStartMessage[uwTxIndex++]); 

    /* Enable TXE interrupt */
    LL_USART_EnableIT_TXE(USARTx); 

    /*##-4- Wait for the end of the transfer ###################################*/
    /* USART IRQ handler is not anymore routed to HAL_UART_IRQHandler() function 
      and is now based on LL API functions use. 
      Therefore, use of HAL IT based services is no more possible. */
    /*  Once TX transfer is completed, LED2 will toggle.
        Then, when RX transfer is completed, LED2 will turn On. */
    while (ubTxComplete == 0)
    {
    }

    ubTxComplete = 0;

  }

  // Initialize message to send
  memset(aTxStartMessage, 0, ubSizeToSend);
  
  // Transmit results on serial link
  sprintf(aTxStartMessage, "total_stats_count: %lu\n\r", total_stats_count);

  /*##-3- Start the transmission process (using LL) *##########################*/  
  /* While the UART in reception process, user can transmit data from 
    "aTxStartMessage" buffer */
  /* Start USART transmission : Will initiate TXE interrupt after TDR register is empty */
  LL_USART_TransmitData8(USARTx, aTxStartMessage[uwTxIndex++]); 

  /* Enable TXE interrupt */
  LL_USART_EnableIT_TXE(USARTx); 

  /*##-4- Wait for the end of the transfer ###################################*/
  /* USART IRQ handler is not anymore routed to HAL_UART_IRQHandler() function 
    and is now based on LL API functions use. 
    Therefore, use of HAL IT based services is no more possible. */
  /*  Once TX transfer is completed, LED2 will toggle.
      Then, when RX transfer is completed, LED2 will turn On. */
  while (ubTxComplete == 0)
  {
  }

  while (ubRxComplete == 0)
  {
    BSP_LED_Toggle(LED2);
    HAL_Delay(250);
  }

  BSP_LED_On(LED2); /* stop blink and keeps ON */

  /*##-5- Send the received Buffer ###########################################*/
  /* Even if use of HAL IT based services is no more possible, use of HAL Polling based services
     (as Transmit in polling mode) is still possible. */
  if(HAL_UART_Transmit(&UartHandle, (uint8_t*)aRxBuffer, RXBUFFERSIZE, 1000)!= HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();
  }
  
  /*##-6- Send the End Message ###############################################*/  
  if(HAL_UART_Transmit(&UartHandle, (uint8_t*)aTxEndMessage, TXENDMESSAGESIZE, 1000)!= HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();
  }
  
  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

static void Timer_Config(void)
{

  /* Compute the prescaler value to have TIMx counter clock equal to 1MHz */

    htim.Instance               = TIMx;
    htim.Init.Prescaler         = (uint32_t)(SystemCoreClock / 1e6) - 1;
    htim.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim.Init.Period            = 0xFFFFFFFF;
    // htim.Init.Period            = 100000;
    htim.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim.Init.RepetitionCounter = 0x0;

    /* Auto-reload register preload is disabled */
    htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    /* Configure timer time base */
  if (HAL_TIM_Base_Init(&htim) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&htim) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Do something
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 to off for error */
  BSP_LED_Off(LED2); 
  while(1)
  {
  }
}

/**
  * @brief  This function computes statistics from a vector of timestamps.
  * @param  timestamps : vector of timestamps in uint32_t (in)
  * @param  size : size of timestamp vector in uint32_t (in)
  * @param  stats : computed statistics as a Statistics structure (out)
  * @retval None
  */
static void ComputeStats(const uint32_t* timestamps, uint32_t size, struct Statistics* stats) {
  stats->count = size;

  stats->total = timestamps[size-1] - timestamps[0];
  
  stats->mean = stats->total/(size-1);

  stats->std_dev = 0;
  double elapsed;
  for(int i=1;i<size;i++) {
    elapsed = timestamps[i] - timestamps[i-1];
    stats->std_dev += (elapsed-stats->mean)*(elapsed-stats->mean);
  }
  stats->std_dev = sqrt(stats->std_dev/(size-1));

  stats->min = -1, stats->max = 0;
  for(int i=1;i<size;i++) {
    // Get min and max, does not take too many cycles
    elapsed = timestamps[i] - timestamps[i-1];
    stats->max = elapsed > stats->max ? elapsed : stats->max;
    stats->min = elapsed < stats->min ? elapsed : stats->min;
  }
}

/**
  * @brief  This function delays accurately.
  * @param  ticks : number of system ticks to wait
  * @retval None
  */
static void HAL_Delay_us(uint32_t ticks)
{
    SysTick->LOAD = ticks*10;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
    // SysTick->CTRL &= ~SYSTICK_CLKSOURCE_HCLK_DIV8;
    // SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;

    // COUNTFLAG is a bit that is set to 1 when counter reaches 0.
    // It's automatically cleared when read.
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    SysTick->CTRL = 0;
}

/**
  * @brief  Rx Transfer completed callback
  * @note   This example shows a simple way to report end of IT Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void UART_CharReception_Callback(void)
{
  /* Read Received character. RXNE flag is cleared by reading of RDR register */
  aRxBuffer[uwRxIndex++] = LL_USART_ReceiveData8(USARTx);

  /* Check if reception is completed (expected nb of bytes has been received) */
  if (uwRxIndex == RXBUFFERSIZE)
  {
    /* Set Reception complete boolean to 1 */
    ubRxComplete = 1;
  }
}

/**
  * @brief  Function called for achieving next TX Byte sending
  * @retval None
  */
void UART_TXEmpty_Callback(void)
{
  if(uwTxIndex == (ubSizeToSend - 1))
  {
    /* Disable TXE interrupt */
    LL_USART_DisableIT_TXE(USARTx);
    
    /* Enable TC interrupt */
    LL_USART_EnableIT_TC(USARTx);
  }

  /* Fill TDR with a new char */
  LL_USART_TransmitData8(USARTx, aTxStartMessage[uwTxIndex++]);
}

/**
  * @brief  Function called at completion of last byte transmission
  * @retval None
  */
void UART_CharTransmitComplete_Callback(void)
{
  if(uwTxIndex == sizeof(aTxStartMessage))
  {
    uwTxIndex = 0;
    
    /* Disable TC interrupt */
    LL_USART_DisableIT_TC(USARTx);
    
    /* Set Tx complete boolean to 1 */
    ubTxComplete = 1;
  }
}

/**
  * @brief  UART error callbacks
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void UART_Error_Callback(void)
{
  __IO uint32_t isr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USARTx_IRQn);
  
  /* Error handling example :
    - Read USART ISR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  isr_reg = LL_USART_ReadReg(USARTx, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    /* Turn LED2 off: Transfer error in reception/transmission process */
    BSP_LED_Off(LED2);
  }
  else
  {
    /* Turn LED2 off: Transfer error in reception/transmission process */
    BSP_LED_Off(LED2);
  }
}


/**
  * @brief  Configures EXTI lines 10 to 15 (connected to PC.13 pin) in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTI9_5_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable and set EXTI lines 10 to 15 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_8) {

    // If first pulses detected, start timer
    if(detected_pulses == 0) {
      HAL_TIM_Base_Start(&htim);
    }

    // And store counter value
    timestamps[detected_pulses%timestamps_size] =
      __HAL_TIM_GET_COUNTER(&htim);

    /* Increment detected pulses */
    detected_pulses++;
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
