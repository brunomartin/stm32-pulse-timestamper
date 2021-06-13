/**
  ******************************************************************************
  * @file    UART/UART_HyperTerminal_IT/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>

#include "stm32l4xx_hal.h"
#include "stm32l4xx_nucleo.h"

#include "socket.h"
#include "dhcp.h"
#include "dns.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
/* Definition for USARTx clock resources */
#define USARTx                           UART4
#define USARTx_CLK_ENABLE()              __HAL_RCC_UART4_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA2_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_UART4_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_UART4_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_0
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF8_UART4
#define USARTx_RX_PIN                    GPIO_PIN_1
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF8_UART4

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL             DMA2_Channel3
#define USARTx_RX_DMA_CHANNEL             DMA2_Channel5

/* Definition for USARTx's DMA Request */
#define USARTx_TX_DMA_REQUEST             DMA_REQUEST_2
#define USARTx_RX_DMA_REQUEST             DMA_REQUEST_2

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA2_Channel3_IRQn
#define USARTx_DMA_RX_IRQn                DMA2_Channel5_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA2_Channel3_IRQHandler
#define USARTx_DMA_RX_IRQHandler          DMA2_Channel5_IRQHandler

/* Definition for USARTx's NVIC IRQ and IRQ Handlers */
#define USARTx_IRQn                      UART4_IRQn
#define USARTx_IRQHandler                UART4_IRQHandler

/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

/* User can use this section to tailor SPIx instance used and associated
   resources */
/* Definition for SPIx clock resources */
#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_CLK_DISABLE()               __HAL_RCC_SPI1_CLK_DISABLE()
#define SPIx_DMAx_CLK_ENABLE()           __HAL_RCC_DMA1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_5
#define SPIx_SCK_GPIO_PORT               GPIOA
#define SPIx_SCK_AF                      GPIO_AF5_SPI1
#define SPIx_MISO_PIN                    GPIO_PIN_6
#define SPIx_MISO_GPIO_PORT              GPIOA
#define SPIx_MISO_AF                     GPIO_AF5_SPI1
#define SPIx_MOSI_PIN                    GPIO_PIN_7
#define SPIx_MOSI_GPIO_PORT              GPIOA
#define SPIx_MOSI_AF                     GPIO_AF5_SPI1

/* Definition for SPIx's DMA */
#define SPIx_TX_DMA_CHANNEL              DMA1_Channel3
#define SPIx_RX_DMA_CHANNEL              DMA1_Channel2

#define SPIx_TX_DMA_REQUEST              DMA_REQUEST_1
#define SPIx_RX_DMA_REQUEST              DMA_REQUEST_1

/* Definition for SPIx's NVIC */
#define SPIx_DMA_TX_IRQn                 DMA1_Channel3_IRQn
#define SPIx_DMA_RX_IRQn                 DMA1_Channel2_IRQn

#define SPIx_DMA_TX_IRQHandler           DMA1_Channel3_IRQHandler
#define SPIx_DMA_RX_IRQHandler           DMA1_Channel2_IRQHandler

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */
/* IRQ Handler treatment functions */

/* Definition for TIMx clock resources */
#define TIMx                           TIM2
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM2_CLK_ENABLE()
#define TIMx_CLK_DISABLE()             __HAL_RCC_TIM2_CLK_DISABLE()

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM2_IRQn
#define TIMx_IRQHandler                TIM2_IRQHandler

/* Definition for exterior interrupts */

#define EXTIx_IRQHandler               EXTI15_10_IRQHandler
#define EXTIx_IRQn                     EXTI15_10_IRQn
#define EXTIx_CLK_ENABLE()              __HAL_RCC_GPIOB_CLK_ENABLE()
#define EXTIx_PIN                      GPIO_PIN_10
#define EXTIx_GPIO_PORT                GPIOB

/* Definition for software interrupts */

#define SWIx_IRQHandler                EXTI9_5_IRQHandler
#define SWIx_IRQn                      EXTI9_5_IRQn
#define SWIx_CLK_ENABLE()               __HAL_RCC_GPIOC_CLK_ENABLE()
#define SWIx_PIN                       GPIO_PIN_9
#define SWIx_GPIO_PORT                 GPIOC

/* w5500 stuff */

#define W5500_CS_Pin                   GPIO_PIN_6
#define W5500_CS_GPIO_Port             GPIOB
#define W5500_CS_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()
#define W5500_CS_CLK_DISABLE()         __HAL_RCC_GPIOB_CLK_DISABLE()

#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define HTTP_SOCKET     2
#define UDP_SOCKET      3

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
