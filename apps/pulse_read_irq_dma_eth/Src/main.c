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
enum {
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
	TRANSFER_ERROR
};
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Timer declaration */
TIM_HandleTypeDef htim = {0};

// 80MHz: timer precision is 12.5ns, 4G*12.5ns: 50s
uint32_t uwPrescalerFreq = 80000000; 

// 1MHz: timer precision is 12.5 * 80 = 1000ns, 4G*1000ns: 4000s
// uint32_t uwPrescalerFreq =  1000000;

// 10MHz: timer precision is 12.5 * 8 = 100ns, 4G*100ns: 400s
// uint32_t uwPrescalerFreq = 10000000;

uint32_t uwPeriod = 4000000000 - 1; // timer will wrapped before 4G

/* UART handler declaration */
UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;

/* Buffer used for transmission */
uint8_t aTxBuffer[] = 
"****READY TO RECEIVE PULSES****\n\r"
"****READY TO RECEIVE PULSES****\n\r"
"****READY TO RECEIVE PULSES****\n\r"
"****READY TO RECEIVE PULSES****\n\r"
"****READY TO RECEIVE PULSES****\n\r"
"****READY TO RECEIVE PULSES****\n\r"
"****READY TO RECEIVE PULSES****\n\r"
;

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

void UART_Printf(const char* fmt, ...);
void UART_Printf_No_Block(const char* fmt, ...);

/* SPI stuff*/
SPI_HandleTypeDef hspi1;

/* transfer state */
__IO uint8_t wTxTransferState = TRANSFER_COMPLETE;
__IO uint8_t wRxTransferState = TRANSFER_COMPLETE;

__IO uint8_t waitTxTransferDone = 1;
__IO uint8_t waitRxTransferDone = 1;

/* w5500 stuff */

__IO uint8_t ip_assigned = 0;

// 1K should be enough, see https://forum.wiznet.io/t/topic/1612/2
uint8_t dhcp_buffer[1024];
// 1K seems to be enough for this buffer as well
uint8_t dns_buffer[1024];

// Detection variables
__IO uint32_t  pulses_detected[2] = {0, 0};

// Set timestamps vector size, be aware of the total RAM size (96kB on this one)
// w5500 has max 16kB of Tx buffer to give to max 8 sockets
// A second detection line is scheduled so 8kB of Tx buffer
// 16kB for the line is ok, this 4*1024 uint32
// 8kB for the line is ok, this 2*1024 uint32
// const uint32_t timestamps_size = 4*1024;
const uint32_t timestamps_size = 9*512;
uint32_t* timestamps[2] = {0, 0};

__IO uint32_t pulses_sent[2] = {0, 0};

uint8_t udp_socket = UDP_SOCKET;
int dest_ports[2] = {8042, 8043};

// buffering_timestamps tells that trabufferingnsfer is occuring
// avoiding sending same timestamps twice 
__IO uint8_t buffering_timestamps = 0;

__IO uint32_t packets_sent[2] = {0, 0};

__IO uint32_t last_packet_time_ms[2] = {-1, -1};

// address is the destination IP address where to
// send timestamps when { 255, 255, 255, 255 }, it waits
// for client connection
// uint8_t address[4] = { 255, 255, 255, 255 };
uint8_t address[4] = { 192, 168, 1, 12 };
// uint8_t address[4] = { 192, 168, 1, 40 };

EXTI_HandleTypeDef exti;

// Buffer of W5500 is 4kB, 4x1024B
// Each fragment shall contain header + data
// 1024B per fragment, 4 fragments per packet is ok for robustess
// Total shall not exceed 4KB unless end of data is lost

// UDP packet fragment size in bytes, MSS is set
// to this value, max is 1472 bytes
// header : packet id + fragment id : 4 + 2 bytes
// content : 1016 bytes: 250 uint32 timestamps values
#define TIMESTAMP_PER_FRAGMENT 250

// timestamp type size in bytes, according to chosen Timer
// It can be uin16 or uint32, TIM2 is uint32
#define TIMESTAMP_TYPE_SIZE sizeof(uint32_t)

// WS550 fragments udp packets and it seems that these packet
// cannot be re assembled by client. So we add an header to these
// fragments : packet id and fragment id
// !!! header size > 4 will leave 0xFF values in packet after a while
#define UDP_FRAGMENT_HEADER_SIZE 24
// #define UDP_FRAGMENT_HEADER_SIZE 0
#define UDP_FRAGMENT_DATA_SIZE (TIMESTAMP_PER_FRAGMENT * TIMESTAMP_TYPE_SIZE)
#define UDP_FRAGMENT_SIZE (UDP_FRAGMENT_HEADER_SIZE + UDP_FRAGMENT_DATA_SIZE)

// 4 fragment seems to be ok to be sent
// lower: too many UDP packet are sent
// higher: too much to wait before sending a packet
#define UDP_FRAGMENT_COUNT 4

// define useful constants
#define UDP_PACKET_DATA_SIZE (UDP_FRAGMENT_COUNT * UDP_FRAGMENT_DATA_SIZE)
#define UDP_PACKET_SIZE (UDP_FRAGMENT_COUNT * UDP_FRAGMENT_SIZE)
#define TIMESTAMP_PER_PACKET (UDP_FRAGMENT_COUNT * TIMESTAMP_PER_FRAGMENT)

uint8_t* udp_packet;

struct Statistics {
  uint32_t count;
  double total;
  double mean;
  double std_dev;
  uint32_t min;
  uint32_t max;
};

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Timer_Config(void);
static void Error_Handler(void);

static void EXTI_IRQHandler_Config(void);

static void UART_Config(void);

static void SPI_Config(void);

void W5500_Select(void);
void W5500_Unselect(void);
void W5500_ReadBuff(uint8_t* buff, uint16_t len);
void W5500_WriteBuff(uint8_t* buff, uint16_t len);
uint8_t W5500_ReadByte(void);
void W5500_WriteByte(uint8_t byte);
void Callback_IPAssigned(void); 
void Callback_IPConflict(void);
static void W5500_Config(void);

void udp_server_start(uint8_t udp_socket, int server_port, uint8_t* address);
void udp_server_stop(uint8_t udp_socket);

static void ComputeStats(const uint32_t* timestamps,
  uint32_t size, struct Statistics* stats);

static uint32_t GetTimerTimeUsFromCounter();
static uint32_t GetTimerTimeUs();
static uint32_t GetTimerTimeMs();

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

  /* Configure UART */
  UART_Config();

  UART_Printf("UART configured\n\r");

  /* Configure SPIx */
  SPI_Config();

  timestamps[0] = (uint32_t*) malloc(timestamps_size*TIMESTAMP_TYPE_SIZE);
  timestamps[1] = (uint32_t*) malloc(timestamps_size*TIMESTAMP_TYPE_SIZE);
  udp_packet = (uint8_t*) malloc(UDP_PACKET_SIZE);

  for(size_t i=0;i<timestamps_size;i++) {
    timestamps[0][i] = (uint32_t) i;
    timestamps[1][i] = (uint32_t) i;
  }

  // Init timestamp buffer with magic number
  memset(timestamps[0], 0xFF, timestamps_size*TIMESTAMP_TYPE_SIZE);
  memset(timestamps[1], 0xFF, timestamps_size*TIMESTAMP_TYPE_SIZE);

  // Mark udp_packet
  memset(udp_packet, 0xFF, UDP_PACKET_SIZE);
  
  /* Configure w5500 */
  W5500_Config();
  
  /* Configure leds */
  // BSP_LED_Init(LED2); // Conflict With SPI1 GPIOs

  /* Start timer now */
  HAL_TIM_Base_Start(&htim);

  int server_port = 8041;

  udp_server_start(udp_socket, server_port, address);

  UART_Printf("Client address:   %d.%d.%d.%d\r\n",
      address[0], address[1], address[2], address[3]
  );

  UART_Printf("Destination ports: %d, %d\r\n",
    dest_ports[0], dest_ports[1]);

  struct Statistics stats;

  uint32_t current_time_ms = GetTimerTimeMs();
  uint32_t last_print_time_ms = current_time_ms;

  UART_Printf("Filling RAM and UDP Tx buffer...\n\r");
  // If we do not do that, first timestamps will take too much
  // time and add a lot of jitter
  for(int i=0;i<2;i++) {
    int32_t nbytes = sendto(udp_socket, udp_packet, UDP_PACKET_SIZE, address, 65535);
  }

  /* Configure External line 13 (connected to PC.13 pin) in interrupt mode */
  EXTI_IRQHandler_Config();

  UART_Printf("****READY TO RECEIVE PULSES****\n\r");

  /* Wait all pulses have been detected */
  while (1) {

    // Following code look if there any timestamp detected to send
    for(uint8_t line=0;line<2;line++) {

      // Store volatile variables before testing them
      uint32_t current_pulses_detected = pulses_detected[line];
      uint32_t current_packets_sent = packets_sent[line];
      uint32_t current_pulses_sent = pulses_sent[line];

      current_time_ms = GetTimerTimeMs();
      if(current_time_ms < last_print_time_ms) {
        UART_Printf("Timer wrapped\r\n");
      }

      uint32_t last_detected_pulse_time_ms = -1;
      if(current_pulses_detected > 0) {
        uint32_t index = (current_pulses_detected-1)%timestamps_size;
        last_detected_pulse_time_ms = GetTimerTimeUsFromCounter(
          timestamps[0][index]) / 1000;
      }

      uint32_t pulses_to_sent = 0;

      // If there is still pulses to send after a duration (50ms) since
      // last pulse detected, force sending theme it by triggering
      // software interrupt
      int8_t force_send_packets = 1;
      force_send_packets &= current_pulses_detected > current_pulses_sent;
      force_send_packets &= last_detected_pulse_time_ms != -1;
      force_send_packets &= current_time_ms > last_detected_pulse_time_ms + 50;
      force_send_packets &= !buffering_timestamps;
      if(force_send_packets) {
        pulses_to_sent = current_pulses_detected - current_pulses_sent;
        UART_Printf("Force sending %d pulses\r\n", pulses_to_sent);
        // Tell that transfer is occuring unless it may reentre here
        buffering_timestamps = 1;

        exti.Line = EXTI_LINE_9;
        HAL_EXTI_GenerateSWI(&exti);
      }
    }

    uint32_t duration = (current_time_ms - last_print_time_ms);
    if(duration > 2000) {
      UART_Printf(
        "INFO:\r\n"
        "  current_time_ms: %d\r\n"
        ,
        current_time_ms
      );
      
      for(uint8_t line=0;line<2;line++) {

        // Store volatile variables before testing them
        uint32_t current_pulses_detected = pulses_detected[line];
        uint32_t current_packets_sent = packets_sent[line];
        uint32_t current_pulses_sent = pulses_sent[line];
        uint32_t pulses_to_sent = current_pulses_detected - current_pulses_sent;

        uint32_t last_detected_pulse_time_ms = -1;
        if(current_pulses_detected > 0) {
          uint32_t index = (current_pulses_detected-1)%timestamps_size;
          last_detected_pulse_time_ms = GetTimerTimeUsFromCounter(
            timestamps[line][index]) / 1000;
        }

        UART_Printf(
          "Line %d:\r\n"
          "  last_detected_pulse_time_ms: %d\r\n"
          "  last_packet_time_ms: %d\r\n"
          "  current_pulses_detected: %d\r\n"
          "  current_pulses_sent: %d\r\n"
          "  current_packets_sent: %d\r\n"
          "  pulses_to_sent: %d\r\n"
          ,
          line, last_detected_pulse_time_ms, last_packet_time_ms[line],
          current_pulses_detected, current_pulses_sent, current_packets_sent,
          pulses_to_sent
        );
      }

      last_print_time_ms = GetTimerTimeMs();
    }

    // Delay of 1ms to let the MCU free, not too long to
    // avoid loosing packet to send
    HAL_Delay(10);
  }

  // Program won't go further if we stop udp server
  // udp_server_stop(udp_socket);
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
  /* SystemCoreClock is 80000000 Max is 80000000 <=> 12.5ns */

    htim.Instance               = TIMx;
    htim.Init.Prescaler         = (uint32_t)(SystemCoreClock / uwPrescalerFreq) - 1;
    htim.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim.Init.Period            = 0xFFFFFFFF;
    htim.Init.Period            = uwPeriod;
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
  * @brief  Get Timer Time in us according to TIMx counter
  * @retval Elapsed us since la TIMx reset
  */
static uint32_t GetTimerTimeUsFromCounter(uint32_t count) {
  return count / (uwPrescalerFreq / 1e6);
}

/**
  * @brief  Get Timer Time in us according to TIMx counter
  * @retval Elapsed us since la TIMx reset
  */
static uint32_t GetTimerTimeUs() {
  uint32_t result = __HAL_TIM_GET_COUNTER(&htim);
  result /= uwPrescalerFreq / 1e6;
  return result;
}

/**
  * @brief  Get Timer Time in ms according to TIMx counter
  * @retval Elapsed us since la TIMx reset
  */
static uint32_t GetTimerTimeMs() {
  return GetTimerTimeUs() / 1000;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 to off for error */
  UART_Printf("Error_Handler()\r\n");
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
    stats->std_dev += pow(elapsed-stats->mean, 2);
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
  * @brief  This function configure UART.
  * @retval None
  */
static void UART_Config(void) {

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
    > screen /dev/ttyAMA0 115200,cs7,parenb,parodd,-cstopb,-crtscts
      
  */
  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_ODD;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }

  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }

  /* The board sends the message and expects to receive it back */
  /* DMA is programmed for reception before starting the transmission, in order to
     be sure DMA Rx is ready when board 2 will start transmitting */

  /*##-2- Program the Reception process #####################################*/  
  if(HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;

}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;

}

void UART_Printf(const char* fmt, ...) {
  char buff[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buff, sizeof(buff), fmt, args);

  UartReady = RESET;

  if(HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)buff, strlen(buff))!= HAL_OK)
  {
    Error_Handler();
  }

  va_end(args);

  while (UartReady != SET) {}
}

void UART_Printf_No_Block(const char* fmt, ...) {
  char buff[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buff, sizeof(buff), fmt, args);

  if(HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)buff, strlen(buff))!= HAL_OK)
  {
    Error_Handler();
  }

  va_end(args);
}

/**
  * @brief  Configures EXTI lines 10 to 15 (connected to PC.13 pin) in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTI_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
  EXTIx_0_CLK_ENABLE();
  EXTIx_1_CLK_ENABLE();
  SWIx_0_CLK_ENABLE();
  SWIx_1_CLK_ENABLE();

  /* Configure PB.4 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;

  GPIO_InitStructure.Pin = EXTIx_0_PIN;
  HAL_GPIO_Init(EXTIx_0_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = EXTIx_1_PIN;
  HAL_GPIO_Init(EXTIx_1_GPIO_PORT, &GPIO_InitStructure);

  /* Enable and set EXTI line 4 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTIx_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTIx_IRQn);

  /* Configure PB.4 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;

  GPIO_InitStructure.Pin = SWIx_0_PIN;
  HAL_GPIO_Init(SWIx_0_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = SWIx_1_PIN;
  HAL_GPIO_Init(SWIx_1_GPIO_PORT, &GPIO_InitStructure);

  /* Enable and set EXTI line 4 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(SWIx_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(SWIx_IRQn);
}

/**
  * @brief Copy timestamp content to UDP packet
  * @param udp_packet_index: index in byte of the UDP packet location to fill
  * @param line: timestamp line to copy from
  * @param timestamp_index: index of first timestamp to copy
  * @param count: number of timestamp to copy
  * @retval next position in bytes of the udp packt write cursor
  */
uint16_t CopyTimestampsToBuffer(uint16_t udp_packet_index, uint8_t line,
  uint32_t timestamp_index, uint32_t count) {

  if(count*TIMESTAMP_TYPE_SIZE > UDP_PACKET_DATA_SIZE) {
    UART_Printf("Too much data to send !!!\r\n");
    Error_Handler();
  }

  uint16_t fragment_index = udp_packet_index%UDP_FRAGMENT_SIZE;

  // If trying to write on one header part, move at the beginning of the
  // next data section
  if(fragment_index < UDP_FRAGMENT_HEADER_SIZE) {
    udp_packet_index += UDP_FRAGMENT_HEADER_SIZE - fragment_index;
  }

  if(udp_packet_index + count*TIMESTAMP_TYPE_SIZE > UDP_PACKET_SIZE) {
    UART_Printf("Trying to right out of UDP packet !!!\r\n");
    Error_Handler();
  }

  // If writing the data overlaps the data section, write that part
  // then next parts
  if(fragment_index + count*TIMESTAMP_TYPE_SIZE > UDP_FRAGMENT_SIZE) {

    // Copy the first part, this will copy for sure
    uint32_t first_part_count = (UDP_FRAGMENT_SIZE - fragment_index)/TIMESTAMP_TYPE_SIZE;
    udp_packet_index = CopyTimestampsToBuffer(udp_packet_index, line,
      timestamp_index, first_part_count);

    // Update cursor position and try to copy next part
    // It may be too long too but it will finally succeed
    timestamp_index += first_part_count;
    uint32_t last_part_count = count - first_part_count;
 
    // Call again this method, last part shall fit and won't enter
    // this part of this section
    udp_packet_index = CopyTimestampsToBuffer(udp_packet_index, line,
      timestamp_index, last_part_count);
  } else {
    // All is good, we can copy
    uint32_t* current_timestamp = timestamps[line] + timestamp_index;
    uint8_t* current_udp_packet = udp_packet + udp_packet_index;
    memcpy(current_udp_packet, current_timestamp, count*TIMESTAMP_TYPE_SIZE);

    // Increment packet index for next write
    udp_packet_index += count*TIMESTAMP_TYPE_SIZE;
  }

  // return the next position taking into account header move
  // and total count written
  return udp_packet_index;
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{  
  if (GPIO_Pin == EXTIx_0_PIN || GPIO_Pin == EXTIx_1_PIN) {

    uint8_t line = GPIO_Pin == EXTIx_0_PIN ? 0 : 1;

    uint32_t index = pulses_detected[line]%timestamps_size;

    // And store counter value
    timestamps[line][index] = __HAL_TIM_GET_COUNTER(&htim);

    /* Increment detected pulses */
    pulses_detected[line]++;

    // If there is enough pulse timestamps to send and
    // the transfer is not already occuring, we can trigger
    // the software interrupt that will send the timestamps
    if((pulses_detected[line] >= pulses_sent[line] + TIMESTAMP_PER_PACKET) &&
      !buffering_timestamps) {

      // Tell that transfer is occuring unless it may reentre here
      buffering_timestamps = 1;

      exti.Line = EXTI_LINE_9;
      HAL_EXTI_GenerateSWI(&exti);
    }

  } else if (GPIO_Pin == SWIx_0_PIN || GPIO_Pin == SWIx_1_PIN) {

    uint8_t line = GPIO_Pin == SWIx_0_PIN ? 0 : 1;

    // Get the number of pulse timestamps to send
    // If higher than step one, it will be step one
    // If it has been triggered by exti, then the number
    // of timestamps to send is maximal
    // Else, it has been triggered by main ot timer and
    // The packet won't be complete

    // Tell that transfer is occuring
    buffering_timestamps = 1;

    uint32_t pulses_to_sent = pulses_detected[line] - pulses_sent[line];

    // If there is no pulse to send then return
    if(pulses_to_sent == 0) {
      return;
    }

    // If number of pulse to send exceeds buffer, take only the first
    // missing ones will be sent next time
    if(pulses_to_sent > TIMESTAMP_PER_PACKET) {
      pulses_to_sent = TIMESTAMP_PER_PACKET;
    }

    // Prepare and send udp packets

    // Move pointer the first timestamp not already sent
    uint32_t current_index = pulses_sent[line]%timestamps_size;
    uint16_t udp_packet_index = 0;

    // Call copy buffer, to let the copy focused on filling
    // UDP buffer, check if overlapping here
    // If overlapping, copy the first part
    if(current_index + pulses_to_sent > timestamps_size) {
      uint32_t part = timestamps_size - current_index;
      udp_packet_index = CopyTimestampsToBuffer(udp_packet_index,
          line, current_index, part);

      // update copy variables
      current_index = 0;
      pulses_sent[line] += part;
      pulses_to_sent -= part;
    }

    CopyTimestampsToBuffer(udp_packet_index, line, current_index, pulses_to_sent);
    
    // Update number of pulses sent
    pulses_sent[line] += pulses_to_sent;

    // if(pulses_sent[line] > 20000) {
    //   Error_Handler();
    // }

    // Mark transfer as finished
    buffering_timestamps = 0;

    // prepare UDP packet pointers and size

    // Send it, following method will block until packet is sent
    // Interrupt accuring in the method has higher priority than
    // this one, so the method won't be block be this interrupt
    int32_t nbytes = sendto(udp_socket, udp_packet, UDP_PACKET_SIZE, address,
      dest_ports[line]);

    last_packet_time_ms[line] = GetTimerTimeMs();
    packets_sent[line]++;

    // Mark udp_packet to be sure all data is sent
    memset(udp_packet, 0xFF, UDP_PACKET_SIZE);
  }
}

/* SPIx init function */
static void SPI_Config(void)
{

  /* SPIx parameter configuration*/
  hspi1.Instance = SPIx;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  Tx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Turn LED2 on: Transfer in transmission/reception process is complete */
  // BSP_LED_On(LED2);
  
  wTxTransferState = TRANSFER_COMPLETE;
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Turn LED2 on: Transfer in transmission/reception process is complete */
  // BSP_LED_On(LED2);
  
  wRxTransferState = TRANSFER_COMPLETE;
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  UART_Printf("HAL_SPI_ErrorCallback.\r\n");
  wTxTransferState = TRANSFER_ERROR;
  wRxTransferState = TRANSFER_ERROR;
}

/* w5500 stuff */
void W5500_Select(void) {
  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, GPIO_PIN_6, GPIO_PIN_RESET);
}

void W5500_Unselect(void) {
  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, GPIO_PIN_6, GPIO_PIN_SET);
}

void W5500_ReadBuff(uint8_t* buff, uint16_t len) {

  // HAL_SPI_Receive(&hspi1, buff, len, HAL_MAX_DELAY);

  if(waitRxTransferDone) {
    wRxTransferState = TRANSFER_WAIT;
  } else {
    
  }

  int8_t result;
  do {
    result = HAL_SPI_Receive_DMA(&hspi1, buff, len);
  } while(result == HAL_BUSY);

  if(result != HAL_OK) {
    /* Transfer error in transmission process */
    UART_Printf("HAL_SPI_Receive_DMA ERROR code: %d\r\n", result);
    Error_Handler();
  }

  // Wait transfer is complete
  if(waitRxTransferDone) {
    while (wRxTransferState == TRANSFER_WAIT) {}
  }
}

void W5500_WriteBuff(uint8_t* buff, uint16_t len) {

  if(waitTxTransferDone) {
    wTxTransferState = TRANSFER_WAIT;
  } else {
    while (wTxTransferState == TRANSFER_WAIT) {}
    wTxTransferState = TRANSFER_WAIT;
  }

  int8_t result;
  do {
    result = HAL_SPI_Transmit_DMA(&hspi1, buff, len);
  } while(result == HAL_BUSY);

  if(result != HAL_OK) {
    /* Transfer error in transmission process */
    UART_Printf("HAL_SPI_Transmit_DMA ERROR code: %d\r\n", result);
    Error_Handler();
  }

  // Wait transfer is complete
  if(waitTxTransferDone) {
    while (wTxTransferState == TRANSFER_WAIT) {}
  }
}

uint8_t W5500_ReadByte(void) {
    uint8_t byte;
    W5500_ReadBuff(&byte, sizeof(byte));
    return byte;
}

void W5500_WriteByte(uint8_t byte) {
    W5500_WriteBuff(&byte, sizeof(byte));
}

void Callback_IPAssigned(void) {
    UART_Printf("Callback: IP assigned! Leased time: %d sec\r\n", getDHCPLeasetime());
    ip_assigned = 1;
}
 
void Callback_IPConflict(void) {
    UART_Printf("Callback: IP conflict!\r\n");
}

void http_request_example(uint8_t* addr);
void udp_server_example();

void W5500_Config() {
    UART_Printf("init() called!\r\n");

    UART_Printf("Registering W5500 callbacks...\r\n");
    reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
    reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);

    UART_Printf("Calling wizchip_init()...\r\n");
    // Sum for each must not exceed 16 for each direction
    // Vector size must be 8 for w5500 (_WIZCHIP_SOCK_NUM_)

    // uint8_t tx_buff_sizes[] = {2, 2, 4, 2, 2, 2, 2, 2};
    // uint8_t rx_buff_sizes[] = {2, 2, 4, 2, 2, 2, 2, 2};
    uint8_t tx_buff_sizes[] = {2, 2, 2, 4, 0, 0, 0, 0};
    uint8_t rx_buff_sizes[] = {2, 2, 2, 2, 0, 0, 0, 0};
    if(wizchip_init(tx_buff_sizes, rx_buff_sizes) != 0) {
      UART_Printf("wizchip_init() ERROR !\r\n");
    }

    UART_Printf("Calling DHCP_init()...\r\n");
    wiz_NetInfo net_info = {
        .mac  = { 0xEA, 0x11, 0x22, 0x33, 0x44, 0xEA },
        .dhcp = NETINFO_DHCP
    };
    // set MAC address before using DHCP
    setSHAR(net_info.mac);
    DHCP_init(DHCP_SOCKET, dhcp_buffer);

    UART_Printf("Registering DHCP callbacks...\r\n");
    reg_dhcp_cbfunc(
        Callback_IPAssigned,
        Callback_IPAssigned,
        Callback_IPConflict
    );

    UART_Printf("Calling DHCP_run()...\r\n");
    // actually should be called in a loop, e.g. by timer
    uint32_t ctr = 10000;
    while((!ip_assigned) && (ctr > 0)) {
        DHCP_run();
        ctr--;
    }

    if(!ip_assigned) {
        UART_Printf("\r\nIP was not assigned :(\r\n");
        return;
    }

    getIPfromDHCP(net_info.ip);
    getGWfromDHCP(net_info.gw);
    getSNfromDHCP(net_info.sn);

    uint8_t dns[4];
    getDNSfromDHCP(dns);

    UART_Printf("IP:  %d.%d.%d.%d\r\nGW:  %d.%d.%d.%d\r\nNet: %d.%d.%d.%d\r\nDNS: %d.%d.%d.%d\r\n",
        net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3],
        net_info.gw[0], net_info.gw[1], net_info.gw[2], net_info.gw[3],
        net_info.sn[0], net_info.sn[1], net_info.sn[2], net_info.sn[3],
        dns[0], dns[1], dns[2], dns[3]
    );

    UART_Printf("Calling wizchip_setnetinfo()...\r\n");
    wizchip_setnetinfo(&net_info);

    UART_Printf("Calling DNS_init()...\r\n");
    DNS_init(DNS_SOCKET, dns_buffer);

    uint8_t addr[4];
    {
        char domain_name[] = "google.com";
        UART_Printf("Resolving domain name \"%s\"...\r\n", domain_name);
        int8_t res = DNS_run(dns, (uint8_t*)&domain_name, addr);
        if(res != 1) {
            UART_Printf("DNS_run() failed, res = %d", res);
            return;
        }
        UART_Printf("Result: %d.%d.%d.%d\r\n", addr[0], addr[1], addr[2], addr[3]);
    }

    wiz_PhyConf phy_conf;
    wizphy_getphyconf(&phy_conf);
    phy_conf.speed = PHY_SPEED_100;
    wizphy_setphyconf(&phy_conf);

    // http_request_example(addr);
    // udp_server_example();
}

void http_request_example(uint8_t* addr) {

    UART_Printf("Creating socket...\r\n");
    uint8_t http_socket = HTTP_SOCKET;

    uint8_t code = socket(http_socket, Sn_MR_TCP, 10888, 0);
    if(code != http_socket) {
        UART_Printf("socket() failed, code = %d\r\n", code);
        return;
    }

    UART_Printf("Socket created, connecting...\r\n");
    code = connect(http_socket, addr, 80);
    if(code != SOCK_OK) {
        UART_Printf("connect() failed, code = %d\r\n", code);
        close(http_socket);
        return;
    }

    UART_Printf("Connected, sending HTTP request...\r\n");
    {
        char req[] = "GET / HTTP/1.0\r\nHost: google.com\r\n\r\n";
        uint16_t len = sizeof(req) - 1;
        uint8_t* buff = (uint8_t*)&req;
        while(len > 0) {
            UART_Printf("Sending %d bytes...\r\n", len);
            int32_t nbytes = send(http_socket, buff, len);
            if(nbytes <= 0) {
                UART_Printf("send() failed, %d returned\r\n", nbytes);
                close(http_socket);
                return;
            }
            UART_Printf("%d bytes sent!\r\n", nbytes);
            len -= nbytes;
        }
    }

    UART_Printf("Request sent. Reading response...\r\n");
    {
        char buff[32];
        for(;;) {
            int32_t nbytes = recv(http_socket, (uint8_t*)&buff, sizeof(buff)-1);
            if(nbytes == SOCKERR_SOCKSTATUS) {
                UART_Printf("\r\nConnection closed.\r\n");
                break;
            }

            if(nbytes <= 0) {
                UART_Printf("\r\nrecv() failed, %d returned\r\n", nbytes);
                break;
            }

            buff[nbytes] = '\0';
            UART_Printf("%s", buff);
        }
    }

    UART_Printf("Closing socket.\r\n");
    close(http_socket);
}

void udp_server_start(uint8_t udp_socket, int server_port, uint8_t* address) {

  UART_Printf("Creating UDP socket...\r\n");

  int8_t result;
	uint8_t buffer[] 	= "Wiznet Says Hi!\r\n";
  size_t buffer_size = strlen(buffer);

  // Set sock options before instanciation
  // SO_MSS max seems to be 1472
  // This is the MSS, Maximum Segment Size
  // max is 1472 bytes
  // If packet is above this size, it will
  // be fragmented. WS5500 fragmented packets
  // does not seem to be linked when receiving them
  // So we have to add packet id and fragment so that
  // a client may reassemble them
  // packet strucure shall 
  // | packet id | fragment number | timestamps         |
  // | 4 bytes   | 2 bytes         | 256x4 = 1024 bytes |
  // 
  // uint16_t value = 1024;
  uint16_t value = UDP_FRAGMENT_SIZE;
  
  setsockopt(udp_socket, SO_MSS, &value);

  uint8_t flag = 0;
  // flag = SF_IO_NONBLOCK;
  // flag = SF_BROAD_BLOCK;
  // flag = SF_BROAD_BLOCK | SF_MULTI_ENABLE;
  // flag = SF_BROAD_BLOCK | SF_IGMP_VER2 | SF_MULTI_ENABLE;

  result = socket(udp_socket, Sn_MR_UDP, server_port, flag);
  UART_Printf("socket Result: %d\r\n", result);

  getsockopt(udp_socket, SO_MSS, &value);
  UART_Printf("SO_MSS: %d\r\n", value);

  // UART_Printf("Sending broadcast message (size: %d)...\r\n", buffer_size);
	// uint8_t broadcast_address[4] = { 255, 255, 255, 255 };
	// int broadcast_port = 8040;
  // flag = SF_BROAD_BLOCK | SF_IGMP_VER2 | SF_MULTI_ENABLE;
  // result = socket(HTTP_SOCKET, Sn_MR_UDP, broadcast_port, flag);
  // int32_t nbytes = sendto(HTTP_SOCKET, buffer, buffer_size,
  //   broadcast_address, 8050);
  // UART_Printf("%d bytes sent.\r\n", nbytes);
  // close(HTTP_SOCKET);

  if(*(uint32_t*)address == 0xFFFFFFFF) {

    UART_Printf("Waiting for client...\r\n");

    uint16_t port;
    do {
      result = recvfrom(udp_socket, buffer, buffer_size, address, &port);
    } while (result == 0);
    
    UART_Printf("recvFrom Result: %d\r\n", result);

    UART_Printf("recvFrom address:  %d.%d.%d.%d\r\n",
        address[0], address[1], address[2], address[3]
    );
    UART_Printf("recvFrom port: %u\r\n", port);

    UART_Printf("Received testBuffer: %s\r\n", buffer);

  }

}

void udp_server_stop(uint8_t udp_socket) {

  UART_Printf("Closing socket.\r\n");
  close(udp_socket);

}

void udp_server_example() {

  uint8_t udp_socket = UDP_SOCKET;
	uint8_t address[4] = { 255, 255, 255, 255 };

  int server_port = 8041;
  int dest_port = 8042;

  udp_server_start(udp_socket, server_port, address);

  UART_Printf("Client address:   %d.%d.%d.%d\r\n",
      address[0], address[1], address[2], address[3]
  );

  UART_Printf("Destination port: %d\r\n", dest_port);

  int32_t bytes_to_send = 16*1024; // 8.4ms per packet

  int32_t packes_to_send = 1000;
  // packes_to_send = 1000*1000;

  waitTxTransferDone = 1;
  waitRxTransferDone = 1;

  uint32_t duration;

  for(int i=0;i<packes_to_send;i++) {

    uint16_t len = bytes_to_send;
    uint8_t* buff = (uint8_t*)timestamps[0];

    // duration = __HAL_TIM_GET_COUNTER(&htim);
    int32_t nbytes = sendto(udp_socket, buff, len, address, dest_port);
    // duration = __HAL_TIM_GET_COUNTER(&htim) - duration;
    // UART_Printf("%d: sendto Result: %d, duration: %lu\r\n", i+1, nbytes, duration);
  }

  waitTxTransferDone = 1;
  waitRxTransferDone = 1;

  udp_server_stop(udp_socket);
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
