/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include <string.h>
#include <math.h>
#include "power.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

char log_buffer[MAX_LOG_BUFFER];
static uint32_t last_readout;

#define ONE_WIRE_SKIP_ROM 0xCC
#define ONE_WIRE_READ_SCRATCH_COMMAND 0xBE
#define ONE_WIRE_CONVERT_T 0x44

// This table comes from Dallas sample code where it is freely reusable,
// though Copyright (C) 2000 Dallas Semiconductor Corporation
static const uint8_t dscrc_table[] = {
      0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
    157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
     35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
    190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
     70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
    219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
    101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
    248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
    140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
     17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
    175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
     50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
    202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
     87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
    233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
    116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t crc8(const uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;
  
  while (len--) {
    crc = dscrc_table[crc ^ *addr++];
  }
  return crc;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_TIM8_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);      // LED on
  snprintf(log_buffer, sizeof(log_buffer),
	   "\nemonTxshield Demo 1.11\n");
  debug_printf(log_buffer);
  snprintf(log_buffer, sizeof(log_buffer),
	   "Patch PA0 through to PB14 for V!!!\n");
  debug_printf(log_buffer);

  init_uarts();
  calibrate_ADCs();
  init_power();                    // Starts ADCs running

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    process_power_data();

    //
    // Dump out the environmental stats every 10 seconds, but it takes the DS18B20s
    // about a second to do a conversion, so go early.
    //
    if (HAL_GetTick() - last_readout > 9000) {
      static uint8_t tx_buff[10];
      static uint8_t rx_buff[MAX_1WIRE_BUS][10];
      uint8_t bus_status[MAX_1WIRE_BUS], crc_status[MAX_1WIRE_BUS];
      float temp_f[MAX_1WIRE_BUS];
      int16_t temp_i[MAX_1WIRE_BUS];

      //
      // Reset both 1wire buses to start a command.  This code is NOT an example
      // of how to use the DS18B20, in fact, it's an example of how NOT to use it.
      // It simply uses two DS18B20s (one per bus) as a handy OneWire slave in order
      // to demo/test UART off-load of OneWire transactions.  As such it does no device
      // discovery, and no device selection by ROM code.  It will only work provided there
      // is only one DS18B20 per bus.  There are plenty of examples on the net implementing
      // the full DS18B20 discovery and addressing stuff, use them instead if you're
      // trying to do anything serious with DS18B20s.  Here they're just a handy OneWire
      // slave to bang against in a very primitive fashion.
      //
      // Also this code deliberately interleaves all transactions between the two buses
      // rather than take the more obvious approach of doing a for (bus=0; bus<2; bus++)
      // loop.  In this way we can demonstrate the simultaneous running of the two buses.
      //
      bus_status[0] = onewire_reset(0);
      bus_status[1] = onewire_reset(1);
      tx_buff[0] = ONE_WIRE_SKIP_ROM;          // Only one device per bus in this demo
      tx_buff[1] = ONE_WIRE_CONVERT_T;
      onewire_tx(0, tx_buff, 2);               // Tell the device on bus 0 to start a temp conversion
      onewire_tx(1, tx_buff, 2);               // Tell the device on bus 1 to start a temp conversion
      HAL_Delay(1000);                         // Let it complete

      onewire_reset(0);
      onewire_reset(1);
      tx_buff[1] = ONE_WIRE_READ_SCRATCH_COMMAND;
      onewire_tx(0, tx_buff, 2);
      onewire_tx(1, tx_buff, 2);
      onewire_rx1(0, rx_buff[0], 9);    // get all 9 bytes of scratch pad - part1 bus0
      onewire_rx1(1, rx_buff[1], 9);    // get all 9 bytes of scratch pad - part1 bus1
      onewire_rx2(0, rx_buff[0], 9);    // get all 9 bytes of scratch pad - part2 bus0
      onewire_rx2(1, rx_buff[1], 9);    // get all 9 bytes of scratch pad - part2 bus1
      crc_status[0] = crc8(rx_buff[0], 9);
      temp_i[0] = ((int16_t)rx_buff[0][1] << 11 | (int16_t)rx_buff[0][0] << 3);
      temp_f[0] = temp_i[0] * 0.0078125;
      crc_status[1] = crc8(rx_buff[1], 9);
      temp_i[1] = ((int16_t)rx_buff[1][1] << 11 | (int16_t)rx_buff[1][0] << 3);
      temp_f[1] = temp_i[1] * 0.0078125;

      snprintf(log_buffer, sizeof(log_buffer), "CPU temp: %dC, Vdda: %dmV\n",
	       get_cpu_temp(), get_vdd());
      debug_printf(log_buffer);

      snprintf(log_buffer, sizeof(log_buffer),
	       "bus0 status: 0x%02x, crc: %s, temp: %.3f\n",
	       bus_status[0], crc_status[0] ? "bad" : " ok", temp_f[0]);
      debug_printf(log_buffer);
      snprintf(log_buffer, sizeof(log_buffer),
	       "bus1 status: 0x%02x, crc: %s, temp: %.3f\n",
	       bus_status[1], crc_status[1] ? "bad" : " ok", temp_f[1]);
      debug_printf(log_buffer);
      last_readout = HAL_GetTick();
    }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_TIM8;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
