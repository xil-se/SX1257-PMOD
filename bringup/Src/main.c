
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// #define TX_TEST
#define RX_TEST

#include "sc18is602b.h"

struct SC18IS602B_config SC18IS602B_config = {
  .user_address = 0x00,
  .handle = &hi2c1,
  .timeout = 1000,
  .i2c_write = (SC18IS602B_i2c_write) HAL_I2C_Master_Transmit,
  .i2c_read = (SC18IS602B_i2c_read) HAL_I2C_Master_Receive,
};


void sx1257_set_tx_freq(int32_t freq)
{
  uint32_t out_freq = (uint32_t)(((float)freq) / 68.66455f);
  // HAL_Delay(5000);
  // printf("Freq: 0x%08x\r\n", out_freq);

  {
    uint8_t data[] = {
      SC18IS602B_SS0,
      0x80 | 0x04, // RegFrfTxMsb
      (out_freq >> 16) & 0xff,
    };
    SC18IS602B_config.i2c_write(SC18IS602B_config.handle, SC18IS602B_ADDR_W(&SC18IS602B_config),
      data, sizeof(data), SC18IS602B_config.timeout);
  }
  HAL_Delay(1);
  {
    uint8_t data[] = {
      SC18IS602B_SS0,
      0x80 | 0x05, // RegFrfTxxMid
      (out_freq >> 8) & 0xff,
    };
    SC18IS602B_config.i2c_write(SC18IS602B_config.handle, SC18IS602B_ADDR_W(&SC18IS602B_config),
      data, sizeof(data), SC18IS602B_config.timeout);
  }
  HAL_Delay(1);
  {
    uint8_t data[] = {
      SC18IS602B_SS0,
      0x80 | 0x06, // RegFrfTxLsb
      out_freq & 0xff,
    };
    SC18IS602B_config.i2c_write(SC18IS602B_config.handle, SC18IS602B_ADDR_W(&SC18IS602B_config),
      data, sizeof(data), SC18IS602B_config.timeout);
  }
  // HAL_Delay(1);

}

void sx1257_set_rx_freq(int32_t freq)
{
  uint32_t out_freq = (uint32_t)(((float)freq) / 68.66455f);
  // HAL_Delay(5000);
  // printf("Freq: 0x%08x\r\n", out_freq);

  {
    uint8_t data[] = {
      SC18IS602B_SS0,
      0x80 | 0x01, // RegFrfRxMsb
      (out_freq >> 16) & 0xff,
    };
    SC18IS602B_config.i2c_write(SC18IS602B_config.handle, SC18IS602B_ADDR_W(&SC18IS602B_config),
      data, sizeof(data), SC18IS602B_config.timeout);
  }
  HAL_Delay(1);
  {
    uint8_t data[] = {
      SC18IS602B_SS0,
      0x80 | 0x02, // RegFrfRxMid
      (out_freq >> 8) & 0xff,
    };
    SC18IS602B_config.i2c_write(SC18IS602B_config.handle, SC18IS602B_ADDR_W(&SC18IS602B_config),
      data, sizeof(data), SC18IS602B_config.timeout);
  }
  HAL_Delay(1);
  {
    uint8_t data[] = {
      SC18IS602B_SS0,
      0x80 | 0x03, // RegFrfRxLsb
      out_freq & 0xff,
    };
    SC18IS602B_config.i2c_write(SC18IS602B_config.handle, SC18IS602B_ADDR_W(&SC18IS602B_config),
      data, sizeof(data), SC18IS602B_config.timeout);
  }
  // HAL_Delay(1);

}

void sx1257_set_rx_ana_gain(uint8_t RxLnaGain, uint8_t RxBasebandGain, uint8_t LnaZin)
{
  {
    uint8_t data[] = {
      SC18IS602B_SS0,
      0x80 | 0x0C, // RegRxAnaGain
      ((RxLnaGain & 0b111) << 5) |
      ((RxBasebandGain & 0b1111) << 1) |
      (LnaZin & 0b1)
    };
    SC18IS602B_config.i2c_write(SC18IS602B_config.handle, SC18IS602B_ADDR_W(&SC18IS602B_config),
      data, sizeof(data), SC18IS602B_config.timeout);
  }
}

#include "usbd_cdc_if.h"
int _write(int fd, uint8_t *data, int count)
{
  CDC_Transmit_FS(data, count);
  return count;
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
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  int count = 0;


/////////// SX1257 driver //////////

  // Enable GPIO for SS1 - It's connected to the RESET pin. SS0 is connected to NSS/CS
  SC18IS602B_gpio_enable(&SC18IS602B_config, SC18IS602B_SS1);

  // RESET = 1
  SC18IS602B_gpio_write(&SC18IS602B_config, SC18IS602B_SS1);
  HAL_Delay(100);

  // Configure SPI
  // MSB is transmitted first
  // CLK is low when idle
  // data is clocked on leading edge
  // SPI CLK is 1843 kHz
  SC18IS602B_spi_config(&SC18IS602B_config,
    SC18IS602B_SPI_CONFIG_VALUES(0, 0, 0, SC18IS602B_SPI_FREQ_1843)
  );
  // HAL_Delay(100);

  // RESET = 0
  SC18IS602B_gpio_write(&SC18IS602B_config, 0);
  HAL_Delay(100);

#ifdef TX_TEST
  // write TX frequency
  // sx1257_set_tx_freq(433000000);
  // sx1257_set_tx_freq(915000000);
  sx1257_set_tx_freq(868000000);
  HAL_Delay(100);
#endif

#ifdef RX_TEST
  // write RX frequency
  sx1257_set_rx_freq(868000000 - 10000);
  HAL_Delay(100);
  sx1257_set_rx_ana_gain(0b001, 0b1111, 0b0);
  HAL_Delay(100);
#endif

  {
    uint8_t data[] = {
      SC18IS602B_SS0,
      0x80 | 0x00, // operating mode
      0x08 | // PA
#ifdef TX_TEST
      0x04 | // TX
#endif
#ifdef RX_TEST
      0x02 | // RX
#endif
      0x01, // IDLE
    };
    SC18IS602B_config.i2c_write(SC18IS602B_config.handle, SC18IS602B_ADDR_W(&SC18IS602B_config),
      data, sizeof(data), SC18IS602B_config.timeout);
    HAL_Delay(100);
  }


  {
    uint8_t data[] = {
      SC18IS602B_SS0,
      0x80 | 0x10, // clk select
      0x02 | 0x00, // clkout enabled, use internal for tx
    };
    SC18IS602B_config.i2c_write(SC18IS602B_config.handle, SC18IS602B_ADDR_W(&SC18IS602B_config),
      data, sizeof(data), SC18IS602B_config.timeout);
    HAL_Delay(100);
  }

  // {
  //   uint8_t data[] = {
  //     SC18IS602B_SS0,
  //     0x0F, // DIO mapping
  //     0x00, // use default
  //   };
  //   SC18IS602B_config.i2c_write(SC18IS602B_config.handle, SC18IS602B_ADDR_W(&SC18IS602B_config),
  //     data, sizeof(data), SC18IS602B_config.timeout);
  //   HAL_Delay(1);
  // }

  // HAL_Delay(100);


//////////////////////////////////////////

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

    // count++;
    // printf("Loop %d\r\n", count);
    


    // read gpio:
    // uint8_t value = 0xFF;
    // ret = SC18IS602B_gpio_read(0x0F, &value);
    // printf("  read=%d %d\r\n", ret, value);

#ifdef RX_TEST
    static uint8_t rx_buf[1024*4];
    for (int i = 0; i < sizeof(rx_buf); i++) {
      register uint8_t reg = 0;
      for (int j = 0; j < 32; j++) {
        // reg |= ((uint32_t)((GPIOB->IDR & GPIO_PIN_3) == 0)) << j;
        reg += (uint32_t)((GPIOB->IDR & GPIO_PIN_3) == 0 ? 0:1);
      }
      rx_buf[i] = reg;
      // rx_buf[i] = GPIOB->IDR & GPIO_PIN_3;
    }
    write(0, rx_buf, sizeof(rx_buf));
    // printf("Yea\r\n");
#endif


#ifdef TX_TEST
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    static unsigned int x,y;
    // if (x == 0)
    //   sx1257_set_tx_freq(868000000 + (rand()) % 1000000);

    for (int i = 0; i < 10; i++) {
      // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
      // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
      GPIOB->ODR ^= GPIO_PIN_4;
      // HAL_Delay(1);
      for (uint32_t xx = 10000+x; xx > 0; xx--) {
        __asm__("nop");
      }
    }
    x = (x+1000) % 10000;
    // x = (x+y) % (10000);
    // y = ((y+1)*(3));
#endif //TX_TEST

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;

  // 100 kHz
  // hi2c1.Init.Timing = 0x2000090E;

  // 400 kHz
  hi2c1.Init.Timing = 0x0000020B;

  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; //GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure input pin 39/PB3
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; //GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
