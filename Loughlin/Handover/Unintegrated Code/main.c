/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
// Encoder counter value
volatile int encoderValue = 0;
volatile uint8_t lastStateA = 0;  // Previous state of OUTA
volatile uint8_t lastStateB = 0;  // Previous state of OUTB
#define AHT10_ADDRESS 0x38 << 1   // 0b1110000; Adress[7-bit]Wite/Read[1-bit]


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Load Cell:

unsigned long ReadCount(void) {
	unsigned long Count = 0;
	unsigned char i;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);


	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12));

	for (i=0; i < 26; i++) {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	  Count = Count << 1;
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) {
		  Count++;
	  }
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	Count = Count ^ 0x800000;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

	return Count;
}

// Currently only measuring 50Kg
float convertToKgf(unsigned long raw_value) {
	float kgf = 0.0;

	// Constants for jump detection and adjustment

	long jump_offset = 16000000;        // Jump offset (observed ~16,000,000)

	float slope = -1.5617e-6;
	float intercept = 90.95;


	if ((raw_value < 70000000 && raw_value > 58300000) || (raw_value < 50000000 && raw_value > 41900000)) {
		raw_value = raw_value - jump_offset;
	}

	kgf = slope * raw_value + intercept;

	return kgf;
}
// Another Scale for the Load Cell - Bigger Range but uncertain on accuracy - 0 - 72Kg
// float convertToKgf(unsigned long raw_value) {
// 	float kgf = 0.0;

// 	// Constants for jump detection and adjustment

// 	long jump_offset = 16000000;        // Jump offset (observed ~16,000,000)

// 	float a = 3.1e-13;
// 	float b = -1.5623e-5;
// 	float intercept = 196.83;


// 	if ((raw_value > 25060000) ) {
// 		raw_value = raw_value - jump_offset;
// 	}

// 	kgf = a * raw_value * raw_value + b* raw_value  + intercept;

// 	previousLoadCount = raw_value;
// 	return kgf;
// }

// Battery Temperature:

uint32_t Read_Temperature_ADC(void) {
    uint32_t adc_value = 0;

    // Start ADC Conversion
    HAL_ADC_Start(&hadc1);

    // Wait for conversion to finish
    if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
        // Get the ADC value
        adc_value = HAL_ADC_GetValue(&hadc1);
    }

    // Stop ADC
    HAL_ADC_Stop(&hadc1);

    return adc_value;
}

float Convert_ADC_to_Temperature(uint32_t adc_value) {
	return -22.22 * log((13114.0f/((float)adc_value)) - 3.2f) + 3.5;
}


// Rotary Encoder:
// PWM2 = PB0 
// PWM1 = PA8
void readEncoder(void)
{
    // Read current states of OUTA and OUTB
    uint8_t stateA = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);  // OUTA
    uint8_t stateB = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);  // OUTB

    // Detect state changes (rising/falling edges)
    if (stateA != lastStateA)  // OUTA has changed
    {
        // Determine direction based on OUTB's state relative to OUTA
        if (stateA == stateB)  // Both are the same -> Clockwise (CW)
        {
            encoderValue++;
        }
        else  // OUTA and OUTB are different -> Counter-clockwise (CCW)
        {
            encoderValue--;
        }
    }

    // Store the current states for the next comparison
    lastStateA = stateA;
    lastStateB = stateB;
}


/* USER CODE END 0 */
float convert_rotary_encoder(void) {
	return encoderValue * 3.5;
}


// AHT10 Ambient Temperature Sensor:

// Function to send a soft reset to the AHT10
void AHT10_SoftReset(void) {
    uint8_t soft_reset_cmd = 0xBA;
    HAL_I2C_Master_Transmit(&hi2c1, AHT10_ADDRESS, &soft_reset_cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(20);  // Wait for the sensor to reset
}

// Function to initialize the AHT10 sensor
void AHT10_Init(void) {
    uint8_t init_cmd[3] = {0xE1, 0x08, 0x00};  // Calibration command for AHT10
    HAL_I2C_Master_Transmit(&hi2c1, AHT10_ADDRESS, init_cmd, 3, HAL_MAX_DELAY);
    HAL_Delay(100);  // Give the sensor time to initialize
}


// Function to read temperature and humidity data
void AHT10_ReadData(float *humidity, float *temperature) {
	uint8_t trigger_measurement_cmd[3] = {0xAC, 0x33, 0x00};
	uint8_t data[6];

	// Send the trigger measurement command
	HAL_I2C_Master_Transmit(&hi2c1, AHT10_ADDRESS, trigger_measurement_cmd, 3, HAL_MAX_DELAY);
	HAL_Delay(80);  // Measurement takes some time, so wait at least 75ms

	// Read 6 bytes of data from the AHT10
	HAL_I2C_Master_Receive(&hi2c1, AHT10_ADDRESS, data, 6, HAL_MAX_DELAY);

	// Check if the sensor is busy
	if ((data[0] & 0x80) == 0) {
		// Humidity data (20 bits from data[1], data[2], data[3])
		uint32_t humidity_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((data[3] >> 4) & 0x0F);

		// Temperature data (20 bits from data[3], data[4], data[5])
		uint32_t temperature_raw = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];

		// Convert raw values to actual values
		*humidity = ((float)humidity_raw / 1048576.0) * 100.0;  // Convert to percentage
		*temperature = ((float)temperature_raw / 1048576.0) * 200.0 - 50.0;  // Convert to Celsius
	}
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{


  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

 

  /* Configure the system clock */
  SystemClock_Config();



  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  
  // Initialize the AHT10
  AHT10_SoftReset();
  AHT10_Init();

  // Initial states of OUTA and OUTB
  lastStateA = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);  // OUTA
  lastStateB = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);  // OUTB


  uint8_t rotary_encoder_buffer[3];
 
 
  rotary_encoder_buffer[0] = 0b10111000; //0xB8

 
  while (1)
  {
    
//	loadCellCount = ReadCount();
//	loadCellkgf = convertToKgf(loadCellCount);

	readEncoder();


	encoderTest = encoderValue;
	rotary_encoder_buffer[1] = (encoderValue  >> 8) & 0xFF;
	rotary_encoder_buffer[2] = encoderValue  & 0xFF;
	CDC_Transmit_FS(rotary_encoder_buffer, 3);

  }
 
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Enable and set EXTI Line Interrupt for the encoder CLK pin (PB0 corresponds to EXTI0)
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);  // Enable interrupt for EXTI line 0 (PB0)

}

/* USER CODE BEGIN 4 */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);  // Call the EXTI handler for the encoder CLK pin (PB0)
}
/* USER CODE END 4 */

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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

