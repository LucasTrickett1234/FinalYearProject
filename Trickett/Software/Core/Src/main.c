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

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "freeRTOS.h"
#include "task.h"


/* Private define ------------------------------------------------------------*/
#define STM_LOCAL_USB_ID 0b1001

#define USB_DATA_ID_I2C 0b10011001
#define USB_DATA_ID_LDC 0b10011011
#define USB_DATA_ID_ADC 0b10010100
#define USB_DATA_ID_PIO 0b10010001

#define AHT10_ADDRESS (0x38 << 1) // 0b1110000; Adress[7-bit]Wite/Read[1-bit]

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

osThreadId xTask_I2CHandle;
osThreadId xTask_PIOHandle;
osThreadId xTask_PWMHandle;
osThreadId xTask_ADCHandle;

// USB mutex for shared resource
osMutexId usb_mutexHandle;


//static uint16_t dma_normal_adc_buffer[3];
uint16_t dma_normal_adc_buffer[3];
extern uint8_t* UserRxBufferFS;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);

//void xTaskPWMHandle(void const * argument);
void xTaskPIOHandle(void const * argument);
void xTaskI2CHandle(void const * argument);
void xTaskADCHandle(void const * argument);
void xTaskUSBHandle(void const * argument);





volatile pwm_config_t pwm_context = {
	.task_period = 100,
	.pwm1_freq = 1000,
	.pwm2_freq = 1000,
	.pwm1_duty = 0,
	.pwm2_duty = 0,
	.enable_flag1 = false,
	.enable_flag2 = false,
	.change_flag1 = false,
	.change_flag2 = false
};

volatile pio_config_t pio_context = {
	.task_period = 100,
	.pio_mode = ENCODER,
	.enable_flag = false,
	.change_flag = false
};

volatile adc_config_t adc_context = {
	.task_period = 100,
	.enable_flag = false,
	.change_flag = false
};

volatile i2c_config_t i2c_context = {
	.task_period = 100,
	.enable_flag = false,
	.change_flag = false
};



/**
  * @brief  Handles PWM control for motor or peripheral devices.
  * @param  argument: Not used
  * @retval None
  */
void xTaskPWMHandle(void const * argument)
{
	const TickType_t xTaskPeriod_ms = pwm_context.task_period;  // 1000 ms for 1000Hz

	for(;;) {
		osDelay(xTaskPeriod_ms);
		// Check if the PWM1 peripheral is enabled
		if (pwm_context.change_flag1) {
			xTaskPeriod_ms = pwm_context.task_period;
//			__HAL_TIM_SET_PRESCALER(&htim1, pwm1_freq);
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm1_duty);

			if (pwm_context.enable_flag1) {
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			} else {
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			}
			pwm_context.change_flag1 = false;
		}

		// Check if the PWM2 peripheral is enabled
		if (pwm_context.change_flag2) {
//			__HAL_TIM_SET_PRESCALER(&htim3, pwm2_freq);
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm2_duty);

			if (pwm_context.enable_flag2) {
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
			} else {
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
			}
			pwm_context.change_flag2 = false;
		}
	}
}



/**
  * @brief  Handles GPIO tasks for PIO input and encoder control.
  * @param  argument: Not used
  * @retval None
  */
void xTaskPIOHandle(void const * argument)
{
    const TickType_t xTaskPeriod_ms = pio_context.task_period;  // 100 ms for 10Hz

	uint8_t buffer[5];
	buffer[0] = USB_DATA_ID_PIO;

    int8_t encoder_count = 0;
    GPIO_PinState State_PB12;
    GPIO_PinState State_PB13;
    GPIO_PinState Prev_PB12;
    GPIO_PinState Prev_PB13;
	uint32_t Count;


	PIO_MODE_t pio_mode = ENCODER;

    // Read the initial state of the input pin
    Prev_PB12 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);  // Assuming outputA is on GPIOA Pin 0
    Prev_PB13 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);  // Assuming outputA is on GPIOA Pin 0

    /* Infinite loop */
    for(;;) {

		if (pio_context.change_flag) {
			pio_mode = pio_context.pio_mode;
			xTaskPeriod_ms = pio_context.task_period;
			pio_context.change_flag = false;
		}


        if (pio_context.enable_flag) {
			State_PB12 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12); // Read the "current" state of the outputA
			State_PB13 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13); // Read the "current" state of the outputA


			switch (pio_mode)
			{

			case ENCODER:
				osDelay(xTaskPeriod_ms);

				// The Two PIO pins are to read the encoder for reflection measuring
				if (State_PB13 != Prev_PB13) {
					// If outputB state is different from outputA, the encoder is rotating clockwise
					if (State_PB12 != State_PB13) {  // Assuming outputB is on GPIOA Pin 1
						encoder_count++;
					} else {
						encoder_count--;
					}

					buffer[1] = encoder_count;

					osMutexWait(usb_mutexHandle, 10);
					CDC_Transmit_FS(buffer, 2);  // Transmit 7 bytes (buffer contains I2C ID + data)
					osMutexRelease(usb_mutexHandle);
				}
				Prev_PB13 = State_PB13; // Update the previous state of the outputA
				break;


			case INPUT:
				osDelay(xTaskPeriod_ms);
				// The Two PIO pins are to read the spare GPIO pins
				if ((State_PB12 != Prev_PB12) || (State_PB13 != Prev_PB13)) {

					buffer[1] = ((State_PB12 << 4) & State_PB13);

					osMutexWait(usb_mutexHandle, 10);
					CDC_Transmit_FS(buffer, 2);  // Transmit 7 bytes (buffer contains I2C ID + data)
					osMutexRelease(usb_mutexHandle);
				}

				Prev_PB12 = State_PB12; // Update the previous state of the outputA
				Prev_PB13 = State_PB13; // Update the previous state of the outputA
				break;


			case LOADCELL:
				Count = 0;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
				osDelay(xTaskPeriod_ms);

				while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12));

				for (int i=0; i < 26; i++) {
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

			    // Parse the 32-bit integer into the byte array
			    buffer[1] = (Count >> 24) & 0xFF;  // Most significant byte
			    buffer[2] = (Count >> 16) & 0xFF;
			    buffer[3] = (Count >> 8) & 0xFF;
			    buffer[4] = Count & 0xFF;          // Least significant byte

				osMutexWait(usb_mutexHandle, 10);
				CDC_Transmit_FS(buffer, 5);  // Transmit 7 bytes (buffer contains I2C ID + data)
				osMutexRelease(usb_mutexHandle);
				break;


			case UNCONFIGURED:
				osDelay(xTaskPeriod_ms);
				break;


			default:
				osDelay(xTaskPeriod_ms);
				break;
			}
        } else {
			osDelay(xTaskPeriod_ms);
        }
    }
}

/**
  * @brief  Handles I2C communication with the AHT10 sensor and sends data via USB.
  * @param  argument: Not used
  * @retval None
  */
void xTaskI2CHandle(void const * argument)
{
	const TickType_t xTaskPeriod_ms = 10;  // 100 ms for 10Hz
    uint8_t soft_reset_cmd = 0xBA;
    uint8_t init_cmd[3] = {0xE1, 0x08, 0x00};  // Calibration command for AHT10

    HAL_I2C_Master_Transmit(&hi2c1, AHT10_ADDRESS, &soft_reset_cmd, 1, HAL_MAX_DELAY);
	osDelay(xTaskPeriod_ms);

    HAL_I2C_Master_Transmit(&hi2c1, AHT10_ADDRESS, init_cmd, 3, HAL_MAX_DELAY);
	osDelay(xTaskPeriod_ms);

	uint8_t trigger_measurement_cmd[3] = {0xAC, 0x33, 0x00};
	uint8_t data[7];
	data[0] = USB_DATA_ID_I2C;

	uint32_t humidity_raw;
	uint32_t temperature_raw;

	float humidity;
	float temperature;

	for (;;) {
		if (i2c_context.enable_flag) {

			if (i2c_context.change_flag) {
				xTaskPeriod_ms = i2c_context.task_period;
				i2c_context.change_flag = false;

			}

			HAL_I2C_Master_Transmit(&hi2c1, AHT10_ADDRESS, trigger_measurement_cmd, 3, HAL_MAX_DELAY);
			osDelay(xTaskPeriod_ms);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			// Read 6 bytes of data from the AHT10
			if (HAL_I2C_Master_Receive(&hi2c1, AHT10_ADDRESS, data+1, 6, HAL_MAX_DELAY) == HAL_OK) {

			}

			// Check if the sensor is busy
			if ((data[1] & 0x80) == 0) {
				osMutexWait(usb_mutexHandle, 10);
				CDC_Transmit_FS(data, 7);  // Transmit 7 bytes (buffer contains I2C ID + data)
				osMutexRelease(usb_mutexHandle);
			}
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		}
	}
}




/**
  * @brief  Function implementing the xTask_ADC thread for ADC data handling.
  * @param  argument: Not used
  * @retval None
  */
void xTaskADCHandle(void const * argument)
{
	const TickType_t xTaskPeriod_ms = adc_context.task_period;

	char buffer[7];
	buffer[0] = USB_DATA_ID_ADC;

	uint16_t adc1_val;
	uint16_t adc2_val;
	uint16_t adc3_val;

	/* Infinite loop */
	for(;;) {
		// Block until the next execution time to maintain execution rate
		osDelay(xTaskPeriod_ms);

		if (adc_context.enable_flag) {
			if (adc_context.change_flag) {
				xTaskPeriod_ms = adc_context.task_period;
				adc_context.change_flag = false;
			}

			adc1_val = dma_normal_adc_buffer[0];
			adc2_val = dma_normal_adc_buffer[1];
			adc3_val = dma_normal_adc_buffer[2];

	//	    while (HAL_ADC_ConvCpltCallback(&hadc1));
//			buffer[1] = (uint8_t)(dma_normal_adc_buffer[0]  >> 8);   // High byte of adc1
//			buffer[2] = (uint8_t)(dma_normal_adc_buffer[0] & 0xFF); // Low byte of adc1
//
//			// Map adc2 to buffer
//			buffer[3] = (uint8_t)(dma_normal_adc_buffer[1] >> 8);   // High byte of adc2
//			buffer[4] = (uint8_t)(dma_normal_adc_buffer[1] & 0xFF); // Low byte of adc2
//
//			// Map adc3 to buffer
//			buffer[5] = (uint8_t)(dma_normal_adc_buffer[2] >> 8);   // High byte of adc3
//			buffer[6] = (uint8_t)(dma_normal_adc_buffer[2] & 0xFF); // Low byte of adc3


			// Send usb data through mutex, timeout of 10ms
			osMutexWait(usb_mutexHandle, 10);
			CDC_Transmit_FS(buffer, 7);  // Transmit 7 bytes (buffer contains I2C ID + data)
			osMutexRelease(usb_mutexHandle);
		}
	}
}



int main(void)
{
/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	SystemClock_Config();
	MX_DMA_Init();

	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_ADC1_Init();

	HAL_ADCEx_Calibration_Start(&hadc1);

	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	MX_USB_DEVICE_Init();

//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dma_normal_adc_buffer, 3);


	/* definition and creation of usb_mutex */
	osMutexDef(usb_mutex);
	usb_mutexHandle = osMutexCreate(osMutex(usb_mutex));

	/* Create the thread(s) */
	/* definition and creation of xTask_I2C */
	osThreadDef(xTaskI2C, xTaskI2CHandle, osPriorityNormal, 1, 128);
	xTask_I2CHandle = osThreadCreate(osThread(xTaskI2C), NULL);

//	/* definition and creation of xTask_GPIO */
	osThreadDef(xTask_PIO, xTaskPIOHandle, osPriorityNormal, 0, 128);
	xTask_PIOHandle = osThreadCreate(osThread(xTask_ADC), NULL);

//	/* definition and creation of xTask_PWM */
	osThreadDef(xTask_PWM, xTaskPWMHandle, osPriorityNormal, 0, 128);
	xTask_PWMHandle = osThreadCreate(osThread(xTask_PWM), NULL);

	/* definition and creation of xTask_ADC */
	osThreadDef(xTask_ADC, xTaskADCHandle, osPriorityNormal, 2, 128);
	xTask_ADCHandle = osThreadCreate(osThread(xTask_ADC), NULL);

	osKernelStart();

	while (1)
	{
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
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
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

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
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
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 65535;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}


/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
