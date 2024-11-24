/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.h
  * @version        : v2.0_Cube
  * @brief          : Header for usbd_cdc_if.c file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief For Usb device.
  * @{
  */

/** @defgroup USBD_CDC_IF USBD_CDC_IF
  * @brief Usb VCP device module
  * @{
  */

/** @defgroup USBD_CDC_IF_Exported_Defines USBD_CDC_IF_Exported_Defines
  * @brief Defines.
  * @{
  */
/* Define size for the receive and transmit buffer over CDC */
#define APP_RX_DATA_SIZE  1024
#define APP_TX_DATA_SIZE  1024
/* USER CODE BEGIN EXPORTED_DEFINES */

/* USER CODE END EXPORTED_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Types USBD_CDC_IF_Exported_Types
  * @brief Types.
  * @{
  */

/* USER CODE BEGIN EXPORTED_TYPES */

/* USER CODE END EXPORTED_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Macros USBD_CDC_IF_Exported_Macros
  * @brief Aliases.
  * @{
  */

/* USER CODE BEGIN EXPORTED_MACRO */

/* USER CODE END EXPORTED_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

/** CDC Interface callback. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;



/* Enum for different PIO modes */
typedef enum {
	ENCODER = 0,
	INPUT,
	UNCONFIGURED,
	LOADCELL
} PIO_MODE_t;

/* Struct to configure PIO */
typedef struct {
	uint32_t task_period;
	PIO_MODE_t pio_mode;
	bool enable_flag;
	bool change_flag;
} pio_config_t;

/* Struct to configure I2C */
typedef struct {
	uint32_t task_period;
	bool enable_flag;
	bool change_flag;
} i2c_config_t;

/* Struct to configure PWM */
typedef struct {
	uint32_t task_period;
	uint16_t pwm1_freq;
	uint16_t pwm2_freq;
	uint8_t pwm1_duty;
	uint8_t pwm2_duty;
	bool enable_flag1;
	bool enable_flag2;
	bool change_flag1;
	bool change_flag2;
} pwm_config_t;

/* Struct to configure ADC */
typedef struct {
	uint32_t task_period;
	bool enable_flag;
	bool change_flag;
} adc_config_t;

/* Enum for different configuration targets */
typedef enum {
	PWM = 0,
	PIO,
	ADC,
	I2C
} config_target_t;

/* Enum for configuration attributes */
typedef enum {
	TOGGLE_TASK = 0,
	CONFIG_ATT1,
	CONFIG_ATT2,
	TASK_FREQ
} config_attribute_t;

/* Enum for configuration channel */
typedef enum {
	PIN_CHANNEL1 = 0,
	PIN_CHANNEL2
} config_channel_t;





/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_FunctionsPrototype USBD_CDC_IF_Exported_FunctionsPrototype
  * @brief Public functions declaration.
  * @{
  */

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

/* USER CODE BEGIN EXPORTED_FUNCTIONS */

/* USER CODE END EXPORTED_FUNCTIONS */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */

