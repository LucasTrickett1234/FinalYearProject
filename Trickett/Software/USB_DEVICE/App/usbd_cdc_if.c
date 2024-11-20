/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : USB device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Licensed under terms in the LICENSE file in the root directory of this
  * software component. If no LICENSE file comes with this software, it is
  * provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include <stdbool.h>
#include <stdint.h>


/* Definitions for USB data identifiers and configuration IDs */
#define STM_LOCAL_USB_ID 0b1001
#define STM32_CONFIG_ID 0b10010000
#define USB_DATA_ID_I2C 0b10011001
#define USB_DATA_ID_ADC 0b10010100
#define USB_DATA_ID_PIO 0b10010001
#define USB_DATA_ID_PWM 0b10011101

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

/* External variables for different configuration contexts */
extern pwm_config_t pwm_context;
extern pio_config_t pio_context;
extern adc_config_t adc_context;
extern i2c_config_t i2c_context;

/* Buffers for USB communication */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];  // Buffer for received data over USB
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];  // Buffer for data to send over USB

/* Declaration of USB device handle */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* Function prototypes */
static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
void configure_peripheral_context(uint8_t config_message, uint8_t config_data);

/* USB CDC Interface functions */
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Initialize CDC media layer over USB FS */
static int8_t CDC_Init_FS(void)
{
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);  // Set TX buffer
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);     // Set RX buffer
  return (USBD_OK);
}

/* Deinitialize CDC media layer */
static int8_t CDC_DeInit_FS(void)
{
  return (USBD_OK);
}

/* Handle CDC class requests */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:
    case CDC_GET_ENCAPSULATED_RESPONSE:
    case CDC_SET_COMM_FEATURE:
    case CDC_GET_COMM_FEATURE:
    case CDC_CLEAR_COMM_FEATURE:
    case CDC_SET_LINE_CODING:
    case CDC_GET_LINE_CODING:
    case CDC_SET_CONTROL_LINE_STATE:
    case CDC_SEND_BREAK:
    default:
    	break;
  }
  return (USBD_OK);
}


/* Handle data received over USB and process based on data identifier */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);  // Continue receiving packets

  switch(Buf[0])
  {
    case USB_DATA_ID_I2C:
      xTaskI2C_flag = true;
      break;

    case STM32_CONFIG_ID:  // Handle peripheral configuration
      configure_peripheral_context(Buf[1] & 0x3F, Buf[2]);
      break;

    case 'a':  // Debugging case for USB data
      xTaskADC_flag = true;
      break;

    default:
      break;
  }

  return (USBD_OK);
}

void configure_peripheral_context(uint8_t config_message, uint8_t config_data)
{
	switch(config_message)
	{
	case '2':
		// Purely for debugging purposes
		break;
	case ((PWM<<4)|(TOGGLE_TASK<<2)|(PIN_CHANNEL1)):
		// Toggle PWM1
		pwm_context.enable_flag1 = !pwm_context.enable_flag1;
		pwm_context.change_flag1 = true;

	case ((PWM<<4)|(TOGGLE_TASK<<2)|(PIN_CHANNEL2)):
		// Toggle PWM2
		pwm_context.change_flag2 = !pwm_context.change_flag2;
		pwm_context.change_flag2 = true;
		break;

	case ((PWM<<4)|(CONFIG_ATT1<<2)|(PIN_CHANNEL1)):
		// Change PWM frequency for PWM1
		pwm_context.pwm1_freq = config_data*config_data;
		pwm_context.change_flag1 = true;
		break;
	case ((PWM<<4)|(CONFIG_ATT1<<2)|(PIN_CHANNEL2)):
		// Change PWM frequency for PWM2
		pwm_context.pwm2_freq = config_data*config_data;
		pwm_context.change_flag2 = true;
		break;
	case ((PWM<<4)|(CONFIG_ATT2<<2)|(PIN_CHANNEL1)):
		// Change PWM duty-cycle for PWM1
		pwm_context.pwm1_duty = config_data;
		pwm_context.change_flag1 = true;
		break;
	case ((PWM<<4)|(CONFIG_ATT2<<2)|(PIN_CHANNEL2)):
		// Change PWM duty-cycle for PWM2
		pwm_context.pwm1_duty = config_data;
		pwm_context.change_flag2 = true;
		break;

	case ((PWM<<4)|(TASK_FREQ<<2)|(PIN_CHANNEL1)):
	case ((PWM<<4)|(TASK_FREQ<<2)|(PIN_CHANNEL2)):
		// Change PWM duty-cycle for PWM2
		pwm_context.task_period = config_data;
		pwm_context.change_flag1 = true;
		break;


	case ((PIO<<4)|(TOGGLE_TASK<<2)|(PIN_CHANNEL1)):
	case ((PIO<<4)|(TOGGLE_TASK<<2)|(PIN_CHANNEL2)):
		// Prevent PIO task from being executed
		pio_context.enable_flag = !pio_context.enable_flag;
		pio_context.change_flag = true;
		break;

	case ((PIO<<4)|(CONFIG_ATT1<<2)|(PIN_CHANNEL1)):
		// Set PIO1 and PIO2 to be an encoder inputs
		// * does both at same time since can result in
		//   faults if going from encoder to output
		pio_context.pio_mode = ENCODER;
		pio_context.change_flag = true;
		break;

	case ((PIO<<4)|(CONFIG_ATT1<<2)|(PIN_CHANNEL2)):
		// Set PIO1 and PIO2 to be an inputs
		// * does both at same time since can result in
		//   faults if going from encoder to output
		pio_context.pio_mode = INPUT;
		pio_context.change_flag = true;
		break;

	case ((PIO<<4)|(CONFIG_ATT2<<2)|(PIN_CHANNEL1)):
		// Set PIO1 and PIO2 to be loadcell inputs
		pio_context.pio_mode = LOADCELL;
		pio_context.change_flag = true;
		break;

	case ((PIO<<4)|(CONFIG_ATT2<<2)|(PIN_CHANNEL2)):
		// Set PIO2 to be an unconfigured (do nothing)
		pio_context.pio_mode = UNCONFIGURED;
		pio_context.change_flag = true;
		break;

	case ((PIO<<4)|(TASK_FREQ<<2)|(PIN_CHANNEL1)):
	case ((PIO<<4)|(TASK_FREQ<<2)|(PIN_CHANNEL2)):
		pio_context.task_period = config_data;
		pio_context.change_flag = true;
		break;


	case ((ADC<<4)|(TOGGLE_TASK<<2)|(PIN_CHANNEL1)):
	case ((ADC<<4)|(TOGGLE_TASK<<2)|(PIN_CHANNEL2)):
		// Toggle whether ADC gets read to USB
		adc_context.enable_flag = !adc_context.enable_flag;
		break;

	case ((ADC<<4)|(TASK_FREQ<<2)|(PIN_CHANNEL1)):
	case ((ADC<<4)|(TASK_FREQ<<2)|(PIN_CHANNEL2)):
		// Toggles frequency of adc task
		adc_context.task_period = config_data;
		adc_context.change_flag = true;
		break;



	case ((I2C<<4)|(TOGGLE_TASK<<2)|(PIN_CHANNEL1)):
	case ((I2C<<4)|(TOGGLE_TASK<<2)|(PIN_CHANNEL2)):
		// Toggles whether Host can send data to I2C device
		i2c_context.enable_flag = !i2c_context.enable_flag;
		break;

	case ((I2C<<4)|(TASK_FREQ<<2)|(PIN_CHANNEL1)):
	case ((I2C<<4)|(TASK_FREQ<<2)|(PIN_CHANNEL2)):
		i2c_context.task_period = config_data;
		i2c_context.change_flag = true;
		break;

	default:
		  break;
	}
}




/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
	uint8_t result = USBD_OK;

	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

	if (hcdc->TxState != 0) {
		return USBD_BUSY;
	}

	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
	result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
	return result;
}
