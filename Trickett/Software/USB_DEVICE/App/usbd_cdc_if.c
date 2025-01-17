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

// Local STM32 ID for USB communication
// * To be set to a different ID when programming different boards
#define STM_LOCAL_USB_ID 0b1001 // The local ID of the STM32



/* Definitions for USB data identifiers and configuration IDs */
#define STM32_CONFIG_ID  0b0000
#define USB_DATA_ID_I2C  0b1001
#define USB_DATA_ID_ADC  0b0100
#define USB_DATA_ID_PIO  0b0001
#define USB_DATA_ID_PWM  0b1101


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


/**
  * @brief  Initializes the USB CDC media layer.
  *         Sets up TX and RX buffers for USB communication.
  * @retval USBD_OK if initialization is successful.
  */
static int8_t CDC_Init_FS(void)
{
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);  // Set TX buffer
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);     // Set RX buffer
  return (USBD_OK);
}


/**
  * @brief  Deinitializes the USB CDC media layer.
  *         Called when the USB CDC is no longer needed.
  * @retval USBD_OK if deinitialization is successful.
  */
static int8_t CDC_DeInit_FS(void)
{
  return (USBD_OK);
}


/**
  * @brief  Handles USB CDC class-specific control requests.
  *         Processes commands such as setting line coding, control line state, etc.
  * @param  cmd: Command code (e.g., CDC_SET_LINE_CODING).
  * @param  pbuf: Buffer containing the request data.
  * @param  length: Length of the request data in bytes.
  * @retval USBD_OK if the command is successfully processed.
  */
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
    case (STM_LOCAL_USB_ID << 4) | USB_DATA_ID_I2C:
      xTaskI2C_flag = true;
      break;

    case (STM_LOCAL_USB_ID << 4) | STM32_CONFIG_ID:  // Handle peripheral configuration
      configure_peripheral_context(Buf[1] & 0x3F, Buf[2]);
      break;

    default:
      break;
  }

  return (USBD_OK);
}


/**
  * @brief  Handles data received over the USB CDC interface.
  *         Processes data based on its identifier (e.g., I2C, ADC).
  * @param  Buf: Pointer to the buffer containing received data.
  * @param  Len: Pointer to the length of received data.
  * @retval USBD_OK if data is successfully processed.
  */
void configure_peripheral_context(uint8_t config_message, uint8_t config_data)
{
	switch(config_message)
	{
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
