# Final Year Project

## STM32 Software
To help others understand how to configure your STM32 and PCB setup using the `configure_peripheral_context` function, here’s a comprehensive documentation that explains the structure and functionality of the configuration system.

Most help from the code can be self-done by viewing USBD_CDC_IF.c file

---

### **Function Overview**

`void configure_peripheral_context(uint8_t config_message, uint8_t config_data)`

This function allows users to configure various peripherals (PWM, PIO, ADC, I2C) on the STM32 using a message-based protocol. The function processes incoming messages to perform tasks such as toggling, adjusting frequency, or changing operational modes. 

### **Message Structure**

The function expects a 3-byte message:

- **Byte 1**: Specifies the STM32 and indicates that a configuration is being performed.
    - **First 4 bits**: STM32 selection
    - **Next 4 bits**: Denotes that this is a configuration message.

- **Byte 2**: Encoded in the `config_message` argument.
    - **Bits 7-4**: Peripheral selection (e.g., PWM, PIO, ADC, I2C).
    - **Bits 3-2**: Task or attribute (e.g., toggle, configure frequency/duty, etc.).
    - **Bits 1-0**: Pin/channel (e.g., PIN_CHANNEL1, PIN_CHANNEL2).

- **Byte 3**: Contains `config_data`, which provides specific data for the task (e.g., frequency, duty cycle, etc.).

### **Supported Peripherals**

The following peripherals are supported:

- **PWM (Pulse Width Modulation)**
- **PIO (Programmable Input/Output)**
- **ADC (Analog to Digital Converter)**
- **I2C (Inter-Integrated Circuit)**

### **Byte 2 Format (config_message)**

| **Bits** | **Field**      | **Description**                                          |
|----------|----------------|----------------------------------------------------------|
| 7-4      | Peripheral     | Specifies which peripheral to configure (e.g., PWM, PIO). |
| 3-2      | Task/Attribute | Specifies the task or attribute (e.g., toggle, configure).|
| 1-0      | Pin/Channel    | Specifies the pin or channel number (e.g., PIN_CHANNEL1). |

#### **Peripheral Definitions** (Bits 7-4)

- **PWM** = `0x1`
- **PIO** = `0x2`
- **ADC** = `0x3`
- **I2C** = `0x4`

#### **Task/Attribute Definitions** (Bits 3-2)

- **TOGGLE_TASK** = `0x0`: Toggle the functionality of the peripheral (e.g., enable/disable).
- **CONFIG_ATT1** = `0x1`: Configure attribute 1 (e.g., frequency, mode).
- **CONFIG_ATT2** = `0x2`: Configure attribute 2 (e.g., duty cycle, specific functionality).
- **TASK_FREQ** = `0x3`: Set task frequency for the peripheral.

#### **Pin/Channel Definitions** (Bits 1-0)

- **PIN_CHANNEL1** = `0x0`: Select pin/channel 1.
- **PIN_CHANNEL2** = `0x1`: Select pin/channel 2.

### **Byte 3 Format (config_data)**

This byte contains specific data related to the task/attribute. For example:

- If configuring frequency for PWM, `config_data` represents the new frequency.
- If configuring duty cycle for PWM, `config_data` represents the new duty cycle.
- If toggling a peripheral, `config_data` is not used.

### **PWM Configuration**

The PWM peripheral can be configured to:
- **Toggle PWM** for a specific channel.
- **Change frequency** of PWM for a specific channel.
- **Change duty cycle** of PWM for a specific channel.

#### **Examples:**
```c
case ((PWM<<4)|(TOGGLE_TASK<<2)|(PIN_CHANNEL1)):
    // Toggle PWM1
    pwm_context.enable_flag1 = !pwm_context.enable_flag1;
    pwm_context.change_flag1 = true;
    break;

case ((PWM<<4)|(CONFIG_ATT1<<2)|(PIN_CHANNEL1)):
    // Change PWM frequency for PWM1
    pwm_context.pwm1_freq = config_data * config_data;
    pwm_context.change_flag1 = true;
    break;

case ((PWM<<4)|(CONFIG_ATT2<<2)|(PIN_CHANNEL1)):
    // Change PWM duty-cycle for PWM1
    pwm_context.pwm1_duty = config_data;
    pwm_context.change_flag1 = true;
    break;
```

### **PIO Configuration**

The PIO peripheral can be configured to:
- **Toggle PIO tasks**.
- **Set PIO mode** (e.g., encoder, input, load cell).
- **Set task frequency** for PIO operations.

#### **Examples:**
```c
case ((PIO<<4)|(TOGGLE_TASK<<2)|(PIN_CHANNEL1)):
    // Toggle PIO tasks
    pio_context.enable_flag = !pio_context.enable_flag;
    pio_context.change_flag = true;
    break;

case ((PIO<<4)|(CONFIG_ATT1<<2)|(PIN_CHANNEL1)):
    // Set PIO1 and PIO2 to be encoder inputs
    pio_context.pio_mode = ENCODER;
    pio_context.change_flag = true;
    break;
```

### **ADC Configuration**

The ADC peripheral can be configured to:
- **Toggle ADC tasks** (e.g., whether ADC readings are sent to USB).
- **Set ADC task frequency**.

#### **Examples:**
```c
case ((ADC<<4)|(TOGGLE_TASK<<2)|(PIN_CHANNEL1)):
    // Toggle whether ADC gets read to USB
    adc_context.enable_flag = !adc_context.enable_flag;
    break;

case ((ADC<<4)|(TASK_FREQ<<2)|(PIN_CHANNEL1)):
    // Set ADC task frequency
    adc_context.task_period = config_data;
    adc_context.change_flag = true;
    break;
```

### **I2C Configuration**

The I2C peripheral can be configured to:
- **Toggle I2C tasks** (e.g., enable/disable communication).
- **Set I2C task frequency**.

#### **Examples:**
```c
case ((I2C<<4)|(TOGGLE_TASK<<2)|(PIN_CHANNEL1)):
    // Toggle whether Host can send data to I2C device
    i2c_context.enable_flag = !i2c_context.enable_flag;
    break;

case ((I2C<<4)|(TASK_FREQ<<2)|(PIN_CHANNEL1)):
    // Set I2C task frequency
    i2c_context.task_period = config_data;
    i2c_context.change_flag = true;
    break;
```

---

### **Customization and Future Modifications**

To modify or extend the configuration capabilities for new peripherals, you can:

1. **Add new cases** to the switch statement in `configure_peripheral_context`.
2. Define new peripheral IDs (bits 7-4), task/attribute definitions (bits 3-2), or pin/channel definitions (bits 1-0) as needed.
3. Adjust `config_data` processing to handle new types of data or behaviors.

By following this structure, the system remains flexible and easily extendable, allowing new peripherals and features to be integrated into your STM32-based PCB configuration.