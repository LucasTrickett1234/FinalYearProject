import time
import serial
import numpy as np

#Legend of keys for receiving data
#I2C 0x99 (Temp/Humidity)
#ADC 0x94
#PIO 0x91
#PWM 0x9D
#PWM 0x9B

#Format for configuring datalogging PCB
#0x(STMNUMBER)(DATATYPE)

#Try to receive data for x seconds before timeout
COMM_TIMOUT = 1

#General function for reading datapoints used for threshold checking
def read_data(port, datatype):
	match datatype:
		case 'Temperature':
			return read_temperature(port)
		case 'Humidity':
			return read_humidity(port)
		case 'Thrust':
			return read_thrust(port)
		case default:
			print('Invalid datatype selected')

# Reading temperature from datalogging PCB
def read_temperature(port):
	ser = serial.Serial(port, 115200, timeout=COMM_TIMOUT)
	received_data = ser.read(7)

	for i in range(len(received_data)):
		if (received_data[i] == 0x99):
			AHT10_ADC_Raw = ((received_data[i+4] & 15) << 16) | (received_data[i+5] << 8) | received_data[i+6]
			AHT10_Temperature = (float(AHT10_ADC_Raw) * 200.00 / 1048576.00) - 50.00
			ser.close()
			return AHT10_Temperature

# Reading humidity from datalogging PCB
def read_humidity(port):
	ser = serial.Serial(port, 115200, timeout=COMM_TIMOUT)
	received_data = ser.read(7)

	for i in range(len(received_data)):
		if (received_data[i] == 0x99):

			AHT10_ADC_Raw = (received_data[i+2] << 12) | (received_data[i+3] << 4) | (received_data[i+4] >> 4)
			AHT10_Humidity = (float(AHT10_ADC_Raw)*100.00/1048576.00)
			ser.close()
			return AHT10_Humidity

# Reading thrust from datalogging PCB
def read_thrust(port):
	ser = serial.Serial(port, 115200, timeout=COMM_TIMOUT)
	received_data = ser.read(7)

	slope = -1.5617e-6
	intercept = 90.95
	jump_offset = 16000000

	for i in range(len(received_data)):
		if (received_data[i] == 0x9B):
			thrust = ((received_data[i+1] << 24) | (received_data[i+2] << 16) | (received_data[i+2] << 8) | received_data[i+2])
			if (thrust < 70000000 and thrust > 58300000) or (thrust < 50000000 and thrust > 41900000):
				thrust = thrust - jump_offset
			thrust = slope * thrust + intercept
			ser.close()
			return thrust