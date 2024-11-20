# SPS Test Rig Prototype Script for Pi modified for Final Demonstration
# Authors: Brandon Rule, Reuben Campbell

#KNOWN BUGS ****
#Software freezes when invalid device is selected for flight controller, such as datalogger
#Software cannot handle if datalogger is disconnected during operation

import tkinter
from tkinter import *
from tkinter import filedialog
from tkinter import messagebox
import subprocess
import csv
import matplotlib.pyplot as plt
#from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from random import *
from mavlink import *

import time
from time import *

from dataset import *
from databox import *
from dataplot import *

from External_PCB_Read import *

# Flight Controller class
mavlink = Mavlink()

root = Tk()
root.geometry("1280x720") # Dimension of main window 
root.title("Drone Propulsion Test Rig GUI - Post Final Demo")

#Main Header Label at top of window
titleLabel =  Label(root, text='SPS Automation Drone Test Rig Prototype V0.04 (Post Final Demo)')
titleLabel.pack()


#///////////////////////////////////////////////////////////////////////////
#
#                     Add External PCBs (COMPLETED)
#
#///////////////////////////////////////////////////////////////////////////

#A container that contains all the widgets for connecting to the flight controller selection
external_sensors = Frame(root)
external_sensors.place(relx=0.2, rely=0.22, height=200, width=210)

# Title for displaying PCB Data
testInputTitle =  Label(external_sensors, text='External Sensor Data')
testInputTitle.place(relx=0.5,rely=0.15,anchor="center")

temperatureTitle =  Label(external_sensors, text='Temperature: ')
temperatureTitle.place(relx=0,rely=0.25,anchor=W)
temperature = Label(external_sensors, text='0 °C')
temperature.place(relx=.45,rely=0.25,anchor=W)

humidityTitle =  Label(external_sensors, text='Humidity: ')
humidityTitle.place(relx=0,rely=0.35,anchor=W)
humidity = Label(external_sensors, text='0 %')
humidity.place(relx=0.35,rely=0.35,anchor=W)

thrustTitle =  Label(external_sensors, text='Thrust:')
thrustTitle.place(relx=0,rely=0.45,anchor=W)
thrust = Label(external_sensors, text='0 Kg')
thrust.place(relx=0.23,rely=0.45,anchor=W)

temperature_raw = 0
humidity_raw = 0
thrust_raw = 0


def external_sensors_update():
    global temperature_raw, humidity_raw, thrust_raw 
    temperature_raw = read_temperature('/dev/ttyACM1')
    humidity_raw = read_humidity('/dev/ttyACM1')
    thrust_raw = read_thrust('/dev/ttyACM1')

    text = "{temp:.2f} °C"
    if (temperature_raw):
        temperature.configure(text=text.format(temp=temperature_raw))
    else:
        temperature_raw = 0
    text = "{temp:.2f} %"
    if (humidity_raw):
        humidity.configure(text=text.format(temp=humidity_raw))
    else:
        humidity_raw = 0
    text = "{temp:.2f} Kg"
    if (thrust_raw):
        thrust.configure(text=text.format(temp=thrust_raw))
    else:
        thrust_raw = 0


#///////////////////////////////////////////////////////////////////////////
#
#                       Data Logging (NEEDS UPDATE)
#                       -Add all datapoints to data logging
#
#///////////////////////////////////////////////////////////////////////////

#UPDATE DATA LOGGING FOR LATEST VERSION

#Open CSV for datalogging
logFile = open("TestData.csv", "a")

#Button for starting and stopping datalogging (and flight test sequence)
dataLogging = False
def datalogButtonAction():
    global dataLogging
    global logFile
    if dataLogging:
        #Stop Data Logging
        logFile.close() #Close Log File
        mavlink.stop_test() #Stop test if running
        datalogButton.config(text="Start")
        dataLogging = False
    else:
        logFile = open("TestData.csv", "a")
        minSpeed = int(minSpeedInput.get(1.0, "end-1c"))
        maxSpeed = int(maxSpeedInput.get(1.0, "end-1c"))
        acceleration = int(accelerationInput.get(1.0, "end-1c"))
        mavlink.start_test(minSpeed, maxSpeed, acceleration)
        datalogButton.config(text="Stop")
        dataLogging = True
#datalogTitle = Label(root, text='Data Logging')
#datalogTitle.place(relx=0.1, rely=0.45, anchor=W)
#datalogButton = Button(text="Start", command=datalogButtonAction)
#datalogButton.place(relx=0.1, rely=0.475)

#Log Data
def logData():
    global dataLogging
    global logFile
    #if dataLogging:
        #print("Logging Data")\
        #logFile.write("{0},{1}\n".format(strftime("%Y-%m-%d %H:%M:%S"),str(mavlink.get_throttle())))
        #print(mavlink.get_throttle())

#///////////////////////////////////////////////////////////////////////////
#
#                              Test Input CSV (COMPLETED)
#
#///////////////////////////////////////////////////////////////////////////

#A container that contains all the widgets for connecting to the flight controller selection
test_input_Frame = Frame(root)
test_input_Frame.place(relx=0.4, rely=0.04, height=200, width=210)

#Test Parameter Text Input
testInputTitle =  Label(test_input_Frame, text='Test Sequence Control')
testInputTitle.place(relx=0.5,rely=0.08,anchor="center")

start_test_button = Button(test_input_Frame, text="Start Test", command=lambda:start_test())
start_test_button.place(relx=0.3, rely=0.25, anchor="center")

stop_test_button = Button(test_input_Frame, text="Stop Test", command=mavlink.stop_test)
stop_test_button.place(relx=0.7, rely=0.25, anchor="center")


#///////////////////////////////////////////////////////////////////////////
#
#                         Display Boxes (COMPLETED)
#
#///////////////////////////////////////////////////////////////////////////


dataset_manager = DataSetManager()

start_time = time.time()

def get_voltage():
    return time.time() - start_time, mavlink.get_esc_data()[0] / 100

def get_current():
    return time.time() - start_time, mavlink.get_esc_data()[1] / 10

def get_rpm():
    return time.time() - start_time, mavlink.get_esc_data()[2]

def get_temperature():
    return time.time() - start_time, mavlink.get_esc_data()[3]

def get_thrust():
    return time.time() - start_time, round(thrust_raw)

dataset_manager.add_dataset("Throttle", "%")

dataset_manager.add_dataset("Voltage", "V", get_voltage)
dataset_manager.add_dataset("Current", "A", get_current)
dataset_manager.add_dataset("RPM", "RPM", get_rpm)
dataset_manager.add_dataset("Temperature", "degC", get_temperature)

dataset_manager.add_dataset("Thrust", "Kg", get_thrust)

databox1 = DataBox(root, dataset_manager, 0.1, 0.2)
databox2 = DataBox(root, dataset_manager, 0.1, 0.4)
databox3 = DataBox(root, dataset_manager, 0.1, 0.6)
databox4 = DataBox(root, dataset_manager, 0.1, 0.8)

#Graph object
dataplot = DataPlot(root, dataset_manager, 0.65, 0.55)

for dataset in dataset_manager.datasets.values():
    dataplot.add_dataset(dataset)

throttle_values = []
bitrate = 10

def load_test_file():
    global throttle_values

    file_path = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv")])

    if file_path:
        throttle_values = []
        with open(file_path, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                throttle_values.append(int(float(row[0])))

        set_throttle_values()

load_test_file_button = Button(test_input_Frame, text="Load Test File", command=load_test_file)
load_test_file_button.place(relx=0.5, rely=0.4, anchor="center")

def set_throttle_values():
    global start_time, throttle_values

    if throttle_values is not None:
        dataset_manager.clear_datasets()
        throttle_dataset = dataset_manager.get_dataset("Throttle")

        start_time = time.time()

        interval = 1 / bitrate
        for i, value in enumerate(throttle_values):
            throttle_dataset.add_data(i * interval, value)

def start_test():
    set_throttle_values()
    mavlink.start_test(throttle_values, bitrate)

#///////////////////////////////////////////////////////////////////////////
#
#                     External PCBs Select & Add Datapoints (COMPLETED AND TESTED)
#                   - List of available COM Ports (COMPLETED)
#                   - User selects COM Port and variable (COMPLETED)
#                   - Try reading and if successful add to an array of variables to be displayed (COMPLETED)
#                   - If initial read failed, display a popup error (COMPLETED)
#                   - Add dropdown menu for adding variables from PCB (COMPLETED)
#
#///////////////////////////////////////////////////////////////////////////

# A container that contains all the widgets for adding external PCB datapoint
add_external_datapoint_Frame = Frame(root)
add_external_datapoint_Frame.place(relx=0.6, rely=0.04, height=125, width=210)

# Create Label for displaying title for Flight Control connection Frame
add_external_title = Label(add_external_datapoint_Frame , text = "External PCB - Add")
add_external_title.place(x=45, y=6)

#List of all external datapoints in the format [['COMPORT', 'DATATYPE'], [...], ...]
external_datapoints = []

external_datatype_exceptions = ['Select datatype']
datalogger_exceptions = ['Select external PCB', 'No serial devices']

# Action when 'Add'button is pressed
def add_datapoint():
    selected_datatype = external_datatype.get()
    selected_datalogger = datalogger.get()

    #Check if datatype and datalogger are valid
    if (selected_datatype not in external_datatype_exceptions):
        if (selected_datalogger not in datalogger_exceptions):
            # If the desired datatype can be read from the device then add it to list of external datapoints available
            read_datapoint = None
            match selected_datatype:
                case 'Temperature':
                    read_datapoint = read_temperature(selected_datalogger[0:12])
                case 'Humidity':
                    read_datapoint = read_humidity(selected_datalogger[0:12])
                case 'Thrust':
                    read_datapoint = read_thrust(selected_datalogger[0:12])
                case default:
                    print('Error: Somehow incorrect datatype selected for PCB')
                    return
            if (read_datapoint):
                external_datapoints.append([selected_datalogger.split()[0], selected_datatype])
                #print(external_datapoints)
                print("Adding datapoint succesful " + selected_datalogger + selected_datatype)
                update_datapoint_list()
            else:
                print("Adding datapoint unsuccesful "  + selected_datalogger + selected_datatype)
                messagebox.showerror("Error", "Adding datapoint failed")

# Button for adding datapoint to display
add_datapaoint_Button = Button(add_external_datapoint_Frame , text = "Add" , command = add_datapoint)
add_datapaoint_Button.place(x=10, y=30)

# datatype of dropdown text 
datalogger = StringVar()

# initial dropdown text 
datalogger.set("Select external PCB")

# Function to update the list of connected USART devices to Raspberry Pi, action when 'Refresh'is pressed
def external_pcb_list_update():
    try:
        # Gather the list of USART devices connected to Raspberry Pi
        device_list = subprocess.check_output("ls /dev/ttyACM*", shell=True, text=True).splitlines()
        for i in range(len(device_list)):
            command = "udevadm info -q all -n " + device_list[i]
            # Gather description of each USART device connected to Raspberry Pi
            raw_device_description = subprocess.check_output(command, shell=True, text=True)
            split_lines = raw_device_description.splitlines()
            for x in range(len(split_lines)):
                # Add device model string to each USART device in device_list to return
                if ("ID_MODEL=" in split_lines[x]):
                    device_list[i] += " - " + split_lines[x][12:]
    except subprocess.CalledProcessError:
        # Command to list serial devices fails because none available
        device_list = ["No serial devices"]
    datalogger.set("Select external PCB")
    # Recreate dropdown whenever the user click "Refresh"
    PCB_dropdown = OptionMenu(add_external_datapoint_Frame , datalogger , *device_list)
    PCB_dropdown.config(width=18)
    PCB_dropdown.place(x=9, y=60)
external_pcb_list_update() #Initialise datalogger PCB list

# Button to update list of USART devices connected to Raspberry Pi
pcb_list_refresh_Button = Button(add_external_datapoint_Frame , text = "Refresh" , command=external_pcb_list_update)
pcb_list_refresh_Button.place(x=118, y=30)

# datatype of dropdown text 
external_datatype = StringVar()

# initial dropdown text 
external_datatype.set("Select datatype")

# Currently supported Datatypes by PCBs
datatype_list = ["Temperature", "Humidity", "Thrust"]
external_datapoint_datatype_Dropdown = OptionMenu(add_external_datapoint_Frame , external_datatype , *datatype_list)
external_datapoint_datatype_Dropdown.config(width=18)
external_datapoint_datatype_Dropdown.place(x=9, y=90)

#///////////////////////////////////////////////////////////////////////////
#
#                 Variable Limit Initialise (COMPLETE)
#                 -Dropdown with list of variables available datapoints (Complete)
#                 -Min, Max threshold inputs (Complete)
#                 -Tickbox for either; display a warning and/or stoppping (Complete)
#                  motor/test sequence (Complete)
#                 -Can't remove a threshold but can overwrite
#                 -If the user is out of threshold range then popup is continous and kinda annoying
#
#///////////////////////////////////////////////////////////////////////////

#Threshold list, format: [[COMPORT, DATATYPE, WARNING/ERROR, MIN_THRESHOLD, MAX_THRESHOLD],...]
threshold_list = []

# A container that contains all the widgets for setting external PCB datapoint limits
set_limit_Frame = Frame(root)
set_limit_Frame.place(relx=0.8, rely=0.04, height=125, width=210)

# Create Label for displaying title for adding limit to datapoints
set_limit_Title = Label(set_limit_Frame , text = "Set Limits")
set_limit_Title.place(x=70, y=6)

# datatype of dropdown text
set_limit_dropdown_text = StringVar()

# initial dropdown text
set_limit_dropdown_text.set("Select datapoint")

#Initially there are no datapoints
current_datapoints = ['No datapoints']

set_limit_datatype_Dropdown = OptionMenu(set_limit_Frame , set_limit_dropdown_text , *current_datapoints)
set_limit_datatype_Dropdown.config(width=18)
set_limit_datatype_Dropdown.place(x=9, y=30)

warning_check = IntVar()
error_check = IntVar()

#Update the datapoint list whenever a datapoint is added
def update_datapoint_list():
    global current_datapoints
    current_datapoints = []
    for i in range(len(external_datapoints)):
        #remeber format for 'external datapoints' is ('COMPORT','DATATYPE')
        current_datapoint = external_datapoints[i][0] + ' - ' + external_datapoints[i][1]
        current_datapoints.append(current_datapoint)

    #Update List
    set_limit_datatype_Dropdown = OptionMenu(set_limit_Frame , set_limit_dropdown_text , *current_datapoints)
    set_limit_datatype_Dropdown.config(width=18)
    set_limit_datatype_Dropdown.place(x=9, y=30)

# Checkbutton for warning popup when threshold exceeded
warning_check_Button = Checkbutton(set_limit_Frame, text = "Warning", 
                    variable = warning_check, 
                    onvalue = 1, 
                    offvalue = 0, 
                    height = 1, 
                    width = 10)
warning_check_Button.place(x=0, y=62)

# Checkbutton for ending test when threshold exceeded
error_check_Button = Checkbutton(set_limit_Frame, text = "Error", 
                    variable = error_check, 
                    onvalue = 1, 
                    offvalue = 0, 
                    height = 1, 
                    width = 10)
error_check_Button.place(x=100, y=62)

# Set min limit
min_threshold_Label = Label(set_limit_Frame , text = "Min:")
min_threshold_Label.place(x=0, y=90)
min_threshold_Entry = Entry(set_limit_Frame, width=4)
min_threshold_Entry.place(x=30,y=90)

# Set max limit
max_threshold_Label = Label(set_limit_Frame , text = "Max:")
max_threshold_Label.place(x=70, y=90)
max_threshold_Entry = Entry(set_limit_Frame, width=4)
max_threshold_Entry.place(x=105,y=90)

def set_threshold():
    global threshold_list
    selected_datapoint = set_limit_dropdown_text.get()
    if (selected_datapoint not in ['No datapoints', 'Select datapoint']):

        #Make sure user enters valid input to max and min fields
        try:
            max_threshold = float(max_threshold_Entry.get())
            min_threshold = float(min_threshold_Entry.get())
        except:
            messagebox.showwarning('Warning','Invalid entry to min or max fields')
            return

        #Make sure user has selected either a warning or error checkbox
        if (error_check.get() == warning_check.get() == 0):
            messagebox.showwarning('Warning','Limit type not selected')
            return

        #Declaring limit type as either error or warning (error takes priority over warning)
        if (error_check.get() == 1):
            limit_type = 'Error'
        else:
            limit_type = 'Warning'

        # Make sure min threshold < max threshold
        if (min_threshold > max_threshold):
            messagebox.showwarning('Warning','min threshold > max_threshold')
            return

        #Check if threshold is already set and remove it before adding updated threshold
        threshold = [selected_datapoint.split()[0], selected_datapoint.split()[2], limit_type, min_threshold,max_threshold]
        for i in range(len(threshold_list)):
            if (threshold[0] in threshold_list[i]) and (threshold[1] in threshold_list[i]):
                #threshold is already set so remove it from the list
                threshold_list.pop(i)

        #Remember: [[COMPORT, DATATYPE, WARNING/ERROR, MIN_THRESHOLD, MAX_THRESHOLD],...]
        threshold_list.append([selected_datapoint.split()[0], selected_datapoint.split()[2], limit_type, min_threshold,max_threshold])
    else:
        messagebox.showwarning('Warning','Valid datapoint not selected')
        return
        #print(threshold_list)
        #print(max_threshold)
        #print(min_threshold)
        #print("threshold set")

#Function to check all thresholds and act accordingly
def check_thresholds():
    global threshold_list
    for i in range(len(threshold_list)):
        #Remember: [[COMPORT, DATATYPE, WARNING/ERROR, MIN_THRESHOLD, MAX_THRESHOLD],...]
        value = read_data(threshold_list[i][0], threshold_list[i][1])
        # Check if the value is out of range and display warning/error if so
        if (value < threshold_list[i][3]) or (value > threshold_list[i][4]):
            if threshold_list[i][2] == 'Error':
                messagebox.showerror('Error', threshold_list[i][0] + ' - ' + threshold_list[i][1] + ' out of range')
                #Stop test if running
                mavlink.stop_test()
            else:
                messagebox.showwarning('Warning', threshold_list[i][0] + ' - ' + threshold_list[i][1] + ' out of range')


# Set button
set_threshold_Button = Button(set_limit_Frame , text = "Set", command=set_threshold, height=1)
set_threshold_Button.place(x=150,y=85)

#///////////////////////////////////////////////////////////////////////////
#
#                 Flight Controller Selection Interface (COMPLETED)
#                 -Edge cases not accounted for:
#                 -Selected flight controller selected is invalid
#                 -Flight controller disconnected while running
#
#///////////////////////////////////////////////////////////////////////////

#A container that contains all the widgets for connecting to the flight controller selection
device_connect_Frame = Frame(root)
device_connect_Frame.place(relx=0.2, rely=0.04, height=125, width=210)

# Create Label for displaying title for Flight Control connection Frame
title = Label(device_connect_Frame , text = "Flight Controller - Select")
title.place(x=22, y=6)

flight_controller_connected = False #Initially disconnected
intialStates = ["Select flight controller", "No serial devices"]

# Action when connect / disconnect button is pressed
def show():
    global flight_controller_connected
    selected_device = clicked.get()
    if flight_controller_connected:
        flight_controller_connected = False
        connect_button.config(text="Connect")
        label.config(text="No device connected")
        mavlink.disconnect() #disconnect from flight controller
    elif selected_device not in intialStates:
        flight_controller_connected = True
        label.config(text=selected_device[15:])
        connect_button.config(text="Disconnect")
        mavlink.connect(selected_device[:12], 57600)
        minSpeed = 0

# Connect/disconnect button
connect_button = Button(device_connect_Frame , text = "Connect" , command = show)
connect_button.place(x=10, y=30)

# Function to update the list of connected USART devices to Raspberry Pi, action when 'Refresh'is pressed
def device_list_update():
    global device_list
    try:
        # Gather the list of USART devices connected to Raspberry Pi
        device_list = subprocess.check_output("ls /dev/ttyACM*", shell=True, text=True).splitlines()
        device_descriptions = []
        for i in range(len(device_list)):
            command = "udevadm info -q all -n " + device_list[i]
            # Gather description of each USART device connected to Raspberry Pi
            raw_device_description = subprocess.check_output(command, shell=True, text=True)
            split_lines = raw_device_description.splitlines()
            for x in range(len(split_lines)):
                # Add device model string to each USART device in device_list to return
                if ("ID_MODEL=" in split_lines[x]):
                    device_list[i] += " - " + split_lines[x][12:]
    except subprocess.CalledProcessError:
        device_list = ["No serial devices"]
    #print(clicked.get()[:20] + "...")
    #clicked.set(clicked.get()[:20] + "...")
    clicked.set("Select flight controller")
    connect_button.config(text="Connect")
    label.configure(text="No device connected")
    mavlink.disconnect() #disconnect from flight controller
    device_dropdown = OptionMenu(device_connect_Frame , clicked , *device_list)
    device_dropdown.config(width=18)
    device_dropdown.place(x=9, y=60)

# Button to update list of USART devices connected to Raspberry Pi
refresh_button = Button(device_connect_Frame , text = "Refresh" , command=device_list_update)
refresh_button.place(x=118, y=30)

# datatype of dropdown text 
clicked = StringVar()
  
# initial dropdown text 
clicked.set("Select flight controller")

# Create Label for displaying USART device description when 'connect' is pressed
label = Label(device_connect_Frame , text = "No device connected") 
label.place(x=105, y=110, anchor="center")

# Initialise list of serial devices
device_list_update()

#///////////////////////////////////////////////////////////////////////////
#
#                   Display datalogging PCB datapoints (INCOMPLETE)
#
#///////////////////////////////////////////////////////////////////////////


#///////////////////////////////////////////////////////////////////////////
#
#                               Main Loop
#
#///////////////////////////////////////////////////////////////////////////

while True:
    #General purpose databoxes
    dataset_manager.update_datasets()
    databox1.update_value()
    databox2.update_value()
    databox3.update_value()
    databox4.update_value()

    #Updating data capture from the external PCBs
    #external_sensors_update()

    #Log data to CSV file
    logData()
    #Update display

    check_thresholds()
    
    root.update()
    sleep(0.1)