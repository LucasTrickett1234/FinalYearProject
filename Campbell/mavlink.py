import time
import threading
from pymavlink import mavutil
import keyboard
from Campbell.mavlink_message import ESC_TELEMETRY

class Mavlink:
    """
    Mavlink: A class to manage communication with a MAVLink-based flight controller.

    This class establishes a connection to a MAVLink flight controller, receives and sends messages,
    manages data from the flight controller (e.g., ESC telemetry), and allows for tests like throttle control.

    Attributes:
        connected (bool): Flag indicating if the connection to the flight controller is active.
        connection (mavutil.mavlink_connection): The connection object to communicate with the flight controller.
        system_messages (dict): Dictionary of system-level message types (e.g., HEARTBEAT) and their handlers.
        data_messages (dict): Dictionary of data messages being tracked (e.g., ESC telemetry).
        message_thread (threading.Thread): Thread for continuously receiving messages.
        test_running (bool): Flag indicating if a test is currently running.
        test_thread (threading.Thread): Thread for running motor throttle tests.
        throttle (int): Current throttle value for motor tests.
    """

    def __init__(self):
        """
        Initializes the Mavlink instance and sets up default attributes.
        """
        self.connected = False
        self.connection = None

        self.system_messages = {
                "HEARTBEAT": self._recieve_heartbeat,
                "COMMAND_ACK": self._recieve_command_ack
            }
        self.data_messages = {}
        self.message_thread = None

        self.test_running = False
        self.test_thread = None

        self.throttle = 0

        self.add_message(ESC_TELEMETRY)

    def connect(self, port, baud_rate=57600):
        """
        Connects to the flight controller via the specified port and baud rate.

        Args:
            port (str): The serial port to connect to (e.g., "COM5").
            baud_rate (int, optional): Baud rate for the connection. Defaults to 57600.

        Raises:
            Exception: If unable to connect to the flight controller.
        """
        if self.connected:
            print("MAVLINK: Already connected to flight controller")
            return

        print(f"MAVLINK: Connecting to flight controller on {port}, {baud_rate}")

        try:
            self.connection = mavutil.mavlink_connection(port, baud_rate=baud_rate, dialect="ardupilotmega")
            self.connection.wait_heartbeat()

            self.connected = True

            self.message_thread = threading.Thread(target=self._message_loop)
            self.message_thread.start()

            self.set_message_rate(ESC_TELEMETRY, 50)
            self.connection.arducopter_arm()

            print("MAVLINK: Connected to flight controller")
        except Exception as e:
            print(e)

    def disconnect(self):
        """
        Disconnects from the flight controller.
        """
        if not self.connected:
            print("MAVLINK: Not connected to flight controller")
            return

        self.connected = False

        if self.message_thread is not None:
            self.message_thread.join()

        if self.connection is not None:
            self.connection.close()

        print("MAVLINK: Disconnected from flight controller")

    def _message_loop(self):
        """
        Continuously receives messages from the flight controller while connected.
        Routes received messages to the appropriate handler.
        """
        while self.connected:
            try:
                msg = self.connection.recv_match(blocking=True)
                if msg is not None:
                    msg_type = msg.get_type()
                    if msg_type in self.system_messages:
                        self.system_messages[msg_type](msg)
                    elif msg_type in self.data_messages:
                        self._receive_data(msg)
            except Exception as e:
                print(e)

    def _recieve_heartbeat(self, msg):
        """
        Handles received HEARTBEAT messages from the flight controller.
        """
        pass

    def _recieve_command_ack(self, msg):
        """
        Handles COMMAND_ACK messages, indicating if a command was accepted or rejected.
        """
        if msg.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"MAVLINK: Command Accepted: {msg}")
            else:
                print(f"MAVLINK: Command Failed: {msg}")

    def _receive_data(self, msg):
        """
        Handles received data messages and stores them in the appropriate message's data fields.

        Args:
            msg (mavutil.mavlink.MAVLink_message): The received data message.
        """
        message = self.data_messages[msg.get_type()]
        for field in message.fields:
            if hasattr(msg, field):
                message.data[field] = getattr(msg, field)
    
    def add_message(self, message):
        """
        Adds a message type to track for receiving data.

        Args:
            message (MavlinkMessage): The message type to add (e.g., ESC telemetry).
        """
        self.data_messages[message.type] = message
    
    def set_message_interval(self, message, interval):
        """
        Sets the interval at which a message should be sent from the flight controller.

        Args:
            message (MavlinkMessage): The message type to set an interval for.
            interval (int): The interval in microseconds.
        """
        print(f"MAVLINK: Setting {message.type} ({message.id}) message interval to: {interval} us")

        try:
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message.id,
                interval,
                0, 0, 0, 0, 0
            )
        except Exception as e:
            print(e)

    def set_message_rate(self, message, rate):
        """
        Sets the rate at which a message should be sent from the flight controller.

        Args:
            message (MavlinkMessage): The message type to set a rate for.
            rate (int): The rate in Hz. Setting rate to 0 disables the message.
        """
        if rate <= 0:
            interval = 0
        else:
            interval = 1e6 / rate

        self.set_message_interval(message, interval)

    def request_message(self, message):
        """
        Requests a single instance of a message from the flight controller.

        Args:
            message (MavlinkMessage): The message type to request (e.g., ESC telemetry).
        """
        print(f"MAVLINK: Requesting {message.type} ({message.id}) message")

        try:
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
                message.id,
                0, 0, 0, 0, 0, 0
            )
        except Exception as e:
            print(e)

    def set_throttle(self, throttle, time):
        """
        Sets the motor throttle for a given duration (motor test mode).

        Args:
            throttle (int): The throttle value (percentage).
            time (float): The duration to maintain the throttle.
        """
        try:
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                0, 3, 0, throttle, time, 0, 0, 0
            )
        except Exception as e:
            print(e)

    def start_test(self, throttle_values, bitrate):
        """
        Starts a motor throttle test with varying throttle values.

        Args:
            throttle_values (list): List of throttle values to apply.
            bitrate (float): The rate at which throttle values are applied (in Hz).
        """
        if self.test_running:
            print("MAVLINK: Cannot start test: Test already in progress")
            return
        
        self.test_running = True
        self.test_thread = threading.Thread(target=self._run_test, args=(throttle_values, bitrate))
        self.test_thread.start()

    def _run_test(self, throttle_values, bitrate):
        """
        Handles running a motor throttle test by applying throttle values at the given bitrate.

        Args:
            throttle_values (list): List of throttle values to apply.
            bitrate (float): The rate at which throttle values are applied (in Hz).
        """
        print("MAVLINK: Starting test")
        
        period = 1 / bitrate
        length = len(throttle_values)
        i = 0

        while self.test_running and i < length:
            self.throttle = throttle_values[i]
            self.set_throttle(self.throttle, period)
            i += 1
            time.sleep(period)
        
        self.test_running = False
        self.throttle = 0
        self.set_throttle(self.throttle, 1)

        print("MAVLINK: Test complete")

    def stop_test(self):
        """
        Stops the currently running motor throttle test.
        """
        if not self.test_running:
            print("MAVLINK: Cannot stop test: No test in progress")
            return
        
        self.test_running = False
        self.throttle = 0
        self.set_throttle(self.throttle, 1)

        if self.test_thread is not None:
            self.test_thread.join()

    def get_throttle(self):
        """
        Returns the current throttle value.

        Returns:
            int: The current throttle value.
        """
        return self.throttle
    
    def get_esc_data(self):
        """
        Returns the most recent ESC telemetry data.

        Returns:
            tuple: Contains the voltage, current, RPM, and temperature from the ESC telemetry.
                   Returns zeros if no data is available.
        """
        i = 1

        voltage = ESC_TELEMETRY.data["voltage"]
        current = ESC_TELEMETRY.data["current"]
        rpm = ESC_TELEMETRY.data["rpm"]
        temperature = ESC_TELEMETRY.data["temperature"]

        if voltage and current and rpm and temperature is not None:
            return voltage[i], current[i], rpm[i], temperature[i]
        else:
            return 0, 0, 0, 0


def main():
    mavlink = Mavlink()
    port = "COM5"

    # Print outputs
    output = True

    def print_outputs():
        while True:
            time.sleep(1)
            if output:
                print(f"ESC: {mavlink.get_esc_data()}")

    threading.Thread(target=print_outputs).start()

    # Controls
    key_states = {'0': False, '1': False, '2': False, '3': False, '4': False, '5': False, '9': False}

    while True:
        time.sleep(0.01)

        if keyboard.is_pressed('1'):
            if not key_states['1']:
                mavlink.connect(port)
                key_states['1'] = True
        else:
            key_states['1'] = False

        if keyboard.is_pressed('2'):
            if not key_states['2']:
                output = not output
                key_states['2'] = True
        else:
            key_states['2'] = False

        if keyboard.is_pressed('3'):
            if not key_states['3'] and mavlink.connected:
                mavlink.set_message_rate(ESC_TELEMETRY, 1)
                key_states['3'] = True
        else:
            key_states['3'] = False

        if keyboard.is_pressed('4'):
            if not key_states['4'] and mavlink.connected:
                mavlink.set_message_rate(ESC_TELEMETRY, 0)
                key_states['4'] = True
        else:
            key_states['4'] = False

        if keyboard.is_pressed('5'):
            if not key_states['5'] and mavlink.connected:
                mavlink.request_message(ESC_TELEMETRY)
                key_states['5'] = True
        else:
            key_states['5'] = False

        if keyboard.is_pressed('9'):
            if not key_states['9'] and mavlink.connected:
                throttle_values = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
                mavlink.start_test(throttle_values, 1)
                key_states['9'] = True
        else:
            key_states['9'] = False

        if keyboard.is_pressed('0'):
            if not key_states['0'] and mavlink.connected:
                mavlink.stop_test()
                key_states['0'] = True
        else:
            key_states['0'] = False


if __name__ == "__main__":
    main()

