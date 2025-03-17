import serial
import time

# Control table addresses (only those used in the code)
ADDR_PING = 0  # Address for ping command
ADDR_MODEL_NUMBER = 3  # Address for model number
ADDR_SCS_TORQUE_ENABLE = 40
ADDR_SCS_GOAL_POSITION = 42
ADDR_SCS_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63

class FeetechServo:
    def __init__(self, port, baud_rate=115200, debug=False):
        """
        Initialize the servo with the specified serial port and baud rate.
        """
        self.ser = serial.Serial(port, baud_rate, timeout=1)
        self.debug = debug  # Enable/disable debug output

    def _send_packet(self, servo_id, instruction, address, data=None, read_length=0):
        """
        Send a packet to the servo and return the response (if any).
        """
        # Clear the serial buffer
        self.ser.reset_input_buffer()

        if data is not None:
            # Write command
            packet = [
                0xFF, 0xFF,  # Header
                servo_id,     # Servo ID (1 byte)
                3 + len(data),  # Length (1 byte) = Instruction (1) + Address (1) + Data (n)
                instruction,  # Instruction (1 byte)
                address,      # Address (1 byte)
                *data         # Data (n bytes)
            ]
        else:
            # Read command
            packet = [
                0xFF, 0xFF,  # Header
                servo_id,     # Servo ID (1 byte)
                4,            # Length (1 byte) = Instruction (1) + Address (1) + Length (1)
                instruction,  # Instruction (1 byte)
                address,      # Address (1 byte)
                read_length   # Length of data to read (1 byte)
            ]

        # Calculate checksum (excluding header)
        checksum = ~sum(packet[2:]) & 0xFF
        packet.append(checksum)  # Add checksum

        # Print the packet for debugging
        if self.debug:
            print(f"Sent packet: {[hex(x) for x in packet]}")

        # Send the packet
        self.ser.write(bytes(packet))

        # Wait for the servo to respond (if it's a read or ping command)
        start_time = time.time()
        while time.time() - start_time < 1:  # Wait up to 1 second for a response
            if self.ser.in_waiting >= 6 + read_length:
                response = self.ser.read(6 + read_length)  # Header (2) + ID (1) + Length (1) + Error (1) + Data (read_length) + Checksum (1)
                if self.debug:
                    print(f"Response: {[hex(x) for x in response]}")
                if len(response) >= 6 + read_length:
                    # Extract data bytes (little-endian)
                    data_bytes = response[5:5 + read_length]
                    return int.from_bytes(data_bytes, byteorder='little')
            time.sleep(0.01)  # Small delay to avoid busy-waiting
        return None

    def ping(self, servo_id):
        """
        Ping the servo to check if it's alive.
        Returns the ping response (1 if alive, -1 if no response).
        """
        ping_response = self._send_packet(servo_id, instruction=0x01, address=ADDR_PING, data=None, read_length=0)
        return 1 if ping_response is not None else -1

    def read_model_number(self, servo_id):
        """
        Read the model number of the servo.
        """
        return self._send_packet(servo_id, instruction=0x02, address=ADDR_MODEL_NUMBER, read_length=2)

    def enable_torque(self, servo_id, enable=True):
        """
        Enable or disable torque on the servo.
        """
        data = [1 if enable else 0]  # 1 to enable, 0 to disable
        return self._send_packet(servo_id, instruction=0x03, address=ADDR_SCS_TORQUE_ENABLE, data=data)

    def write_position(self, servo_id, position):
        """
        Set the goal position of the servo.
        """
        data = [position & 0xFF, (position >> 8) & 0xFF]  # Convert position to 2 bytes (little-endian)
        return self._send_packet(servo_id, instruction=0x03, address=ADDR_SCS_GOAL_POSITION, data=data)

    def read_position(self, servo_id):
        """
        Read the current position of the servo.
        """
        return self._send_packet(servo_id, instruction=0x02, address=ADDR_SCS_PRESENT_POSITION, read_length=2)

    def read_load(self, servo_id):
        """
        Read the current load on the servo.
        """
        return self._send_packet(servo_id, instruction=0x02, address=ADDR_PRESENT_LOAD, read_length=2)

    def read_voltage(self, servo_id):
        """
        Read the current voltage of the servo.
        """
        return self._send_packet(servo_id, instruction=0x02, address=ADDR_PRESENT_VOLTAGE, read_length=1)

    def read_temperature(self, servo_id):
        """
        Read the current temperature of the servo.
        """
        return self._send_packet(servo_id, instruction=0x02, address=ADDR_PRESENT_TEMPERATURE, read_length=1)

    def close(self):
        """
        Close the serial connection.
        """
        self.ser.close()


if __name__ == "__main__":
    # Example usage with debug enabled
    servo = FeetechServo(port='/dev/cu.wchusbserial1120', baud_rate=115200, debug=True)

    try:
        # Ping the servo
        ping_response = servo.ping(servo_id=1)
        print(f"Ping response: {'Alive' if ping_response == 1 else 'No response'}")

        # Read model number
        model_number = servo.read_model_number(servo_id=1)
        print(f"Model number: {model_number}")

        # Enable torque
        result = servo.enable_torque(servo_id=1, enable=True)
        print(f"Enabled torque: {result}")

        # Move to position 1500
        result = servo.write_position(servo_id=1, position=1500)
        print(f"Move to position: {result}")

        # Read current position
        position = servo.read_position(servo_id=1)
        print(f"Current position: {position}")

        # Read load
        load = servo.read_load(servo_id=1)
        print(f"Current load: {load}")

        # Read voltage
        voltage = servo.read_voltage(servo_id=1)
        print(f"Current voltage: {voltage * 0.1} V")  # Voltage is returned in 0.1V units

        # Read temperature
        temperature = servo.read_temperature(servo_id=1)
        print(f"Current temperature: {temperature} °C")

    finally:
        # Close the connection
        servo.close()