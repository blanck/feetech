import serial
import time

# Control table addresses (only those used in the code)
ADDR_PING = 0  # Address for ping command
ADDR_MODEL_NUMBER = 3  # Address for model number
ADDR_SERVO_ID = 5 # Address for servo ID
ADDR_SCS_TORQUE_ENABLE = 40
ADDR_SCS_GOAL_POSITION = 42
ADDR_SCS_GOAL_SPEED    = 46
ADDR_SCS_LOCK = 55 # Address for EPROM lock
ADDR_SCS_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED = 58
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
        enable_response = self._send_packet(servo_id, instruction=0x03, address=ADDR_SCS_TORQUE_ENABLE, data=data)
        return enable if enable_response is not None else None

    def write_position(self, servo_id, position):
        """
        Set the goal position of the servo.
        """
        data = [position & 0xFF, (position >> 8) & 0xFF]  # Convert position to 2 bytes (little-endian)
        position_result = self._send_packet(servo_id, instruction=0x03, address=ADDR_SCS_GOAL_POSITION, data=data)
        return position if position_result is not None else None

    def _write_byte(self, servo_id, address, value):
        """
        Helper to write a single byte to the control table.
        """
        return self._send_packet(servo_id, instruction=0x03, address=address, data=[value & 0xFF])

    def _read_byte(self, servo_id, address):
        """
        Helper to read a single byte from the control table.
        """
        return self._send_packet(servo_id, instruction=0x02, address=address, read_length=1)

    def set_new_id(self, servo_id, new_servo_id):
        """
        Change the servo ID and persist it in EEPROM
        """
        if servo_id == new_servo_id:
            if self.debug:
                print("New ID matches the current ID; nothing to change.")
            return True

        if servo_id in (0xFE, 0xFF) or new_servo_id in (0xFE, 0xFF):
            raise ValueError("Broadcast IDs (254/255) cannot be assigned to a single servo.")

        if not (0 <= servo_id <= 253 and 0 <= new_servo_id <= 253):
            raise ValueError("Servo IDs must be in the range 0-253.")

        if self.ping(servo_id) == -1:
            if self.debug:
                print(f"Servo {servo_id} did not respond to ping; aborting ID change.")
            return False

        # Disable torque before touching EEPROM
        torque_disabled = False
        torque_state = self._write_byte(servo_id, ADDR_SCS_TORQUE_ENABLE, 0)
        if torque_state is not None:
            torque_disabled = True
            if self.debug:
                print("Torque disabled before EEPROM access.")
            time.sleep(0.05)

        if self.debug:
            print(f"Unlocking EEPROM on ID {servo_id}...")
        if self._write_byte(servo_id, ADDR_SCS_LOCK, 0) is None:
            if self.debug:
                print("Failed to unlock EEPROM.")
            if torque_disabled:
                self._write_byte(servo_id, ADDR_SCS_TORQUE_ENABLE, 1)
            return False

        if self.debug:
            print(f"Writing new ID {new_servo_id}...")
        if self._write_byte(servo_id, ADDR_SERVO_ID, new_servo_id) is None:
            self._write_byte(servo_id, ADDR_SCS_LOCK, 1)
            if self.debug:
                print("Failed to write new ID; EEPROM relocked with original ID.")
            return False

        time.sleep(0.15)  # Give the servo time to flush the EEPROM write.

        if self.debug:
            print(f"Locking EEPROM using new ID {new_servo_id}...")
        if self._write_byte(new_servo_id, ADDR_SCS_LOCK, 1) is None:
            if self.debug:
                print("Failed to re-lock EEPROM; ID change may not persist.")
            return False

        # Verify by pinging and reading back address
        if self.ping(new_servo_id) == -1:
            if self.debug:
                print(f"Servo did not respond on new ID {new_servo_id}.")
            return False

        stored_id = self._read_byte(new_servo_id, ADDR_SERVO_ID)
        if stored_id != new_servo_id:
            if self.debug:
                print(f"EEPROM verification failed: read {stored_id}, expected {new_servo_id}.")
            if torque_disabled:
                self._write_byte(new_servo_id, ADDR_SCS_TORQUE_ENABLE, 1)
            return False

        if torque_disabled:
            self._write_byte(new_servo_id, ADDR_SCS_TORQUE_ENABLE, 1)
            if self.debug:
                print("Torque re-enabled after EEPROM write.")

        if self.debug:
            print(f"Successfully changed ID from {servo_id} to {new_servo_id}.")
        return True

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
    
    def set_speed(self, servo_id, speed):
        """
        Set the goal speed of the servo.
        """
        data = [speed & 0xFF, (speed >> 8) & 0xFF]  # Convert speed to 2 bytes (little-endian)
        speed_result = self._send_packet(servo_id, instruction=0x03, address=ADDR_SCS_GOAL_SPEED, data=data)
        return speed if speed_result is not None else None

    def read_speed(self, servo_id):
        """
        Read the current speed of the servo.
        """
        return self._send_packet(servo_id, instruction=0x02, address=ADDR_PRESENT_SPEED, read_length=2)

    def close(self):
        """
        Close the serial connection.
        """
        self.ser.close()


if __name__ == "__main__":
    # Example usage with debug enabled, baud rate 115200 is the default for Feetech servos but hls uses 1_000_000
    servo = FeetechServo(port='/dev/cu.wchusbserial1120', baud_rate=115200, debug=False)

    try:
        # Set the servo ID and the new ID if you want to change it
        servo_id = 1
        new_servo_id = None # Set to None to not change the ID

        # Read model number
        model_number = servo.read_model_number(servo_id=servo_id)
        print(f"Model number: {model_number}")

        # Enable torque
        result = servo.enable_torque(servo_id=servo_id, enable=True)
        print(f"Enabled torque: {result}")

        # Set speed to 1000
        result = servo.set_speed(servo_id=servo_id, speed=1000)
        print(f"Set speed: {result}")

        # Move to position 1500
        import random
        pos = random.randint(0, 4096)
        result = servo.write_position(servo_id=servo_id, position=pos)
        print(f"Move to position: {result}")

        # Read current position
        position = servo.read_position(servo_id=servo_id)
        print(f"Current position: {position}")

        # Read load
        load = servo.read_load(servo_id=servo_id)
        print(f"Current load: {load}")

        # Read voltage
        voltage = servo.read_voltage(servo_id=servo_id)
        if voltage:
            print(f"Current voltage: {voltage * 0.1} V")  # Voltage is returned in 0.1V units
        else:
            print("Failed to read voltage")

        if new_servo_id:
            servo.set_new_id(servo_id=servo_id, new_servo_id=new_servo_id)
            print(f"Set new ID: {servo_id} -> {new_servo_id}")

        # Read temperature
        temperature = servo.read_temperature(servo_id=servo_id)
        print(f"Current temperature: {temperature} °C")

    finally:
        # Close the connection
        servo.close()
