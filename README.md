# Feetech Servo Python Driver

This is an **unofficial Python driver** for controlling Feetech serial bus servos over RS485 using a USB-to-TTL adapter. It was put together for our own use since other libraries and code examples were too complicated, involving multiple files and folders, and were just very hard to implement compared to a single-file class with logical method names.

Feel free to use, suggest missing features, or commit improvements!

## Features

- Control multiple servos over RS485.
- Read and write servo parameters such as position, speed, load, voltage, and temperature.
- Enable/disable torque control.
- Ping servos to check connectivity.
- Fetch model number and other servo information.

## Compatibility

- **Tested Hardware**:
  - **USB-to-TTL Adapter**: Feetech FE-URT-1.
  - **USB-to-TTL Adapter**: Waveshare Bus Servo Adapter (A).
  - **Servo Motor**: Feetech SM45BL (24V, 45kgcm, RS485 serial bus).
  - **Servo Motor**: Waveshare ST3215 (12V, 30kgcm, TTL serial bus).
  - **Servo Motor**: Feetech HLS3935M (12V, 35kgcm, TTL serial bus)
- **Likely Compatible**: Other Feetech RS485 serial bus servos.

## Installation

### Option 1: Clone the Repository

1. Clone this repository:

   ```bash
   git clone https://github.com/blanck/feetech.git
   cd feetech
   ```

2. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

### Option 2: Install Manually

If you don’t want to clone the repository, you can simply install the required dependency manually:

```bash
pip install pyserial
```

### Connect Your Hardware

- Connect the Feetech FE-URT-1 or Waveshare USB-to-TTL adapter to your computer.
- Connect the RS485/TTL servo(s) to the adapter.
- Ensure the servo ID(s) are properly configured.

## Usage

### Basic Example

```python
from feetech import FeetechServo

# Initialize the servo driver
servo = FeetechServo(port='/dev/ttyUSB0', baud_rate=115200, debug=True)

try:
    # Ping the servo
    ping_response = servo.ping(servo_id=1)
    print(f"Ping response: {'Alive' if ping_response == 1 else 'No response'}")

    # Read the model number
    model_number = servo.read_model_number(servo_id=1)
    print(f"Model number: {model_number}")

    # Enable torque
    servo.enable_torque(servo_id=1, enable=True)

    # Move to position 1500
    servo.write_position(servo_id=1, position=1500)

    # Read current position
    position = servo.read_position(servo_id=1)
    print(f"Current position: {position}")

    # Read load, voltage, and temperature
    load = servo.read_load(servo_id=1)
    voltage = servo.read_voltage(servo_id=1)
    temperature = servo.read_temperature(servo_id=1)
    print(f"Load: {load}, Voltage: {voltage * 0.1} V, Temperature: {temperature} °C")

finally:
    # Close the connection
    servo.close()
```

### Methods

#### Initialize the Driver

```python
servo = FeetechServo(port='/dev/ttyUSB0', baud_rate=115200, debug=False)
```

- `port`: Serial port (e.g., `/dev/ttyUSB0` on Linux or `COM3` on Windows).
- `baud_rate`: Baud rate (default is `115200`, or `1_000_000` for Waveshare Bus Servo Adapter).
- `debug`: Enable debug output (default is `False`).

#### Ping a Servo

```python
ping_response = servo.ping(servo_id=1)
```

- Returns `1` if the servo is alive, `-1` if no response.

#### Read Model Number

```python
model_number = servo.read_model_number(servo_id=1)
```

- Returns the model number of the servo.

#### Enable/Disable Torque

```python
servo.enable_torque(servo_id=1, enable=True)
```

- `enable`: `True` to enable torque, `False` to disable.

#### Set Goal Position

```python
servo.write_position(servo_id=1, position=1500)
```

- `position`: Target position (0–4095 or as per servo limits).

#### Set Goal Speed

```python
servo.set_speed(servo_id=1, speed=1500)
```

- `speed`: Target speed (0–4095 or as per servo limits).

#### Set New Servo ID

```python
servo.set_new_id(servo_id=1, new_servo_id=2)
```

- `servo_id`: Current ID of the servo.
- `new_servo_id`: New ID to assign (0–253, cannot be 254 or 255).
- Returns `True` on success, `False` on failure. The new ID is persisted in EEPROM.

#### Read Current Position

```python
position = servo.read_position(servo_id=1)
```

- Returns the current position of the servo.

#### Read Load

```python
load = servo.read_load(servo_id=1)
```

- Returns the current load on the servo.

#### Read Voltage

```python
voltage = servo.read_voltage(servo_id=1)
```

- Returns the current voltage in 0.1V units. Multiply by `0.1` to get volts.

#### Read Temperature

```python
temperature = servo.read_temperature(servo_id=1)
```

- Returns the current temperature in °C.

#### Close Connection

```python
servo.close()
```

- Closes the serial connection.

## Documentation

### Debug Mode

Enable debug mode during initialization to print packet and response details:

```python
servo = FeetechServo(port='/dev/ttyUSB0', baud_rate=115200, debug=True)
```

## Compatibility Notes

- This driver is **unofficial** and has been tested with the **Feetech SM45BL** servo and **FE-URT-1 USB-to-TTL adapter**.
- It is likely compatible with other Feetech RS485 servos, but you may need to adjust control table addresses or packet formats.

## Contributing

Contributions are welcome! If you find any issues or have suggestions for improvements, please open an issue or submit a pull request.

## License

This project is licensed under the MIT License.
