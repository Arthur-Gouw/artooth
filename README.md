# Alfred - Mecanum Line Follower Robot

A mecanum-wheeled robot with IR-based line following. The ESP32-S3 handles motor control and IR sensor reading, while a Raspberry Pi runs the high-level control logic (manual driving + autonomous line following) over UART.

## Project Structure

- `src/main.cpp` - ESP32-S3 firmware (PlatformIO/Arduino). Controls 4 mecanum motors, reads 7 IR sensors, and communicates with the Raspberry Pi via UART.
- `Minilab5/linefollower.py` - Raspberry Pi Python script. Provides a pygame GUI with manual (WASD+QE) and automatic (FSM line-following) modes.

## Hardware

- **MCU:** ESP32-S3-DevKitM-1
- **SBC:** Raspberry Pi (connected via UART at 115200 baud)
- **Motors:** 4x mecanum wheels (A/B = front left/right, C/D = rear left/right)
- **Sensors:** 7-channel IR array (3 used for line following: left, middle, right)

## Setup

### ESP32 Firmware

1. Install [PlatformIO](https://platformio.org/)
2. Open this project folder
3. Build and upload:
   ```
   pio run --target upload
   ```

### Raspberry Pi

1. Install dependencies:
   ```
   pip install pyserial pygame
   ```
2. Run the controller:
   ```
   python Minilab5/linefollower.py
   ```

## Controls (Raspberry Pi GUI)

| Key | Action |
|-----|--------|
| W/S | Forward / Reverse |
| A/D | Strafe left / right (manual only) |
| Q/E | Tank rotate left / right |
| M | Toggle auto (line following) / manual mode |
| 1/2 | Decrease / increase speed |
| Space | Stop |
| Esc | Quit |
