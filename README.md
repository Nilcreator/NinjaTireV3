# Build Your Own AI Ninja Robot (Raspberry Pi, Gemini, LCD Face, ToF Sensor)



This comprehensive tutorial guides you through building and programming an advanced, voice-controlled robot based on a Raspberry Pi Zero 2W. This upgraded version features a VL6180X Time-of-Flight distance sensor for accurate obstacle detection and a 2-inch SPI LCD that gives the robot an expressive face.

The robot integrates movement, sound, vision, and AI, all controllable through a web interface or by voice using Google's Gemini AI.

---

## **Table of Contents**

*   [1. Hardware Requirements](#1-hardware-requirements)
*   [2. Hardware Assembly](#2-hardware-assembly)
*   [3. GPIO Pinout Configuration](#3-gpio-pinout-configuration)
*   [4. Software Setup on Raspberry Pi](#4-software-setup-on-raspberry-pi)
*   [5. Code Implementation](#5-code-implementation)
*   [6. Running the Application](#6-running-the-application)
*   [7. Using the Interface](#7-using-the-interface)
*   [8. Troubleshooting](#8-troubleshooting)

---

## **1. Hardware Requirements**

This project uses specific components that are designed to work together.

*   **Core:**
    *   Raspberry Pi Zero 2 W
    *   DFRobot IO Expansion HAT for Pi Zero
    *   DFRobot UPS HAT for Raspberry Pi Zero
    *   16GB+ Micro SD Card (Class 10)
*   **Sensors & Display:**
    *   **[NEW]** **Waveshare 2-inch SPI LCD Module (240x320, ST7789V driver):** This will be the robot's face.
    *   **[NEW]** **VL6180X Distance Sensor (I2C):** This replaces the older ultrasonic sensor.
*   **Actuators & Audio:**
    *   SG90 180° Servos (x2)
    *   SG90 360° Continuous Rotation Servos (x2)
    *   Active Buzzer Module
    *   (Optional) INMP441 I2S Microphone Module
*   **Miscellaneous:**
    *   Robot Chassis/Frame
    *   Jumper Wires (Female-to-Female)
    *   5V, >= 2.5A Micro USB Power Supply

---

## **2. Hardware Assembly**

**⚠️ IMPORTANT:** Always disconnect the power supply before connecting or disconnecting any components!

1.  **Stack the HATs:**
    *   First, mount the **DFRobot UPS HAT** onto the Raspberry Pi Zero's 40-pin GPIO header.
    *   Next, carefully stack the **DFRobot IO Expansion HAT** on top of the UPS HAT. Ensure all pins are aligned and press down firmly.

2.  **Connect the VL6180X Distance Sensor (I2C):**
    *   On the IO Expansion HAT, locate one of the **I2C** ports.
    *   Connect the sensor using jumper wires:
        *   Sensor `VCC` -> HAT `3.3V`
        *   Sensor `GND` -> HAT `GND`
        *   Sensor `SCL` -> HAT `SCL`
        *   Sensor `SDA` -> HAT `SDA`

3.  **Connect the Waveshare 2-inch LCD (SPI):**
    *   This requires connecting to both the dedicated SPI pins and some general-purpose GPIO pins on the HAT.
    *   Connect the LCD using jumper wires:
        *   LCD `VCC` -> HAT `3.3V`
        *   LCD `GND` -> HAT `GND`
        *   LCD `DIN` (MOSI) -> HAT `SPI_MOSI` (GPIO 10)
        *   LCD `CLK` (SCLK) -> HAT `SPI_SCLK` (GPIO 11)
        *   LCD `CS` (Chip Select) -> HAT `SPI_CS0` (GPIO 8)
        *   LCD `DC` (Data/Command) -> HAT `D25` (GPIO 25)
        *   LCD `RST` (Reset) -> HAT `D27` (GPIO 27)
        *   LCD `BL` (Backlight) -> HAT `D24` (GPIO 24)

4.  **Connect Servos:**
    *   Connect the four servos to the dedicated servo ports `S0` - `S3` on the IO Expansion HAT.
    *   Ensure correct polarity: Signal (Yellow/Orange), VCC (Red), GND (Brown/Black).
    *   **Default Configuration:**
        *   `S0`: Right Leg/Hip (180° SG90)
        *   `S1`: Left Leg/Hip (180° SG90)
        *   `S2`: Right Foot/Wheel (360° SG90)
        *   `S3`: Left Foot/Wheel (360° SG90)

5.  **Connect Buzzer:**
    *   Connect the buzzer module to the HAT's GPIO breakout pins.
    *   `Signal/IO` -> HAT `D23` (GPIO 23)
    *   `VCC/+` -> HAT `3.3V`
    *   `GND/-` -> HAT `GND`

6.  **(Optional) Connect I2S Microphone:**
    *   `VDD` -> HAT `3.3V`
    *   `GND` -> HAT `GND`
    *   `SCK` -> HAT `D18` (GPIO 18)
    *   `WS` -> HAT `D19` (GPIO 19)
    *   `SD` -> HAT `D20` (GPIO 20)
    *   `L/R` -> `GND`

7.  **Final Assembly:** Mount all components onto your robot chassis. Organize wiring carefully to prevent it from getting caught during movement.

---

## **3. GPIO Pinout Configuration**

This table summarizes the final, conflict-free pinout for the upgraded robot.

| Component | Pin Function | HAT Label | RPi GPIO (BCM) | Bus/Type |
| :--- | :--- | :--- | :--- | :--- |
| **VL6180X Sensor** | SCL | `SCL` | 3 | I2C |
| | SDA | `SDA` | 2 | I2C |
| **Waveshare LCD** | DIN | `SPI_MOSI`| 10 | SPI |
| | CLK | `SPI_SCLK`| 11 | SPI |
| | CS | `SPI_CS0` | 8 | SPI |
| | DC | `D25` | 25 | GPIO |
| | RST | `D27` | 27 | GPIO |
| | BL | `D24` | 24 | GPIO |
| **Buzzer** | Signal | `D23` | 23 | GPIO |
| **I2S Mic** | SCK | `D18` | 18 | I2S |
| | WS | `D19` | 19 | I2S |
| | SD | `D20` | 20 | I2S |
| **Servos (x4)**| Signal | `S0`-`S3` | N/A | PWM |

---

## **4. Software Setup on Raspberry Pi**

This setup process can be time-consuming on a Pi Zero. Be patient and ensure a stable power supply.

1.  **Install OS:** Use Raspberry Pi Imager to flash **Raspberry Pi OS (Legacy, 32-bit)**. The legacy "Bullseye" version often has better compatibility for these specific hardware libraries. Use the advanced options (⚙️) to pre-configure WiFi, SSH, and your user account.

2.  **First Boot & Connect:** Boot the Pi and connect via SSH (`ssh your_username@your_pi_hostname.local`).

3.  **System Update:**
    ```bash
    sudo apt update
    sudo apt full-upgrade -y
    sudo apt install -y curl git
    sudo reboot
    ```

4.  **Enable Hardware Interfaces:**
    *   Run the configuration tool: `sudo raspi-config`
    *   Navigate to `Interface Options`.
    *   Enable **`I2C`**. (For the VL6180X sensor).
    *   Enable **`SPI`**. (For the LCD face).
    *   Enable **`I2S`**. (For the optional microphone).
    *   Exit `raspi-config` and reboot when prompted.

5.  **Install Dependencies:** Install libraries for Python, audio, I2C, and compiling packages.
    ```bash
    sudo apt install -y python3-dev python3-pip python3-venv build-essential libasound2-dev portaudio19-dev libportaudio2 libportaudiocpp0 ffmpeg flac libatlas-base-dev python3-smbus i2c-tools libopenjp2-7
    ```

6.  **Install Rust Compiler:** This is required by a dependency of the `google-generativeai` library. **This step can take over an hour.**
    ```bash
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
    ```
    *   Choose option `1) Proceed with installation (default)`.
    *   After installation, run the command it provides to configure your shell, similar to:
        ```bash
        source "$HOME/.cargo/env"
        echo 'source "$HOME/.cargo/env"' >> ~/.bashrc
        ```

7.  **Increase Swap Space (Crucial):** Temporarily increase swap space to prevent crashes during package installation.
    ```bash
    sudo dphys-swapfile swapoff
    sudo sed -i 's/CONF_SWAPSIZE=100/CONF_SWAPSIZE=1024/' /etc/dphys-swapfile
    sudo dphys-swapfile setup
    sudo dphys-swapfile swapon
    ```

8.  **Create Project & Virtual Environment:**
    ```bash
    mkdir ~/NinjaRobot
    cd ~/NinjaRobot
    python3 -m venv .venv
    source .venv/bin/activate
    ```
    *(Your terminal prompt should now show `(.venv)`)*

9.  **Install Python Libraries:** **This step will also take a very long time.**
    ```bash
    pip install --upgrade pip
    pip install RPi.GPIO spidev smbus2 Pillow numpy
    pip install google-generativeai SpeechRecognition gTTS pygame Flask google-cloud-speech
    ```

10. **Revert Swap Space:** After installation, revert swap to the default to reduce SD card wear.
    ```bash
    sudo dphys-swapfile swapoff
    sudo sed -i 's/CONF_SWAPSIZE=1024/CONF_SWAPSIZE=100/' /etc/dphys-swapfile
    sudo dphys-swapfile setup
    sudo dphys-swapfile swapon
    sudo reboot
    ```
    *(Reconnect via SSH after the reboot)*

11. **Verify I2C Sensor:** After rebooting, check if the VL6180X sensor is detected. Its address should be `0x29`.
    ```bash
    sudo i2cdetect -y 1
    ```
    If you don't see `29` in the grid, double-check your wiring.

---

## **5. Code Implementation**

Set up the project directory with all the necessary code files. Ensure your virtual environment is active (`source ~/NinjaRobot/.venv/bin/activate`).

### **File 1: `DFRobot_VL6180X.py`** (Sensor Library)

This is a required library for the new distance sensor.

**Create the file:**
```bash
cd ~/NinjaRobot
nano DFRobot_VL6180X.py
```

**Paste the following code into the file:**
```python
# -*- coding: utf-8 -*
""" file DFRobot_VL6180X.py
  # DFRobot_VL6180X Class infrastructure, implementation of underlying methods
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @licence     The MIT License (MIT)
  @author      [yangfeng]<feng.yang@dfrobot.com> 
  @version  V1.0
  @date  2021-02-09
  @get from https://www.dfrobot.com
  @url https://github.com/DFRobot/DFRobot_VL6180X
"""
import smbus2 as smbus # Use smbus2
import time

class DFRobot_VL6180X:
  # IIC ADDR
  VL6180X_IIC_ADDRESS                          = 0x29

  # The sensor register address
  VL6180X_IDENTIFICATION_MODEL_ID             = 0x000
  VL6180X_SYSTEM_MODE_GPIO0                   = 0X010
  VL6180X_SYSTEM_MODE_GPIO1                   = 0X011
  VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO        = 0x014
  VL6180X_SYSTEM_INTERRUPT_CLEAR              = 0x015
  VL6180X_SYSTEM_FRESH_OUT_OF_RESET           = 0x016
  VL6180X_SYSTEM_GROUPED_PARAMETER_HOLD       = 0x017
  VL6180X_SYSRANGE_START                      = 0x018
  VL6180X_SYSRANGE_THRESH_HIGH                = 0x019
  VL6180X_SYSRANGE_THRESH_LOW                 = 0x01A
  VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD    = 0x01B
  VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME       = 0x01C
  VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE = 0x022
  VL6180X_SYSRANGE_MAX_AMBIENT_LEVEL_MULT     = 0x02C
  VL6180X_SYSRANGE_RANGE_CHECK_ENABLES        = 0x02D
  VL6180X_SYSRANGE_VHV_RECALIBRATE            = 0x02E
  VL6180X_SYSRANGE_VHV_REPEAT_RATE            = 0x031
  VL6180X_SYSALS_START                        = 0x038
  VL6180X_SYSALS_THRESH_HIGH                  = 0x03A
  VL6180X_SYSALS_THRESH_LOW                   = 0x03C
  VL6180X_SYSALS_INTERMEASUREMENT_PERIOD      = 0x03E
  VL6180X_SYSALS_ANALOGUE_GAIN                = 0x03F
  VL6180X_SYSALS_INTEGRATION_PERIOD           = 0x040
  VL6180X_RESULT_RANGE_STATUS                 = 0x04D
  VL6180X_RESULT_ALS_STATUS                   = 0x04E
  VL6180X_RESULT_INTERRUPT_STATUS_GPIO        = 0x04F
  VL6180X_RESULT_ALS_VAL                      = 0x050
  VL6180X_RESULT_RANGE_VAL                    = 0x062
  VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD     = 0x10A
  VL6180X_FIRMWARE_RESULT_SCALER              = 0x120
  VL6180X_I2C_SLAVE_DEVICE_ADDRESS            = 0x212
  VL6180X_INTERLEAVED_MODE_ENABLE             = 0x2A3
  
  # The valid ID of the sensor
  VL6180X_ID                                  = 0xB4
  VL6180X_ALS_GAIN_20                         = 0
  VL6180X_ALS_GAIN_10                         = 1
  VL6180X_ALS_GAIN_5                          = 2
  VL6180X_ALS_GAIN_2_5                        = 3
  VL6180X_ALS_GAIN_1_67                       = 4
  VL6180X_ALS_GAIN_1_25                       = 5
  VL6180X_ALS_GAIN_1                          = 6
  VL6180X_ALS_GAIN_40                         = 7
  
  def __init__(self,iic_addr =VL6180X_IIC_ADDRESS,bus = 1):
    self.__i2cbus = smbus.SMBus(bus)
    self.__i2c_addr = iic_addr
    self.__gain = 1.0
    self.__atime =100

  def begin(self):
    try:
        device_id = self.__read_byte(self.VL6180X_IDENTIFICATION_MODEL_ID)
    except IOError:
        return False
    if device_id != self.VL6180X_ID:
      return False
    self.__init_sensor()
    return True 

  def range_poll_measurement(self):
    self.__write_byte(self.VL6180X_SYSRANGE_START, 0x01)
    time.sleep(0.01) # Poll for completion
    return self.range_get_measurement()

  def range_get_measurement(self):
    return self.__read_byte(self.VL6180X_RESULT_RANGE_VAL)

  def __init_sensor(self):
    self.__write_byte(0x0207, 0x01)
    self.__write_byte(0x0208, 0x01)
    self.__write_byte(0x0096, 0x00)
    self.__write_byte(0x0097, 0xfd)
    self.__write_byte(0x00e3, 0x00)
    self.__write_byte(0x00e4, 0x04)
    self.__write_byte(0x00e5, 0x02)
    self.__write_byte(0x00e6, 0x01)
    self.__write_byte(0x00e7, 0x03)
    self.__write_byte(0x00f5, 0x02)
    self.__write_byte(0x00d9, 0x05)
    self.__write_byte(0x00db, 0xce)
    self.__write_byte(0x00dc, 0x03)
    self.__write_byte(0x00dd, 0xf8)
    self.__write_byte(0x009f, 0x00)
    self.__write_byte(0x00a3, 0x3c)
    self.__write_byte(0x00b7, 0x00)
    self.__write_byte(0x00bb, 0x3c)
    self.__write_byte(0x00b2, 0x09)
    self.__write_byte(0x00ca, 0x09)
    self.__write_byte(0x0198, 0x01)
    self.__write_byte(0x01b0, 0x17)
    self.__write_byte(0x01ad, 0x00)
    self.__write_byte(0x00ff, 0x05)
    self.__write_byte(0x0100, 0x05)
    self.__write_byte(0x0199, 0x05)
    self.__write_byte(0x01a6, 0x1b)
    self.__write_byte(0x01ac, 0x3e)
    self.__write_byte(0x01a7, 0x1f)
    self.__write_byte(0x0030, 0x00)
    self.__write_byte(self.VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x24) 
    self.__write_byte(self.VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 0x32)
    self.__write_byte(self.VL6180X_SYSRANGE_RANGE_CHECK_ENABLES, 0x10 | 0x01)
    self.__write_byte(self.VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, 0x7B)
    self.__write_byte(self.VL6180X_SYSALS_INTEGRATION_PERIOD, 0x64)
    self.__write_byte(self.VL6180X_SYSALS_ANALOGUE_GAIN, 0x46)
        self.__write_byte(self.VL6180X_SYSTEM_MODE_GPIO1, 0x10)
    self.__write_byte(self.VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x00)

  def __write_byte(self, register, value):
      self.__i2cbus.write_byte_data(self.__i2c_addr, (register >> 8) & 0xFF, register & 0xFF)
      self.__i2cbus.write_byte_data(self.__i2c_addr, register & 0xFF, value)

  def __read_byte(self, register):
      self.__i2cbus.write_byte_data(self.__i2c_addr, (register >> 8) & 0xFF, register & 0xFF)
      return self.__i2cbus.read_byte(self.__i2c_addr)
```
*Save and Exit (`Ctrl+X`, `Y`, `Enter`)*

### **File 2: `Ninja_Distance.py`** (Updated)

This file is now completely rewritten to use the new VL6180X sensor.

**Create/overwrite the file:**
```bash
nano Ninja_Distance.py```

**Paste the following code:**
```python
# -*- coding: utf-8 -*
'''!
  @file Ninja_Distance.py
  @brief Measures distance using a VL6180X I2C Time-of-Flight sensor.
  @copyright Copyright (c) 2024
  @license The MIT License (MIT)
  @author Assistant
  @version V2.0
  @date 2024-05-25
'''

import time
import sys
import DFRobot_VL6180X

# --- Configuration ---
I2C_BUS = 1 
SENSOR_ADDRESS = 0x29

# --- Global Sensor Object ---
sensor = None
sensor_initialized = False

def setup_sensor():
    """Initializes the VL6180X sensor."""
    global sensor, sensor_initialized
    if sensor_initialized:
        return True
    
    print("Initializing VL6180X distance sensor...")
    try:
        sensor = DFRobot_VL6180X.DFRobot_VL6180X(iic_addr=SENSOR_ADDRESS, bus=I2C_BUS)
        if not sensor.begin():
            print("Error: Failed to initialize the VL6180X sensor.")
            print("Please check wiring and run 'sudo i2cdetect -y 1'.")
            sensor_initialized = False
            return False
        
        print("VL6180X sensor initialized successfully.")
        sensor_initialized = True
        return True
    except Exception as e:
        print(f"An exception occurred during sensor setup: {e}")
        sensor_initialized = False
        return False

def measure_distance():
    """
    Measures distance using the VL6180X sensor.
    Returns:
        float: Distance in centimeters (cm).
        -1: If there is a measurement error or sensor is not initialized.
    """
    if not sensor_initialized or not sensor:
        print("Error: Cannot measure, sensor not initialized.")
        return -1
        
    try:
        # The sensor returns distance in millimeters (mm)
        distance_mm = sensor.range_poll_measurement()
        
        # The sensor returns 255 on error or if the object is out of range.
        if distance_mm == 255:
            return -1 # Treat out of range/error as -1
        
        # Convert millimeters to centimeters for consistency with the rest of the project
        distance_cm = distance_mm / 10.0
        return distance_cm
        
    except Exception as e:
        print(f"An unexpected error occurred during measurement: {e}")
        return -1

# --- Main Execution for Testing ---
if __name__ == "__main__":
    if not setup_sensor():
        sys.exit("Exiting due to sensor initialization failure.")

    print("\n--- VL6180X Distance Sensor Test ---")
    print("Starting measurements (press Ctrl+C to exit)...")
    print("-" * 40)
    
    try:
        while True:
            dist_cm = measure_distance()
            if dist_cm == -1:
                print("Status: Out of range or measurement error")
            else:
                print(f"Distance: {dist_cm:.2f} cm")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        print("Sensor test finished.")
```
*Save and Exit*

### **File 3: `Ninja_Face_LCD.py`** (New)

This new file contains all the logic for controlling the LCD and drawing faces.

**Create the file:**
```bash
nano Ninja_Face_LCD.py
```

**Paste the following code:**
```python
# -*- coding: utf-8 -*
'''!
  @file Ninja_Face_LCD.py
  @brief Controls the Waveshare 2-inch SPI LCD to display expressions for the Ninja Robot.
  @n Combines the low-level driver and high-level face drawing logic.
  @copyright Copyright (c) 2024
  @license The MIT License (MIT)
  @author Assistant
  @version V1.0
  @date 2024-05-25
'''
import spidev
import RPi.GPIO as GPIO
import time
from PIL import Image, ImageDraw, ImageFont
import numpy as np

# --- Low-level LCD Driver Class ---
class WaveshareLCD:
    def __init__(self, rst_pin=27, dc_pin=25, bl_pin=24, cs_pin=8):
        # Pin configuration (BCM numbering) from our new pinout
        self.RST_PIN = rst_pin
        self.DC_PIN = dc_pin
        self.BL_PIN = bl_pin
        self.CS_PIN = cs_pin
        
        self.width = 240
        self.height = 320

        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.RST_PIN, GPIO.OUT)
        GPIO.setup(self.DC_PIN, GPIO.OUT)
        GPIO.setup(self.BL_PIN, GPIO.OUT)
        GPIO.setup(self.CS_PIN, GPIO.OUT)

        # Initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0) # SPI bus 0, device 0 (CS0)
        self.spi.max_speed_hz = 40000000
        self.spi.mode = 0b00

    def _command(self, cmd):
        GPIO.output(self.DC_PIN, GPIO.LOW)
        self.spi.writebytes([cmd])

    def _data(self, data):
        GPIO.output(self.DC_PIN, GPIO.HIGH)
        self.spi.writebytes([data])

    def _reset(self):
        GPIO.output(self.RST_PIN, GPIO.HIGH); time.sleep(0.01)
        GPIO.output(self.RST_PIN, GPIO.LOW); time.sleep(0.01)
        GPIO.output(self.RST_PIN, GPIO.HIGH); time.sleep(0.01)

    def init_display(self):
        self._reset()
        self.backlight_on()
        
        # ST7789V Initialization Sequence
        self._command(0x36); self._data(0x00)
        self._command(0x3A); self._data(0x05)
        self._command(0xB2); self._data(0x0C); self._data(0x0C); self._data(0x00); self._data(0x33); self._data(0x33)
        self._command(0xB7); self._data(0x35)
        self._command(0xBB); self._data(0x19)
        self._command(0xC0); self._data(0x2C)
        self._command(0xC2); self._data(0x01)
        self._command(0xC3); self._data(0x12)
        self._command(0xC4); self._data(0x20)
        self._command(0xC6); self._data(0x0F)
        self._command(0xD0); self._data(0xA4); self._data(0xA1)
        self._command(0xE0); self._data(0xD0); self._data(0x04); self._data(0x0D); self._data(0x11); self._data(0x13); self._data(0x2B); self._data(0x3F); self._data(0x54); self._data(0x4C); self._data(0x18); self._data(0x0D); self._data(0x0B); self._data(0x1F); self._data(0x23)
        self._command(0xE1); self._data(0xD0); self._data(0x04); self._data(0x0C); self._data(0x11); self._data(0x13); self._data(0x2C); self._data(0x3F); self._data(0x44); self._data(0x51); self._data(0x2F); self._data(0x1F); self._data(0x1F); self._data(0x20); self._data(0x23)
        self._command(0x21)
        self._command(0x11); time.sleep(0.12)
        self._command(0x29)
        self.clear()
        print("LCD Driver Initialized.")

    def set_window(self, x_start, y_start, x_end, y_end):
        self._command(0x2A); self._data((x_start >> 8) & 0xFF); self._data(x_start & 0xFF); self._data((x_end >> 8) & 0xFF); self._data(x_end & 0xFF)
        self._command(0x2B); self._data((y_start >> 8) & 0xFF); self._data(y_start & 0xFF); self._data((y_end >> 8) & 0xFF); self._data(y_end & 0xFF)
        self._command(0x2C)

    def display(self, image):
        if image.width != self.width or image.height != self.height:
            image = image.resize((self.width, self.height))
        
        pixel_data = np.array(image.convert("RGB"), dtype='uint16')
        color = ((pixel_data[:, :, 0] & 0xF8) << 8) | ((pixel_data[:, :, 1] & 0xFC) << 3) | (pixel_data[:, :, 2] >> 3)
        
        self.set_window(0, 0, self.width - 1, self.height - 1)
        GPIO.output(self.DC_PIN, GPIO.HIGH)
        self.spi.writebytes2(color.astype('>H').flatten().tolist())

    def clear(self):
        black_image = Image.new("RGB", (self.width, self.height), "BLACK")
        self.display(black_image)

    def backlight_on(self):
        GPIO.output(self.BL_PIN, GPIO.HIGH)

    def cleanup(self):
        print("Cleaning up LCD resources.")
        self.clear()
        self.backlight_on() # Turn off backlight by setting to low
        GPIO.output(self.BL_PIN, GPIO.LOW)
        self.spi.close()
        # GPIO.cleanup() is handled globally in ninja_core

# --- High-level Face Drawing Class ---
class RobotFace:
    def __init__(self, lcd_driver):
        self.lcd = lcd_driver
        self.width = 320 # Drawing canvas is horizontal
        self.height = 240
        try:
            self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24)
        except IOError:
            self.font = ImageFont.load_default()

    def _display(self, image):
        # Rotate horizontal canvas for vertical display
        rotated_image = image.rotate(90, expand=True)
        self.lcd.display(rotated_image)

    def _create_base_image(self):
        image = Image.new("RGB", (self.width, self.height), "BLACK")
        draw = ImageDraw.Draw(image)
        return image, draw

    def _draw_eyes(self, draw, mood='idle', pupil_data=None):
        eye_y, eye_radius = 120, 45
        left_eye_x = self.width // 2 - 80
        right_eye_x = self.width // 2 + 80

        draw.ellipse((left_eye_x - eye_radius, eye_y - eye_radius, left_eye_x + eye_radius, eye_y + eye_radius), fill="WHITE")
        draw.ellipse((right_eye_x - eye_radius, eye_y - eye_radius, right_eye_x + eye_radius, eye_y + eye_radius), fill="WHITE")

        if pupil_data:
            for i, p in enumerate(pupil_data):
                eye_x = left_eye_x if i == 0 else right_eye_x
                px, py, p_radius = p
                draw.ellipse((eye_x + px - p_radius, eye_y + py - p_radius, eye_x + px + p_radius, eye_y + py + p_radius), fill="BLACK")

        if mood == 'happy':
            draw.arc((left_eye_x - eye_radius, eye_y - eye_radius, left_eye_x + eye_radius, eye_y + eye_radius), 180, 360, fill="BLACK", width=5)
            draw.arc((right_eye_x - eye_radius, eye_y - eye_radius, right_eye_x + eye_radius, eye_y + eye_radius), 180, 360, fill="BLACK", width=5)
        elif mood == 'sleepy':
            draw.rectangle((left_eye_x - eye_radius, eye_y - eye_radius, left_eye_x + eye_radius, eye_y - 15), fill="BLACK")
            draw.rectangle((right_eye_x - eye_radius, eye_y - eye_radius, right_eye_x + eye_radius, eye_y - 15), fill="BLACK")
        elif mood == 'angry':
            draw.line([(left_eye_x - 50, eye_y - 30), (left_eye_x + 30, eye_y - 10)], fill="WHITE", width=15)
            draw.line([(right_eye_x + 50, eye_y - 30), (right_eye_x - 30, eye_y - 10)], fill="WHITE", width=15)
        elif mood == 'scary':
            draw.ellipse((left_eye_x - eye_radius, eye_y - eye_radius, left_eye_x + eye_radius, eye_y + eye_radius), fill="RED")
            draw.ellipse((right_eye_x - eye_radius, eye_y - eye_radius, right_eye_x + eye_radius, eye_y + eye_radius), fill="RED")
            draw.ellipse((left_eye_x - 10, eye_y - 10, left_eye_x + 10, eye_y + 10), fill="BLACK")
            draw.ellipse((right_eye_x - 10, eye_y - 10, right_eye_x + 10, eye_y + 10), fill="BLACK")

    def show_expression(self, mood='idle'):
        print(f"FACE: Setting expression to '{mood}'")
        image, draw = self._create_base_image()
        pupils = [(0, 0, 15), (0, 0, 15)] # Default pupils

        if mood == 'idle':
            self.animate_blink()
            return
        elif mood == 'happy':
            self._draw_eyes(draw, mood='happy', pupil_data=pupils)
            draw.arc((120, 180, 200, 240), 0, 180, fill="WHITE", width=10)
        elif mood == 'sad':
            pupils = [(0, 15, 12), (0, 15, 12)]
            self._draw_eyes(draw, mood='sad', pupil_data=pupils)
            draw.arc((130, 200, 190, 230), 180, 360, fill="WHITE", width=8)
        elif mood == 'angry' or mood == 'danger':
            pupils = [(0, 0, 18), (0, 0, 18)]
            self._draw_eyes(draw, mood='angry', pupil_data=pupils)
            draw.line((130, 210, 190, 210), fill="WHITE", width=8)
        elif mood == 'sleepy':
            self.animate_sleepy()
            return
        elif mood == 'scary':
             self._draw_eyes(draw, mood='scary')
        elif mood == 'confused':
            pupils = [(-20, 0, 12), (20, 0, 12)]
            self._draw_eyes(draw, mood='confused', pupil_data=pupils)
            draw.text((145, 180), "?", fill="WHITE", font=self.font)
        else: # Default/fallback case
            self._draw_eyes(draw, pupil_data=pupils)
            draw.line((140, 200, 180, 200), fill="WHITE", width=6)
        
        self._display(image)

    def animate_blink(self):
        # Open eyes
        image_open, draw_open = self._create_base_image()
        self._draw_eyes(draw_open, pupil_data=[(0, 0, 15), (0, 0, 15)])
        draw_open.line((140, 200, 180, 200), fill="WHITE", width=6)
        
        # Closed eyes
        image_closed, draw_closed = self._create_base_image()
        draw_closed.line((self.width // 2 - 120, 120, self.width // 2 - 40, 120), fill="WHITE", width=8)
        draw_closed.line((self.width // 2 + 40, 120, self.width // 2 + 120, 120), fill="WHITE", width=8)
        draw_closed.line((140, 200, 180, 200), fill="WHITE", width=6)

        self._display(image_closed)
        time.sleep(0.15)
        self._display(image_open)

    def animate_sleepy(self):
        image_sleepy, draw_sleepy = self._create_base_image()
        self._draw_eyes(draw_sleepy, mood='sleepy')
        self._display(image_sleepy)
```
*Save and Exit*

### **File 4: `ninja_core.py`** (Heavily Updated)

This is the central nervous system of the robot. It's updated to use the new face and sensor and to map sounds to facial expressions.

**Create/overwrite the file:**
```bash
nano ninja_core.py
```

**Paste the following code:**
```python
# -*- coding:utf-8 -*-
'''!
  @file ninja_core.py
  @brief Core logic for the Ninja Robot, now with an LCD face and ToF sensor.
  @copyright Copyright (c) 2024
  @license The MIT License (MIT)
  @author Assistant
  @version V2.0 (LCD Face & VL6180X Integration)
  @date 2024-05-25
'''
import sys
import time
import re
import threading
import json
import RPi.GPIO as GPIO
import google.generativeai as genai

# --- Configuration ---
GOOGLE_API_KEY = "PASTE_YOUR_GOOGLE_GEMINI_API_KEY_HERE"
GEMINI_MODEL_NAME = "gemini-1.5-flash-latest"

DISTANCE_THRESHOLD_CM = 15.0 # Stop if an obstacle is closer than this

# --- Import Robot Modules ---
try:
    import Ninja_Movements_v1 as movements
    import Ninja_Buzzer as buzzer
    import Ninja_Distance as distance
    import Ninja_Face_LCD as face_lcd
except ImportError as e:
    print(f"Error importing robot modules: {e}")
    sys.exit(1)

# --- Global Variables ---
model = None
movement_thread = None
distance_check_thread = None
is_continuous_moving = False
buzzer_pwm = None
face = None # Global face object
keep_distance_checking = False
hardware_initialized = False

# --- Mapping sounds to face expressions ---
SOUND_TO_FACE_MAP = {
    "hello": "happy", "thanks": "happy", "yes": "idle",
    "no": "sad", "danger": "danger", "scared": "scary",
    "exciting": "happy", "happy": "happy", "sleepy": "sleepy",
    "left": "idle", "right": "idle"
}

def initialize_gemini():
    global model
    if model: return True
    print(f"Initializing Gemini model: {GEMINI_MODEL_NAME}...")
    if not GOOGLE_API_KEY or "PASTE_YOUR" in GOOGLE_API_KEY:
        print("ERROR: Google Gemini API key is not set in ninja_core.py.")
        return False
    try:
        genai.configure(api_key=GOOGLE_API_KEY)
        model = genai.GenerativeModel(GEMINI_MODEL_NAME)
        print("Gemini model loaded successfully.")
        return True
    except Exception as e:
        print(f"Error loading Gemini model: {e}")
        return False

def initialize_hardware():
    global buzzer_pwm, face, hardware_initialized
    if hardware_initialized: return True
    print("Initializing hardware components...")
    try:
        # Init LCD Face first (uses RPi.GPIO)
        lcd = face_lcd.WaveshareLCD()
        lcd.init_display()
        face = face_lcd.RobotFace(lcd)
        face.show_expression('idle') # Start with a blinking idle face
        
        # Init Servos
        movements.init_board_and_servo()
        
        # Init Buzzer
        buzzer.setup()
        buzzer_pwm = GPIO.PWM(buzzer.BUZZER_PIN, 440)
        buzzer_pwm.start(0)
        
        # Init Distance Sensor
        distance.setup_sensor()
        
        hardware_initialized = True
        print("Hardware components initialized.")
        movements.reset_servos()
        play_robot_sound('hello') # This will also trigger the face
        time.sleep(1)
        print("Hardware initialization and setup complete.")
        return True
    except Exception as e:
        print(f"An error occurred during hardware initialization: {e}")
        try: GPIO.cleanup()
        except: pass
        hardware_initialized = False
        return False

def cleanup_all():
    global keep_distance_checking, movement_thread, is_continuous_moving, hardware_initialized
    print("\n--- Initiating Cleanup ---")
    if hardware_initialized:
        play_robot_sound("sleepy")
        time.sleep(1)

    keep_distance_checking = False
    if distance_check_thread and distance_check_thread.is_alive():
        distance_check_thread.join(timeout=0.5)

    if is_continuous_moving or (movement_thread and movement_thread.is_alive()):
        movements.stop()
    if movement_thread and movement_thread.is_alive():
        movement_thread.join(timeout=1.0)
    is_continuous_moving = False

    if hardware_initialized:
        try: movements.rest()
        except: pass
        if face: face.lcd.cleanup()
        if buzzer_pwm: buzzer_pwm.stop()
        GPIO.cleanup()
    
    hardware_initialized = False
    print("Cleanup complete.")

def play_robot_sound(sound_keyword):
    if not hardware_initialized: return
    
    # Set face expression based on sound
    expression = SOUND_TO_FACE_MAP.get(sound_keyword.lower(), 'idle')
    if face:
        face.show_expression(expression)

    # Play the sound
    sound_action = buzzer.SOUND_MAP.get(sound_keyword.lower())
    try:
        if sound_action == buzzer.SOUND_SCARED_IDENTIFIER:
            buzzer.play_scared_sound(buzzer_pwm)
        elif sound_action == buzzer.SOUND_EXCITING_IDENTIFIER:
            buzzer.play_exciting_trill(buzzer_pwm)
        elif isinstance(sound_action, list):
            buzzer.play_sequence(buzzer_pwm, sound_action)
        else:
            print(f"Warning: No sound for '{sound_keyword}'.")
    except Exception as e:
        print(f"Error playing sound '{sound_keyword}': {e}")

def distance_checker():
    global keep_distance_checking, is_continuous_moving
    print("Distance checker thread started.")
    last_warning_time = 0
    warning_interval = 2.0

    while keep_distance_checking:
        if not is_continuous_moving or movements.stop_movement: break
        if not hardware_initialized: break

        dist_cm = distance.measure_distance()

        if dist_cm != -1 and 0 <= dist_cm < DISTANCE_THRESHOLD_CM:
            print(f"!!! OBSTACLE DETECTED at {dist_cm:.1f} cm !!!")
            current_time = time.time()
            if current_time - last_warning_time > warning_interval:
                play_robot_sound('danger')
                last_warning_time = current_time
            if is_continuous_moving:
                movements.stop()
                is_continuous_moving = False
            break 
        time.sleep(0.15)
    print("Distance checker thread finished.")
    keep_distance_checking = False

def process_user_command_with_gemini(user_command_full, language_code='en-US'):
    if not model:
        return {"action_type": "unknown", "error": "Gemini model not ready."}
    
    prompt = f"""
Analyze the following command for a robot. The robot can move, make sounds, and show expressions.
The command could be English or Japanese. Respond in JSON only.

Available Functions: 'hello', 'walk', 'stepback', 'run', 'runback', 'turnleft_step', 'turnright_step', 'rotateleft', 'rotateright', 'stop', 'reset_servos', 'rest'.
Available Speeds: 'normal', 'fast', 'slow'.
Available Sounds: 'hello', 'thanks', 'no', 'yes', 'danger', 'exciting', 'happy', 'right', 'left', 'scared', 'sleepy'.

Is the command a general question OR a robot command?
- If a question (e.g., 'what is the weather'), set "action_type": "conversation", "response_text": "your answer".
- If a command, determine the action.

JSON Output Keys: "action_type", "move_function", "speed", "sound_keyword", "error", "response_text".

Examples:
"Ninja, run forward fast" -> {{"action_type": "combo", "move_function": "run", "speed": "fast", "sound_keyword": "exciting"}}
"忍者、止まれ" (Ninja, tomare) -> {{"action_type": "move", "move_function": "stop", "sound_keyword": "no"}}
"Are you happy?" -> {{"action_type": "conversation", "response_text": "Yes, I am! I feel great today."}}
"Say thank you" -> {{"action_type": "sound", "sound_keyword": "thanks"}}

Command: "{user_command_full}"
Provide ONLY the JSON output:
"""
    print(f"Sending to Gemini: '{user_command_full}'")
    try:
        response = model.generate_content(prompt)
        response_text = response.text.strip()
        
        # Clean response and parse JSON
        json_match = re.search(r'\{.*\}', response_text, re.DOTALL)
        if json_match:
            action_data = json.loads(json_match.group(0))
            print(f"Gemini Interpretation: {action_data}")
            return action_data
        else:
            print(f"Error: No JSON found in Gemini response: {response_text}")
            return {"action_type": "unknown", "error": "Could not parse AI response."}

    except Exception as e:
        print(f"Error communicating with Gemini API: {e}")
        return {"action_type": "unknown", "error": "API communication error."}

def execute_action(action_data, original_command_language='en-US'):
    global movement_thread, is_continuous_moving, keep_distance_checking

    if not hardware_initialized:
        print("Error: Hardware not initialized. Cannot execute action.")
        return

    action_type = action_data.get("action_type", "unknown")

    if action_type == "conversation":
        response_text = action_data.get("response_text", "I'm not sure.")
        print(f"CONVERSATIONAL: {response_text}")
        # In a real system, this text would be sent to a TTS engine.
        # For now, we just show an expression and make a sound.
        play_robot_sound('yes')
        return

    # Stop previous continuous movement if a new move is commanded
    move_func_name = action_data.get("move_function")
    is_new_move = action_type in ["move", "combo"] and move_func_name
    
    if is_new_move and is_continuous_moving:
        print("Stopping previous continuous movement.")
        movements.stop()
        if movement_thread and movement_thread.is_alive():
            movement_thread.join(timeout=1.0)
        is_continuous_moving = False
        time.sleep(0.2)
    
    # Execute sound first
    if action_type in ["sound", "combo"] and action_data.get("sound_keyword"):
        play_robot_sound(action_data["sound_keyword"])
        if move_func_name: time.sleep(0.3) # Pause before moving

    # Execute movement
    if move_func_name:
        target_func = getattr(movements, move_func_name, None)
        if target_func:
            speed = action_data.get("speed", "normal")
            print(f"Executing movement: {move_func_name} (Speed: {speed})")
            
            is_continuous = move_func_name in ["walk", "stepback", "run", "runback", "rotateleft", "rotateright"]
            
            if is_continuous:
                if not is_continuous_moving:
                    is_continuous_moving = True
                    movements.stop_movement = False
                    movement_thread = threading.Thread(target=target_func, args=(speed, None), daemon=True)
                    movement_thread.start()
                    
                    if move_func_name in ["walk", "run"]:
                        keep_distance_checking = True
                        distance_check_thread = threading.Thread(target=distance_checker, daemon=True)
                        distance_check_thread.start()
            elif move_func_name == "stop":
                movements.stop()
                is_continuous_moving = False
            else: # Finite movements
                target_func()
        else:
            print(f"Error: Movement function '{move_func_name}' not found.")
            play_robot_sound('no')

    if action_type == "unknown":
        play_robot_sound('no')

def get_robot_status():
    if not hardware_initialized: return "Hardware Not Initialized"
    if is_continuous_moving: return "Moving..."
    return "Idle"

if __name__ == "__main__":
    if not initialize_gemini() or not initialize_hardware():
        sys.exit("Failed to initialize system. Exiting.")

    print("\n--- Ninja Robot Core Test Interface ---")
    print("Type 'exit' to stop.")
    
    try:
        while True:
            command_full = input("🎤> ").strip()
            if command_full.lower() in ["exit", "quit"]: break
            if not command_full: continue

            action_data = process_user_command_with_gemini(command_full)
            if action_data: execute_action(action_data)
            
    except KeyboardInterrupt:
        print("\nCtrl+C detected.")
    finally:
        cleanup_all()
        print("Program terminated.")
```
*Save and Exit*

### **File 5: `Ninja_Movements_v1.py`**

The movements file remains largely the same, but ensure it uses the `DFRobot_RaspberryPi_Expansion_Board` library correctly. **No changes are needed to this file from the original version you provided.**

### **File 6: `Ninja_Buzzer.py`**

The buzzer file, which maps keywords to sound sequences, is also unchanged. **No changes are needed to this file.**

### **File 7: `web_interface.py`**

The web interface code requires minimal changes, mainly to ensure it correctly initializes the new `ninja_core` and handles its states. This version is simplified for robustness.

**Create/overwrite the file:**
```bash
nano web_interface.py
```

**Paste the following code:**
```python
# -*- coding:utf-8 -*-
import time
import atexit
import os
import json
from flask import Flask, render_template, request, jsonify
import ninja_core

app = Flask(__name__)

# --- Initialization ---
print("--- Initializing Robot Core from Web Interface ---")
last_status_message = "System Initializing..."
if not ninja_core.initialize_gemini():
    last_status_message = "CRITICAL: Gemini AI failed to initialize."
elif not ninja_core.initialize_hardware():
    last_status_message = "CRITICAL: Robot hardware failed to initialize."
else:
    last_status_message = "System Initialized. Ready for commands."

# Register cleanup function to be called on exit
atexit.register(ninja_core.cleanup_all)

# --- State Variables ---
last_command_details = {"type": "N/A", "content": ""}
last_interpretation_or_response = {"status": "Awaiting command"}

@app.route('/')
def index():
    try:
        interp_str = json.dumps(last_interpretation_or_response, indent=2, ensure_ascii=False)
    except TypeError:
        interp_str = "{}"
        
    return render_template('index.html',
                           status=last_status_message,
                           last_command_type=last_command_details["type"],
                           last_command_content=last_command_details["content"],
                           interpretation=interp_str,
                           robot_state=ninja_core.get_robot_status())

@app.route('/controller_command', methods=['POST'])
def handle_controller_command():
    global last_status_message, last_command_details, last_interpretation_or_response
    data = request.get_json()
    command = data.get('command', '')
    speed = data.get('speed', 'normal')
    
    last_command_details = {"type": "Controller", "content": f"{command} ({speed})"}
    
    if not ninja_core.hardware_initialized:
        last_status_message = "Error: Hardware not initialized."
        return jsonify({"status": "error", "message": last_status_message}), 500

    # For direct commands, create the action data structure manually
    action_data = {"move_function": command, "speed": speed}
    if command in ['hello', 'stop', 'rest', 'reset_servos']:
        action_data["action_type"] = "move"
        # Map sound for direct command
        sound_map = {"hello": "hello", "stop": "no", "rest": "thanks"}
        if command in sound_map:
            action_data["sound_keyword"] = sound_map[command]
            action_data["action_type"] = "combo"
    else: # Continuous movements
        action_data["action_type"] = "move"
        sound_map = {"run": "exciting", "turnleft_step": "left", "rotateright": "right"} # etc.
        if command in sound_map:
            action_data["sound_keyword"] = sound_map[command]
            action_data["action_type"] = "combo"

    last_interpretation_or_response = action_data
    
    try:
        ninja_core.execute_action(action_data)
        last_status_message = f"Controller action '{command}' initiated."
        status = "success"
    except Exception as e:
        last_status_message = f"Error executing '{command}': {e}"
        status = "error"
        
    return jsonify({
        "status": status,
        "message": last_status_message,
        "interpretation": last_interpretation_or_response
    })

@app.route('/voice_command_text', methods=['POST'])
def handle_voice_command_text():
    global last_status_message, last_command_details, last_interpretation_or_response
    data = request.get_json()
    command_text = data.get('command_text', '')
    lang_code = data.get('language_code', 'en-US')
    
    last_command_details = {"type": f"Voice ({lang_code})", "content": command_text}

    if not ninja_core.model:
        last_status_message = "Error: AI not initialized."
        return jsonify({"status": "error", "message": last_status_message}), 500

    processed_data = ninja_core.process_user_command_with_gemini(command_text, lang_code)
    last_interpretation_or_response = processed_data
    
    action_type = processed_data.get("action_type")
    
    if action_type == "conversation":
        last_status_message = "AI responded conversationally."
        status = "info"
        ninja_core.execute_action(processed_data) # Let core handle the sound/face
    elif action_type != "unknown" and ninja_core.hardware_initialized:
        try:
            ninja_core.execute_action(processed_data)
            last_status_message = f"Voice action initiated: {processed_data.get('move_function') or processed_data.get('sound_keyword')}"
            status = "success"
        except Exception as e:
            last_status_message = f"Error executing voice action: {e}"
            status = "error"
    else:
        last_status_message = "AI could not determine a valid action."
        status = "warning"
        if ninja_core.hardware_initialized: ninja_core.play_robot_sound('no')
        
    return jsonify({
        "status": status,
        "message": last_status_message,
        "interpretation": last_interpretation_or_response
    })


if __name__ == '__main__':
    try:
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip_address = s.getsockname()[0]
        s.close()
        print(f"INFO: Web server will be accessible at http://{ip_address}:5000")
    except Exception:
        ip_address = "0.0.0.0"
        print("INFO: Could not determine local IP. Access via http://<your_pi_hostname>.local:5000")

    app.run(host='0.0.0.0', port=5000, debug=False)```

### **File 8: `templates/index.html`**

Create the `templates` directory and the `index.html` file inside it. **No changes are needed to the HTML file you provided.**

```bash
mkdir ~/NinjaRobot/templates
nano ~/NinjaRobot/templates/index.html
```
*(Paste the original HTML code here)*

---

## **6. Running the Application**

1.  **Activate Environment:** Navigate to your project directory and activate the virtual environment:
    ```bash
    cd ~/NinjaRobot
    source .venv/bin/activate
    ```

2.  **Find Pi's IP Address:**
    ```bash
    hostname -I | awk '{print $1}'
    ```
    *(Note this IP address)*

3.  **Start the Web Server:**
    ```bash
    python3 web_interface.py
    ```
    *The LCD screen should light up and display the robot's idle face. The terminal will show initialization messages.*

4.  **Access Web Interface:** On a phone or computer on the **same WiFi network**, open a browser and go to:
    `http://<YOUR_PI_IP_ADDRESS>:5000`

---

## **7. Using the Interface**

The interface allows for direct control and voice commands. As you issue commands, you should see the robot's physical actions, hear sounds from the buzzer, and see its facial expression change on the LCD.

*   **D-Pad & Action Buttons:** Control movement directly. The robot's face will change based on the action (e.g., a happy face for "Hello").
*   **Mic Buttons (EN/JP):** Use your browser's microphone to give commands.
    *   **Robot Commands:** Start with "Ninja" (or "忍者"). Example: "Ninja, walk forward." The robot will execute the command and its face will change.
    *   **General Questions:** Ask anything else. Example: "Are you happy?" The robot will answer conversationally (response shown in the status area) and its face will react.

---

## **8. Troubleshooting**

*   **LCD Not Working:**
    *   Double-check all 8 wires from the LCD to the HAT.
    *   Ensure SPI is enabled in `raspi-config`.
    *   Verify the pin numbers in `Ninja_Face_LCD.py` (`rst_pin=27, dc_pin=25, bl_pin=24`).
*   **Distance Sensor Error / `i2cdetect` fails:**
    *   Check the 4 I2C wires (VCC, GND, SCL, SDA).
    *   Ensure I2C is enabled in `raspi-config`.
    *   Make sure you installed `smbus2` via pip.
*   **`ModuleNotFoundError`:**
    *   Make sure your virtual environment is active (`source .venv/bin/activate`).
    *   Ensure all libraries from Step 4.9 were installed successfully.
*   **AI Errors / API Key Issues:**
    *   Make sure you have pasted your correct Google Gemini API key into `ninja_core.py`.
*   **Robot movement is erratic:**
    *   Ensure your power supply is at least 5V, 2.5A.
    *   Check that the 360° and 180° servos are connected to the correct ports as defined in the assembly guide.
