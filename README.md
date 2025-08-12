# **Build Your Own AI Ninja Robot (Upgraded with LCD Face & ToF Sensor)**



This comprehensive tutorial guides you through building and programming an advanced, voice-controlled robot based on a Raspberry Pi Zero 2W. This upgraded version features a precise **VL6180X Time-of-Flight distance sensor** for obstacle detection and a **Waveshare 2-inch SPI LCD** that gives the robot an expressive, interactive face.

The project integrates movement, sound, vision, and AI, all controllable through a web interface or by voice using Google's Gemini AI.

---

## **Table of Contents**

*   [1. Hardware Requirements](#1-hardware-requirements)
*   [2. Hardware Assembly (Corrected Pinout)](#2-hardware-assembly-corrected-pinout)
*   [3. Final GPIO Pinout Configuration](#3-final-gpio-pinout-configuration)
*   [4. Software Setup on Raspberry Pi](#4-software-setup-on-raspberry-pi)
*   [5. Code Implementation (Updated)](#5-code-implementation-updated)
*   [6. Running the Application](#6-running-the-application)
*   [7. Using the Interface](#7-using-the-interface)
*   [8. Troubleshooting](#8-troubleshooting)

---

## **1. Hardware Requirements**

*   **Core:**
    *   Raspberry Pi Zero 2 W
    *   **DFRobot IO Expansion HAT for Pi Zero**
    *   DFRobot UPS HAT for Raspberry Pi Zero
    *   16GB+ Micro SD Card (Class 10)
*   **Sensors & Display:**
    *   **Waveshare 2-inch SPI LCD Module** (240x320, ST7789V driver)
    *   **VL6180X Distance Sensor** (I2C)
*   **Actuators & Audio:**
    *   SG90 180° Servos (x2)
    *   SG90 360° Continuous Rotation Servos (x2)
    *   Active Buzzer Module
*   **Miscellaneous:**
    *   Robot Chassis/Frame
    *   Jumper Wires (Female-to-Female)
    *   5V, >= 2.5A Micro USB Power Supply

---

## **2. Hardware Assembly (Corrected Pinout)**

**⚠️ IMPORTANT:** Always disconnect the power supply before connecting or disconnecting any components! The following instructions use the **exact labels printed on the DFRobot IO Expansion HAT**.

1.  **Stack the HATs:**
    *   First, mount the **DFRobot UPS HAT** onto the Raspberry Pi Zero's 40-pin GPIO header.
    *   Next, carefully stack the **DFRobot IO Expansion HAT** on top of the UPS HAT.

2.  **Connect the VL6180X Distance Sensor (I2C):**
    *   Locate the dedicated 4-pin header labeled **`I2C`** on the IO Expansion HAT.
    *   Connect the sensor directly to this port:
        *   Sensor `VCC` -> HAT `I2C` port `V` pin
        *   Sensor `GND` -> HAT `I2C` port `G` pin
        *   Sensor `SCL` -> HAT `I2C` port `SCL` pin
        *   Sensor `SDA` -> HAT `I2C` port `SDA` pin

3.  **Connect the Waveshare 2-inch LCD (SPI):**
    *   This uses both the dedicated `SPI` header and some of the digital `D` pins.
    *   **SPI Header Connections:** Locate the header labeled **`SPI`**.
        *   LCD `DIN` (Data In) -> HAT `SPI` header `MOSI` pin
        *   LCD `CLK` (Clock) -> HAT `SPI` header `SCK` pin
        *   LCD `CS` (Chip Select) -> HAT `SPI` header `CS` pin
    *   **Digital Pin Connections:**
        *   LCD `DC` (Data/Command) -> HAT `D25` pin
        *   LCD `RST` (Reset) -> HAT `D17` pin
        *   LCD `BL` (Backlight) -> HAT `D24` pin
    *   **Power Connections:**
        *   LCD `VCC` -> Any `3V3` pin on the HAT
        *   LCD `GND` -> Any `GND` pin on the HAT

4.  **Connect Servos:**
    *   Connect the four servos to the dedicated 3-pin headers labeled **`S0`**, **`S1`**, **`S2`**, and **`S3`**.
    *   Ensure correct polarity: Signal (Yellow/Orange wire), VCC (Red wire), GND (Brown/Black wire).
    *   **Default Configuration:**
        *   `S0`: Right Leg/Hip (180° SG90)
        *   `S1`: Left Leg/Hip (180° SG90)
        *   `S2`: Right Foot/Wheel (360° SG90)
        *   `S3`: Left Foot/Wheel (360° SG90)

5.  **Connect Buzzer:**
    *   Connect the buzzer module to the digital pins:
    *   `Signal/IO` -> HAT `D23` pin
    *   `VCC/+` -> HAT `3V3` pin
    *   `GND/-` -> HAT `GND` pin

6.  **Final Assembly:** Mount all components onto your robot chassis. Organize wiring carefully.

---

## **3. Final GPIO Pinout Configuration**

This table summarizes the final, conflict-free pinout, referencing the HAT's physical labels.

| Component | Pin Function | HAT Label | RPi GPIO (BCM) | Bus/Type |
| :--- | :--- | :--- | :--- | :--- |
| **VL6180X Sensor**| SCL | `I2C` Header `SCL` | 3 | I2C |
| | SDA | `I2C` Header `SDA` | 2 | I2C |
| **Waveshare LCD** | DIN | `SPI` Header `MOSI` | 10 | SPI |
| | CLK | `SPI` Header `SCK` | 11 | SPI |
| | CS | `SPI` Header `CS` | 8 | SPI |
| | DC | `D25` | 25 | GPIO |
| | RST | `D17` | 17 | GPIO |
| | BL | `D24` | 24 | GPIO |
| **Buzzer** | Signal | `D23` | 23 | GPIO |
| **Servos (x4)**| Signal | `S0`-`S3` | N/A | PWM |

---

## **4. Software Setup on Raspberry Pi**

This setup process can be time-consuming on a Pi Zero. Be patient and ensure a stable power supply.

1.  **Install OS:** Use Raspberry Pi Imager to flash **Raspberry Pi OS (Legacy, 32-bit)** based on "Bullseye". This version has proven to have the best compatibility for these specific hardware libraries. Use the advanced options (⚙️) to pre-configure WiFi, SSH, and your user account.

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
    *   Navigate to `3 Interface Options`.
    *   Enable **`I3 I2C`**.
    *   Enable **`I4 SPI`**.
    *   (Optional) Enable **`I5 I2S`** if using the microphone.
    *   Exit `raspi-config` and reboot.

5.  **Install System Dependencies:**
    ```bash
    sudo apt install -y python3-dev python3-pip python3-venv build-essential libasound2-dev portaudio19-dev libportaudio2 libportaudiocpp0 ffmpeg flac libatlas-base-dev python3-smbus i2c-tools libopenjp2-7
    ```

6.  **Install Rust Compiler:** Required by a Gemini library dependency. **This step can take over an hour.**
    ```bash
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
    ```
    *   Choose option `1) Proceed with installation (default)`.
    *   After installation, configure your shell:
        ```bash
        source "$HOME/.cargo/env"
        echo 'source "$HOME/.cargo/env"' >> ~/.bashrc
        ```

7.  **Increase Swap Space (Crucial):**
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

9.  **Install Python Libraries:** **This step will also take a very long time.**
    ```bash
    pip install --upgrade pip
    pip install RPi.GPIO spidev smbus2 Pillow numpy
    pip install google-generativeai SpeechRecognition gTTS pygame Flask google-cloud-speech
    ```

10. **Revert Swap Space & Reboot:**
    ```bash
    sudo dphys-swapfile swapoff
    sudo sed -i 's/CONF_SWAPSIZE=1024/CONF_SWAPSIZE=100/' /etc/dphys-swapfile
    sudo dphys-swapfile setup
    sudo dphys-swapfile swapon
    sudo reboot
    ```

11. **Verify I2C Sensor:** After rebooting and reconnecting via SSH, check if the VL6180X is detected at address `0x29`.
    ```bash
    sudo i2cdetect -y 1
    ```
    If `29` is not in the grid, double-check your wiring on the `I2C` header.

---

## **5. Code Implementation (Updated)**

Set up your project directory with the new and updated code files.

### **File 1: `DFRobot_VL6180X.py`** (Sensor Library)

This local library is required for the distance sensor.

**Create the file:**
```bash
cd ~/NinjaRobot
nano DFRobot_VL6180X.py
```
**Paste the following code:**
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
import smbus2 as smbus # Use smbus2 for better compatibility
import time

class DFRobot_VL6180X:
  VL6180X_IIC_ADDRESS = 0x29
  VL6180X_IDENTIFICATION_MODEL_ID = 0x000
  VL6180X_SYSTEM_FRESH_OUT_OF_RESET = 0x016
  VL6180X_SYSRANGE_START = 0x018
  VL6180X_RESULT_RANGE_VAL = 0x062
  VL6180X_ID = 0xB4

  def __init__(self, iic_addr=VL6180X_IIC_ADDRESS, bus=1):
    self._i2cbus = smbus.SMBus(bus)
    self._i2c_addr = iic_addr

  def begin(self):
    try:
      model_id = self._read_register(self.VL6180X_IDENTIFICATION_MODEL_ID)
      if model_id != self.VL6180X_ID:
        return False
      if self._read_register(self.VL6180X_SYSTEM_FRESH_OUT_OF_RESET) == 1:
        self._write_register(0x0207, 0x01)
        self._write_register(0x0208, 0x01)
        self._write_register(0x0096, 0x00)
        self._write_register(0x0097, 0xfd)
        self._write_register(0x00e3, 0x00)
        self._write_register(0x00e4, 0x04)
        self._write_register(0x00e5, 0x02)
        self._write_register(0x00e6, 0x01)
        self._write_register(0x00e7, 0x03)
        self._write_register(0x00f5, 0x02)
        self._write_register(0x00d9, 0x05)
        self._write_register(0x00db, 0xce)
        self._write_register(0x00dc, 0x03)
        self._write_register(0x00dd, 0xf8)
        self._write_register(0x009f, 0x00)
        self._write_register(0x00a3, 0x3c)
        self._write_register(0x00b7, 0x00)
        self._write_register(0x00bb, 0x3c)
        self._write_register(0x00b2, 0x09)
        self._write_register(0x00ca, 0x09)
        self._write_register(0x0198, 0x01)
        self._write_register(0x01b0, 0x17)
        self._write_register(0x01ad, 0x00)
        self._write_register(0x00ff, 0x05)
        self._write_register(0x0100, 0x05)
        self._write_register(0x0199, 0x05)
        self._write_register(0x01a6, 0x1b)
        self._write_register(0x01ac, 0x3e)
        self._write_register(0x01a7, 0x1f)
        self._write_register(0x0030, 0x00)
        self._write_register(0x014, 0x24) # SYSTEM_INTERRUPT_CONFIG_GPIO
        self._write_register(0x01C, 0x32) # SYSRANGE_MAX_CONVERGENCE_TIME
        self._write_register(0x02D, 0x11) # SYSRANGE_RANGE_CHECK_ENABLES
        self._write_register(0x022, 0x7B) # SYSRANGE_EARLY_CONVERGENCE_ESTIMATE
        self._write_register(0x040, 0x64) # SYSALS_INTEGRATION_PERIOD
        self._write_register(0x03F, 0x46) # SYSALS_ANALOGUE_GAIN
        self._write_register(0x011, 0x10) # SYSTEM_MODE_GPIO1
        self._write_register(self.VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x00)
      return True
    except IOError:
      return False

  def range_poll_measurement(self):
    self._write_register(self.VL6180X_SYSRANGE_START, 0x01)
    # A short delay is needed for the measurement to complete
    time.sleep(0.01) 
    return self._read_register(self.VL6180X_RESULT_RANGE_VAL)

  def _write_register(self, reg, data):
    self._i2cbus.write_i2c_block_data(self._i2c_addr, (reg >> 8) & 0xFF, [reg & 0xFF, data])
    
  def _read_register(self, reg):
    self._i2cbus.write_i2c_block_data(self._i2c_addr, (reg >> 8) & 0xFF, [reg & 0xFF])
    time.sleep(0.001)
    return self._i2cbus.read_byte(self._i2c_addr)
```
*Save and Exit (`Ctrl+X`, `Y`, `Enter`)*

### **File 2: `Ninja_Distance.py`** (Updated)

This file uses the new sensor library.

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
'''
import time
import sys
import DFRobot_VL6180X

sensor = None
sensor_initialized = False

def setup_sensor():
    """Initializes the VL6180X sensor."""
    global sensor, sensor_initialized
    if sensor_initialized: return True
    
    print("Initializing VL6180X distance sensor...")
    try:
        sensor = DFRobot_VL6180X.DFRobot_VL6180X(bus=1)
        if not sensor.begin():
            print("Error: Failed to initialize the VL6180X sensor.")
            return False
        
        print("VL6180X sensor initialized successfully.")
        sensor_initialized = True
        return True
    except Exception as e:
        print(f"An exception occurred during sensor setup: {e}")
        return False

def measure_distance():
    """
    Measures distance in centimeters (cm).
    Returns -1 on error/timeout.
    """
    if not sensor_initialized: return -1
        
    try:
        distance_mm = sensor.range_poll_measurement()
        if distance_mm == 255: return -1
        return distance_mm / 10.0
    except Exception as e:
        print(f"Error during measurement: {e}")
        return -1

# Main test block
if __name__ == "__main__":
    if not setup_sensor():
        sys.exit("Exiting due to sensor failure.")

    print("\n--- VL6180X Distance Sensor Test ---")
    try:
        while True:
            dist_cm = measure_distance()
            if dist_cm == -1: print("Status: Out of range or error")
            else: print(f"Distance: {dist_cm:.2f} cm")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
```
*Save and Exit*

### **File 3: `Ninja_Face_LCD.py`** (New File)

This new file controls the robot's face.

**Create the file:**
```bash
nano Ninja_Face_LCD.py
```
**Paste the following code:**
```python
# -*- coding: utf-8 -*
'''!
  @file Ninja_Face_LCD.py
  @brief Controls the Waveshare 2-inch SPI LCD to display expressions.
  @copyright Copyright (c) 2024
  @license The MIT License (MIT)
  @author Assistant
  @version V1.1
'''
import spidev
import RPi.GPIO as GPIO
import time
from PIL import Image, ImageDraw, ImageFont
import numpy as np

# --- Low-level LCD Driver Class ---
class WaveshareLCD:
    def __init__(self, rst_pin=17, dc_pin=25, bl_pin=24, cs_pin=8):
        self.RST_PIN, self.DC_PIN, self.BL_PIN, self.CS_PIN = rst_pin, dc_pin, bl_pin, cs_pin
        self.width, self.height = 240, 320
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in [self.RST_PIN, self.DC_PIN, self.BL_PIN, self.CS_PIN]:
            GPIO.setup(pin, GPIO.OUT)

        self.spi = spidev.SpiDev()
        self.spi.open(0, 0) # SPI bus 0, device 0 (CS0)
        self.spi.max_speed_hz = 40000000

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
        self._command(0x2A)
        self.spi.writebytes2([(x_start >> 8) & 0xFF, x_start & 0xFF, (x_end >> 8) & 0xFF, x_end & 0xFF])
        self._command(0x2B)
        self.spi.writebytes2([(y_start >> 8) & 0xFF, y_start & 0xFF, (y_end >> 8) & 0xFF, y_end & 0xFF])
        self._command(0x2C)

    def display(self, image):
        if image.width != self.width or image.height != self.height:
            image = image.resize((self.width, self.height))
        
        pixel_data = np.array(image.convert("RGB"), dtype=np.uint16)
        color = ((pixel_data[..., 0] & 0xF8) << 8) | ((pixel_data[..., 1] & 0xFC) << 3) | (pixel_data[..., 2] >> 3)
        
        self.set_window(0, 0, self.width - 1, self.height - 1)
        GPIO.output(self.DC_PIN, GPIO.HIGH)
        self.spi.writebytes2(color.tobytes('C'))

    def clear(self):
        black_image = Image.new("RGB", (self.width, self.height), "BLACK")
        self.display(black_image)

    def backlight_on(self): GPIO.output(self.BL_PIN, GPIO.HIGH)
    def backlight_off(self): GPIO.output(self.BL_PIN, GPIO.LOW)
    def cleanup(self):
        print("Cleaning up LCD resources.")
        self.clear(); self.backlight_off(); self.spi.close()

# --- High-level Face Drawing Class ---
class RobotFace:
    def __init__(self, lcd_driver):
        self.lcd = lcd_driver
        self.width, self.height = 320, 240 # Draw on a horizontal canvas
        try:
            self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24)
        except IOError:
            self.font = ImageFont.load_default()

    def _display(self, image):
        self.lcd.display(image.rotate(90, expand=True))

    def _create_base_image(self):
        return Image.new("RGB", (self.width, self.height), "BLACK")

    def _draw_eyes(self, draw, mood='idle', pupil_data=None):
        eye_y, r = 120, 45 # y-center, radius
        lx, rx = self.width // 2 - 80, self.width // 2 + 80
        draw.ellipse((lx - r, eye_y - r, lx + r, eye_y + r), fill="WHITE")
        draw.ellipse((rx - r, eye_y - r, rx + r, eye_y + r), fill="WHITE")
        if pupil_data:
            for i, (px, py, pr) in enumerate(pupil_data):
                ex = lx if i == 0 else rx
                draw.ellipse((ex + px - pr, eye_y + py - pr, ex + px + pr, eye_y + py + pr), fill="BLACK")

    def show_expression(self, mood='idle'):
        print(f"FACE: Setting expression to '{mood}'")
        image = self._create_base_image()
        draw = ImageDraw.Draw(image)
        pupils = [(0, 0, 15), (0, 0, 15)]

        if mood == 'idle': self.animate_blink(); return
        elif mood == 'happy':
            self._draw_eyes(draw, pupil_data=pupils)
            draw.arc((120, 180, 200, 240), 0, 180, fill="WHITE", width=10)
        elif mood == 'sad':
            pupils = [(0, 15, 12), (0, 15, 12)]
            self._draw_eyes(draw, pupil_data=pupils)
            draw.arc((130, 200, 190, 230), 180, 360, fill="WHITE", width=8)
        elif mood in ['angry', 'danger']:
            r = 45; eye_y=120; lx=self.width//2-80; rx=self.width//2+80
            draw.line((lx - r, eye_y - 20, lx + r, eye_y - 40), fill="WHITE", width=15)
            draw.line((rx + r, eye_y - 20, rx - r, eye_y - 40), fill="WHITE", width=15)
            draw.line((130, 210, 190, 210), fill="WHITE", width=8)
        elif mood == 'sleepy': self.animate_sleepy(); return
        else: self._draw_eyes(draw, pupil_data=pupils)
        
        self._display(image)

    def animate_blink(self):
        open_img = self._create_base_image()
        draw_open = ImageDraw.Draw(open_img)
        self._draw_eyes(draw_open, pupil_data=[(0, 0, 15), (0, 0, 15)])
        
        closed_img = self._create_base_image()
        draw_closed = ImageDraw.Draw(closed_img)
        r = 50; eye_y = 120; lx=self.width//2-80; rx=self.width//2+80
        draw_closed.arc((lx - r, eye_y - r, lx + r, eye_y + r), 200, 340, fill="WHITE", width=8)
        draw_closed.arc((rx - r, eye_y - r, rx + r, eye_y + r), 200, 340, fill="WHITE", width=8)

        self._display(closed_img); time.sleep(0.15); self._display(open_img)

    def animate_sleepy(self):
        img = self._create_base_image()
        draw = ImageDraw.Draw(img)
        r = 50; eye_y = 120; lx=self.width//2-80; rx=self.width//2+80
        draw.arc((lx - r, eye_y - r, lx + r, eye_y + r), 180, 360, fill="WHITE", width=8)
        draw.arc((rx - r, eye_y - r, rx + r, eye_y + r), 180, 360, fill="WHITE", width=8)
        self._display(img)```
*Save and Exit*

### **File 4: `ninja_core.py`** (Heavily Updated)

The core logic is updated to integrate the new face and sensor.

**Create/overwrite the file:**```bash
nano ninja_core.py
```
**Paste the following code:**
```python
# -*- coding:utf-8 -*-
'''!
  @file ninja_core.py
  @brief Core logic for the Ninja Robot with LCD face and ToF sensor.
  @copyright Copyright (c) 2024
  @license The MIT License (MIT)
  @author Assistant
  @version V2.1
'''
import sys, time, re, threading, json
import RPi.GPIO as GPIO
import google.generativeai as genai

# --- Configuration ---
GOOGLE_API_KEY = "PASTE_YOUR_GOOGLE_GEMINI_API_KEY_HERE"
GEMINI_MODEL_NAME = "gemini-1.5-flash-latest"
DISTANCE_THRESHOLD_CM = 15.0

# --- Import Robot Modules ---
try:
    import Ninja_Movements_v1 as movements, Ninja_Buzzer as buzzer, Ninja_Distance as distance, Ninja_Face_LCD as face_lcd
except ImportError as e:
    print(f"Error importing robot modules: {e}"); sys.exit(1)

# --- Global Variables ---
model, movement_thread, distance_check_thread, buzzer_pwm, face = (None,) * 5
is_continuous_moving, keep_distance_checking, hardware_initialized = (False,) * 3

# --- Mapping sounds to face expressions ---
SOUND_TO_FACE_MAP = {
    "hello": "happy", "thanks": "happy", "yes": "idle", "no": "sad",
    "danger": "danger", "scared": "danger", "exciting": "happy", "happy": "happy",
    "sleepy": "sleepy", "left": "idle", "right": "idle"
}

def initialize_gemini():
    global model
    if model: return True
    if "PASTE_YOUR" in GOOGLE_API_KEY:
        print("ERROR: Google Gemini API key not set in ninja_core.py."); return False
    try:
        genai.configure(api_key=GOOGLE_API_KEY)
        model = genai.GenerativeModel(GEMINI_MODEL_NAME)
        print("Gemini model loaded."); return True
    except Exception as e:
        print(f"Error loading Gemini model: {e}"); return False

def initialize_hardware():
    global buzzer_pwm, face, hardware_initialized
    if hardware_initialized: return True
    print("Initializing hardware...")
    try:
        lcd = face_lcd.WaveshareLCD(); lcd.init_display()
        face = face_lcd.RobotFace(lcd); face.show_expression('idle')
        movements.init_board_and_servo()
        buzzer.setup(); buzzer_pwm = GPIO.PWM(buzzer.BUZZER_PIN, 440); buzzer_pwm.start(0)
        distance.setup_sensor()
        hardware_initialized = True
        print("Hardware initialized."); movements.reset_servos(); play_robot_sound('hello'); time.sleep(1)
        return True
    except Exception as e:
        print(f"Hardware initialization error: {e}")
        try: GPIO.cleanup()
        except: pass
        return False

def cleanup_all():
    global keep_distance_checking, movement_thread, is_continuous_moving, hardware_initialized
    print("\n--- Initiating Cleanup ---")
    if hardware_initialized: play_robot_sound("sleepy"); time.sleep(1)
    keep_distance_checking = False
    if distance_check_thread and distance_check_thread.is_alive(): distance_check_thread.join(0.5)
    if is_continuous_moving or (movement_thread and movement_thread.is_alive()): movements.stop()
    if movement_thread and movement_thread.is_alive(): movement_thread.join(1.0)
    is_continuous_moving = False
    if hardware_initialized:
        try: movements.rest()
        except: pass
        if face: face.lcd.cleanup()
        if buzzer_pwm: buzzer_pwm.stop()
        GPIO.cleanup()
    hardware_initialized = False; print("Cleanup complete.")

def play_robot_sound(sound_keyword):
    if not hardware_initialized: return
    expression = SOUND_TO_FACE_MAP.get(sound_keyword.lower(), 'idle')
    if face: threading.Thread(target=face.show_expression, args=(expression,)).start()
    
    sound_action = buzzer.SOUND_MAP.get(sound_keyword.lower())
    try:
        if sound_action == buzzer.SOUND_SCARED_IDENTIFIER: buzzer.play_scared_sound(buzzer_pwm)
        elif sound_action == buzzer.SOUND_EXCITING_IDENTIFIER: buzzer.play_exciting_trill(buzzer_pwm)
        elif isinstance(sound_action, list): buzzer.play_sequence(buzzer_pwm, sound_action)
    except Exception as e: print(f"Error playing sound '{sound_keyword}': {e}")

def distance_checker():
    global keep_distance_checking, is_continuous_moving
    print("Distance checker thread started.")
    while keep_distance_checking:
        if not is_continuous_moving or movements.stop_movement or not hardware_initialized: break
        dist_cm = distance.measure_distance()
        if 0 < dist_cm < DISTANCE_THRESHOLD_CM:
            print(f"!!! OBSTACLE at {dist_cm:.1f} cm !!!")
            play_robot_sound('danger')
            if is_continuous_moving: movements.stop(); is_continuous_moving = False
            break 
        time.sleep(0.15)
    print("Distance checker thread finished."); keep_distance_checking = False

def process_user_command_with_gemini(command, lang='en-US'):
    if not model: return {"action_type": "unknown", "error": "Gemini not ready."}
    prompt = f"""Analyze the command for a robot. Respond in JSON only.
Is it a general question or a robot command?
- Question (e.g., 'what is the weather'): {{"action_type": "conversation", "response_text": "your answer"}}.
- Command: Use keys "action_type", "move_function", "speed", "sound_keyword".
Functions: 'hello', 'walk', 'stepback', 'run', 'runback', 'turnleft_step', 'turnright_step', 'rotateleft', 'rotateright', 'stop', 'reset_servos', 'rest'.
Sounds: 'hello', 'thanks', 'no', 'yes', 'danger', 'exciting', 'happy', 'right', 'left', 'scared', 'sleepy'.
Command: "{command}" -> """
    try:
        response = model.generate_content(prompt)
        match = re.search(r'\{.*\}', response.text, re.DOTALL)
        if match: return json.loads(match.group(0))
        return {"action_type": "unknown", "error": f"No JSON in response: {response.text}"}
    except Exception as e: return {"action_type": "unknown", "error": f"API error: {e}"}

def execute_action(action_data, lang='en-US'):
    global movement_thread, is_continuous_moving, keep_distance_checking
    if not hardware_initialized: print("Error: Hardware not ready."); return
    
    action_type = action_data.get("action_type", "unknown")

    if action_type == "conversation":
        print(f"CONVERSATIONAL: {action_data.get('response_text', '...')}")
        play_robot_sound('yes'); return

    move_func = action_data.get("move_function")
    if move_func and is_continuous_moving:
        movements.stop()
        if movement_thread and movement_thread.is_alive(): movement_thread.join(1.0)
        is_continuous_moving = False; time.sleep(0.2)
    
    if action_data.get("sound_keyword"):
        play_robot_sound(action_data["sound_keyword"])
        if move_func: time.sleep(0.3)

    if move_func:
        target_func = getattr(movements, move_func, None)
        if target_func:
            speed = action_data.get("speed", "normal")
            is_continuous = move_func in ["walk", "run", "stepback", "runback", "rotateleft", "rotateright"]
            if is_continuous:
                is_continuous_moving = True; movements.stop_movement = False
                movement_thread = threading.Thread(target=target_func, args=(speed, None), daemon=True); movement_thread.start()
                if move_func in ["walk", "run"]:
                    keep_distance_checking = True
                    distance_check_thread = threading.Thread(target=distance_checker, daemon=True); distance_check_thread.start()
            elif move_func == "stop": movements.stop(); is_continuous_moving = False
            else: target_func()
        else: print(f"Error: Func '{move_func}' not found."); play_robot_sound('no')
    
    if action_type == "unknown": play_robot_sound('no')

def get_robot_status():
    if not hardware_initialized: return "Hardware Offline"
    return "Moving..." if is_continuous_moving else "Idle"

# Main test block
if __name__ == "__main__":
    if not initialize_gemini() or not initialize_hardware():
        sys.exit("System initialization failed.")
    try:
        while True:
            cmd = input("🎤> ").strip()
            if cmd.lower() in ["exit", "quit"]: break
            if cmd: execute_action(process_user_command_with_gemini(cmd))
    except KeyboardInterrupt: print("\nCtrl+C detected.")
    finally: cleanup_all()
```
*Save and Exit*

### **Remaining Files (Unchanged)**

The following files from your original upload do not require any changes:

*   `Ninja_Movements_v1.py`
*   `Ninja_Buzzer.py`
*   `web_interface.py`
*   `templates/index.html`

Ensure your Gemini API key is pasted into `ninja_core.py`.

---

## **6. Running the Application**

1.  **Activate Environment:**
    ```bash
    cd ~/NinjaRobot
    source .venv/bin/activate
    ```
2.  **Find Pi's IP Address:**
    ```bash
    hostname -I | awk '{print $1}'
    ```
3.  **Start the Web Server:**
    ```bash
    python3 web_interface.py
    ```
    *The LCD screen will light up and display the robot's idle face. The terminal will show initialization messages.*

4.  **Access Web Interface:** On another device on the same WiFi network, open a browser and go to:
    `http://<YOUR_PI_IP_ADDRESS>:5000`

---

## **7. Using the Interface**

The interface functions as before, but now with enhanced feedback from the robot's new components.

*   **LCD Face:** The robot's face will change expression to match the sound or action being performed (e.g., happy for "hello," angry for "danger," sleepy for "rest"). It will blink periodically when idle.
*   **Distance Sensor:** When the robot is moving forward (`walk` or `run`), it will now automatically stop and play a "danger" sound if it detects an obstacle closer than 15 cm.

---

## **8. Troubleshooting**

*   **LCD Not Working:**
    *   **Check wiring carefully!** `DC`->`D25`, `RST`->`D17`, `BL`->`D24`. Also check the `SPI` header connections (`MOSI`, `SCK`, `CS`).
    *   Ensure SPI is enabled (`sudo raspi-config`).
*   **Distance Sensor Error / `i2cdetect` fails:**
    *   Check the 4 wires are correctly plugged into the dedicated `I2C` header on the HAT.
    *   Ensure I2C is enabled (`sudo raspi-config`).
*   **`ModuleNotFoundError`:**
    *   Make sure your virtual environment is active (`source .venv/bin/activate`).
    *   Ensure all libraries from Step 4.9 were installed successfully.
*   **AI Errors / API Key Issues:**
    *   Make sure you have pasted your correct Google Gemini API key into `ninja_core.py`.
*   **Robot movement is erratic:**
    *   Check that your power supply is at least 5V, 2.5A. The LCD adds to the power draw.
    *   Verify the 360° and 180° servos are connected to the correct `S0-S3` ports.
