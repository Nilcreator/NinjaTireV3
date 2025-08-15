# Build Your Own AI NinjaRobot: A Comprehensive Guide

Welcome to the NinjaRobot project! This guide will walk you through every step of building an intelligent, expressive robot using a Raspberry Pi Zero 2 W. This project is designed to be a comprehensive learning platform, integrating mechanical assembly, modern electronics, and AI-powered software.

By the end of this guide, you will have a robot that can:

*   **Move with Precision:** Using a custom-calibrated servo driver for accurate and repeatable movements.
*   **Perceive its Environment:** With a modern Time-of-Flight (ToF) distance sensor.
*   **Show Expressions:** Using a vibrant SPI LCD screen as an interactive face.
*   **Be Controlled Remotely:** Through a modern web interface built with FastAPI.
*   **Understand Commands:** Leveraging Google's Gemini AI for natural language processing.

This guide is designed for everyone, from beginners taking their first steps in robotics to experienced makers looking for a fun and expandable project. Let's begin!

---

## Part 1: Hardware Assembly

Correctly assembling the hardware is the foundation of our project. We will list the required components and provide a clear wiring diagram.

### Required Components

| Component | Quantity | Notes & Links |
| :--- | :--- | :--- |
| **Raspberry Pi Zero 2 W** | 1 | The "brain" of our robot. [Product Brief](https://akizukidenshi.com/goodsaffix/raspberry-pi-zero-2-w-product-brief.pdf) |
| **DFRobot IO HAT for Zero** | 1 | Simplifies connections to all components. [Wiki Page](https://wiki.dfrobot.com/I_O_Expansion_HAT_for_Pi_zero_V1_0_SKU_DFR0604) |
| **SG90 Servo Motors** | 4 | 2x 180° for legs, 2x 360° for feet/wheels. [Datasheet](https://wikifactory.com/@islaytx/open-edu/v/0b7f3f6/file/4_Mechanics/sg90_datasheet.pdf) |
| **VL6180X Distance Sensor** | 1 | Time-of-Flight sensor for distance measurement. [Wiki Page](https://wiki.dfrobot.com/DFRobot_VL6180X_TOF_Distance_Ranging_Sensor_Breakout_Board_SKU_SEN0427) |
| **Waveshare 2" SPI LCD** | 1 | The robot's expressive face. [Wiki Page](https://www.waveshare.com/wiki/2inch_LCD_Module) |
| **Buzzer** | 1 | For sound feedback. An active buzzer is easiest. [Example Datasheet](https://www.soberton.com/wp-content/uploads/2019/02/WST-1206BX-17-Feb-2019.pdf) |
| **MicroSD Card** | 1 | 16GB or higher, Class 10 recommended. |
| **Power Supply** | 1 | 5V, 2.5A (or higher) Micro USB adapter. |

### Wiring Diagram

**Safety First! Always ensure your Raspberry Pi is completely powered off and unplugged before connecting or disconnecting any components.**

1.  Carefully mount the **DFRobot IO Expansion HAT** onto your Raspberry Pi Zero's 40-pin GPIO header. Ensure it is seated firmly and evenly.
2.  All subsequent connections will be made to the pins on the DFRobot HAT.

| Component | Pin Function | Connection to DFRobot HAT Pin Label | Raspberry Pi Pin (BCM) |
| :--- | :--- | :--- | :--- |
| **Servos (x4)** | PWM Signal | `S0`, `S1`, `S2`, `S3` (or any 4 from 0-15) | N/A (HAT Controlled) |
| **Buzzer** | Signal | `D23` | GPIO 23 |
| **I2C Devices** | | *(Shared I2C Port on HAT)* | |
| &boxvr;&nbsp; **VL6180X Sensor** | Power (VIN) | `3.3V` | 3.3V Power |
| | Ground (GND) | `GND` | Ground |
| | Clock (SCL) | `SCL` | GPIO 3 |
| | Data (SDA) | `SDA` | GPIO 2 |
| **SPI Display** | | *(Pass-through GPIO Headers)* | |
| &boxvr;&nbsp; **Waveshare 2" LCD** | Power (VCC) | `3.3V` | 3.3V Power |
| | Ground (GND) | `GND` | Ground |
| | Data In (DIN) | `SPI_MOSI` | GPIO 10 |
| | Clock (CLK) | `SPI_SCLK` | GPIO 11 |
| | Chip Select (CS) | `SPI_CE0` | GPIO 8 |
| | Data/Command (DC) | `D25` | GPIO 25 |
| | Reset (RST) | `D17` | GPIO 17 |
| | Backlight (BL) | `D18` | GPIO 18 |

### Explanation of Part 1

*   **What We Did:** We identified all the necessary electronic parts and created a master plan for connecting them to the Raspberry Pi via the DFRobot Expansion HAT.
*   **Why We Did It:** A correct and stable hardware connection is the most critical first step.
    *   The **Raspberry Pi** is a mini-computer that runs our code.
    *   The **DFRobot HAT** is like a "power strip" for robotics; it makes it much easier and safer to connect multiple components like servos, sensors, and displays.
    *   Each component communicates using a different "language" or protocol. The **VL6180X sensor** uses **I2C**, a simple two-wire bus perfect for low-speed data. The **LCD screen** uses **SPI**, a faster protocol needed to send the large amount of data required for displaying images. Our wiring plan ensures that each component is connected to the correct pins for its specific protocol.

---

## Part 2: Raspberry Pi Installation & Setup

Now, we will prepare the Raspberry Pi's software environment from a blank microSD card.

### Step 1: Install Raspberry Pi OS (Headless)

This "headless" method lets you set up the Pi from your main computer without needing a dedicated monitor or keyboard for the Pi.

1.  On your main computer, download and install the official **Raspberry Pi Imager** from [raspberrypi.com/software](https://www.raspberrypi.com/software/).
2.  Launch the Imager.
    *   **Device:** Select `Raspberry Pi Zero 2 W`.
    *   **Operating System:** Click `CHOOSE OS` > `Raspberry Pi OS (other)` > **`Raspberry Pi OS (Legacy, 32-bit)`**. This version ("Bullseye") offers excellent stability for our hardware libraries.
3.  Click the **gear icon (⚙️)** to pre-configure the OS:
    *   Set a **hostname** (e.g., `ninjarobot.local`).
    *   **Enable SSH** and use password authentication.
    *   Set a memorable **username and password**.
    *   **Configure your Wi-Fi** with its name (SSID) and password.
4.  **Save** the settings.
5.  Insert your microSD card, select it under **Storage**, and click **WRITE**.

### Step 2: System Configuration

1.  Once writing is complete, insert the SD card into the Pi and power it on. Wait a few minutes.
2.  Connect to the Pi from your computer's terminal:
    ```bash
    ssh your_username@ninjarobot.local
    ```
3.  Update the system software:
    ```bash
    sudo apt update
    sudo apt full-upgrade -y
    sudo apt install -y curl git
    ```
4.  Enable the hardware interfaces for our new sensor and LCD:
    ```bash
    sudo raspi-config
    ```
    *   Go to `3 Interface Options` -> `I5 SPI` -> choose `<Yes>`.
    *   Go back to `3 Interface Options` -> `I4 I2C` -> choose `<Yes>`.
    *   Select `<Finish>` and reboot when prompted. Reconnect via SSH after it restarts.

### Step 3: Install Dependencies & Build Tools

These are system-level tools needed to install our Python software. **This part can take a long time on a Pi Zero.**

1.  Install essential libraries:
    ```bash
    sudo apt install -y python3-dev python3-pip python3-venv build-essential libasound2-dev portaudio19-dev libportaudiocpp0 ffmpeg flac libatlas-base-dev python3-smbus i2c-tools
    ```
2.  Install the Rust compiler (a requirement for the Gemini AI library):
    ```bash
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
    ```
    *   When prompted, choose `1) Proceed with installation (default)`.
    *   After it finishes, configure your current terminal session with the provided command:
        ```bash
        source "$HOME/.cargo/env"
        ```
3.  **Crucially, increase virtual memory (swap)** to prevent crashes during installation:
    ```bash
    sudo dphys-swapfile swapoff
    echo "CONF_SWAPSIZE=1024" | sudo tee /etc/dphys-swapfile
    sudo dphys-swapfile setup
    sudo dphys-swapfile swapon
    ```

### Step 4: Setup Python Project Environment

1.  Create a project folder and a Python virtual environment inside it:
    ```bash
    mkdir ~/NinjaRobot
    cd ~/NinjaRobot
    python3 -m venv .venv
    ```
2.  **Activate** the environment. You must do this every time you work on the project.
    ```bash
    source .venv/bin/activate
    ```
    *(Your terminal prompt will now start with `(.venv)`)*
3.  **Install all Python libraries.** This single command installs everything needed for the robot. **This is the longest step and may take over an hour.**
    ```bash
    pip install --upgrade pip
    pip install smbus2 RPi.GPIO Pillow numpy spidev google-generativeai SpeechRecognition gTTS pygame "fastapi[all]" "uvicorn[standard]"
    ```
4.  **Revert swap space** to protect your SD card and reboot:
    ```bash
    sudo dphys-swapfile swapoff
    echo "CONF_SWAPSIZE=100" | sudo tee /etc/dphys-swapfile
    sudo dphys-swapfile setup
    sudo dphys-swapfile swapon
    sudo reboot
    ```
5.  After rebooting, reconnect and activate your environment again: `cd ~/NinjaRobot && source .venv/bin/activate`.

### Explanation of Part 2

*   **What We Did:** We installed the operating system, enabled remote access, updated all the base software, enabled the I2C and SPI hardware "ports", and installed all the necessary Python code libraries inside an isolated "virtual environment."
*   **Why We Did It:** This process creates a clean, stable, and correctly configured software foundation.
    *   The **headless setup** is essential for working with a mobile robot.
    *   Enabling **I2C/SPI** is like flipping a switch that allows our code to talk to the sensor and screen.
    *   The **virtual environment** prevents the robot's specific libraries from conflicting with system software, making the project more reliable and easier to maintain.
    *   Increasing **swap space** temporarily gives the tiny Pi Zero the breathing room it needs to complete the demanding, one-time installation of complex software libraries.

---

## Part 3: Calibrating the Servos (`ninja_servo_init.py`)

Before the robot can move correctly, we must teach it about its own motors. This calibration process is the most important step for achieving precise movement.

### Why Calibration is Essential

Not all servo motors are created equal. Due to tiny manufacturing differences, the signal that makes one servo go to 90° might make another go to 92° or 88°. The `ninja_servo_init.py` script is a tool that lets you find the *exact* signal for each servo's minimum, center, and maximum positions. It saves this information to a `servo_calibration.json` file, which acts as the robot's "muscle memory" for all future movements.

### The Servo Driver Code

Create a file named `ninja_servo_init.py` inside your `~/NinjaRobot` directory and paste the complete code below into it.

```python
# -*- coding: utf-8 -*-
"""
ninja_servo_init.py (v1.4 - Corrected)

A general-purpose servo calibration and testing tool for the NinjaRobot project.
This script allows users to:
1. Define a custom set of servos with names, pins, and types (180/360).
2. Interactively calibrate the min, center, and max positions with fine-grained control.
3. Save the calibration data to a 'servo_calibration.json' file.
4. Erase existing calibration data to start fresh.
5. Enter an interactive testing phase after calibration to validate the results.

New Features in v1.4:
- Corrected a bug where 'm' and 'M' commands were treated the same.
  The command for Maximum is now 'x'.
- Reworked the testing loop to hold the servo's position until the next
  command is entered, instead of returning to center after a short delay.

Author: Your Name/Assistant
Date: 2024-05-27
Version: 1.4
"""
import time
import json
import os
import curses

# Attempt to import the DFRobot library
try:
    from DFRobot_RaspberryPi_Expansion_Board import DFRobot_Expansion_Board_IIC as Board
    from DFRobot_RaspberryPi_Expansion_Board import DFRobot_Expansion_Board_Servo as Servo
except ImportError:
    print("Error: DFRobot library not found.")
    print("Please ensure the 'DFRobot_RaspberryPi_Expansion_Board' directory is in your project folder.")
    exit()

# --- Global Variables ---
CALIBRATION_FILE = 'servo_calibration.json'
board = None
servo = None
servos_to_calibrate = []

# --- Core Hardware and Data Functions ---

def init_hardware():
    """Initializes the DFRobot Expansion Board and Servo controller."""
    global board, servo
    board = Board(1, 0x10)
    while board.begin() != board.STA_OK:
        print("Error: DFRobot Board connection failed. Check wiring and I2C.")
        print("Retrying in 3 seconds...")
        time.sleep(3)
    print("DFRobot Board connection successful.")
    servo = Servo(board)
    servo.begin()
    print("Servo controller initialized.")

def load_calibration_data():
    """Loads existing calibration data from the JSON file if it exists."""
    if os.path.exists(CALIBRATION_FILE):
        with open(CALIBRATION_FILE, 'r') as f:
            print(f"Loaded existing data from {CALIBRATION_FILE}")
            return json.load(f)
    return {}

def save_calibration_data(data):
    """Saves the calibration data to the JSON file."""
    with open(CALIBRATION_FILE, 'w') as f:
        json.dump(data, f, indent=4)
    print(f"\nCalibration data saved successfully to {CALIBRATION_FILE}")

def get_servo_setup_from_user():
    """Interactively prompts the user to define servos and handles the 'erase' command."""
    global servos_to_calibrate
    print("\n--- Ninja Robot Servo Setup ---")
    print("Define servos using format: name,pin,type (e.g., rt0,0,180)")
    print("Type 'erase' to delete current settings, or 'done' to finish setup.\n")

    while True:
        user_input = input(f"Define servo #{len(servos_to_calibrate) + 1} (or 'done'/'erase'): ").strip().lower()
        if user_input == 'done':
            if not servos_to_calibrate:
                print("No servos defined. Exiting.")
                return False
            print("\nServo setup complete. Proceeding to calibration...")
            time.sleep(1)
            return True
        if user_input == 'erase':
            if os.path.exists(CALIBRATION_FILE):
                confirm = input(f"Are you sure you want to delete '{CALIBRATION_FILE}'? (y/n): ").lower()
                if confirm == 'y':
                    os.remove(CALIBRATION_FILE)
                    print(f"'{CALIBRATION_FILE}' has been deleted.")
                    servos_to_calibrate = []
                else:
                    print("Erase cancelled.")
            else:
                print(f"'{CALIBRATION_FILE}' does not exist. Nothing to erase.")
            continue
        try:
            name, pin_str, type_str = user_input.split(',')
            pin, servo_type = int(pin_str), int(type_str)
            if not (0 <= pin <= 15) or servo_type not in [180, 360]:
                raise ValueError("Invalid pin or type.")
            if any(s['name'] == name or s['pin'] == pin for s in servos_to_calibrate):
                raise ValueError("Duplicate name or pin.")
            servos_to_calibrate.append({'name': name, 'pin': pin, 'type': servo_type})
            print(f"  -> Added '{name}' (Pin: {pin}, Type: {servo_type}°)")
        except ValueError as e:
            print(f"Error: {e}. Please use format: name,pin,type (e.g., lf3,3,180)")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

# --- Calibration UI and Logic ---

def calculate_current_degree(current_value, servo_data):
    """Calculates the calibrated degree based on the raw value and set points."""
    stype = servo_data.get('type')
    min_key, center_key, max_key = ("min", "center", "max") if stype == 180 else ("ccw_max", "stop", "cw_max")
    min_val, center_val, max_val = servo_data.get(min_key), servo_data.get(center_key), servo_data.get(max_key)
    if center_val is None: return "N/A"
    try:
        if current_value >= center_val and max_val is not None:
            span = max_val - center_val
            if span <= 0: return "+90.0°" if current_value >= max_val else "0.0°"
            return f"+{90.0 * (current_value - center_val) / span:.1f}°"
        elif current_value < center_val and min_val is not None:
            span = center_val - min_val
            if span <= 0: return "-90.0°" if current_value <= min_val else "0.0°"
            return f"{-90.0 * (center_val - current_value) / span:.1f}°"
    except (TypeError, ZeroDivisionError): return "N/A"
    return "Set points needed"

def draw_ui(stdscr, servo_info, cal_data, current_value):
    """Renders the Text-based User Interface using curses."""
    stdscr.clear(); h, w = stdscr.getmaxyx()
    name, pin, stype = servo_info['name'], servo_info['pin'], servo_info['type']
    s_data = cal_data.get(name, {})
    title = f"--- Calibrating '{name}' (Pin: {pin}, Type: {stype}°) ---"
    stdscr.addstr(0, (w - len(title)) // 2, title, curses.A_BOLD)
    stdscr.addstr(2, 2, f"Current Raw Value: {current_value:.1f}")
    stdscr.addstr(3, 2, f"Calibrated Position: {calculate_current_degree(current_value, s_data)}", curses.A_REVERSE)
    stdscr.addstr(5, 2, "Saved Points:", curses.A_UNDERLINE)
    min_label, center_label, max_label = ("-90° (Min)", "0° (Center)", "+90° (Max)") if stype == 180 else ("CCW Max (-90)", "Stop (0)", "CW Max (+90)")
    min_key, center_key, max_key = ("min", "center", "max") if stype == 180 else ("ccw_max", "stop", "cw_max")
    stdscr.addstr(6, 4, f"{min_label:14}: {s_data.get(min_key, 'Not Set')}")
    stdscr.addstr(7, 4, f"{center_label:14}: {s_data.get(center_key, 'Not Set')}")
    stdscr.addstr(8, 4, f"{max_label:14}: {s_data.get(max_key, 'Not Set')}")
    stdscr.addstr(h - 7, 2, "Controls:", curses.A_UNDERLINE)
    stdscr.addstr(h - 6, 4, "'w'/'s': Coarse Adjust(+/- 1.0) | 'e'/'d': Fine Adjust(+/- 0.1)")
    stdscr.addstr(h - 4, 4, f"'Shift+X': Set current value as {min_label}.")
    stdscr.addstr(h - 3, 4, f"'Shift+C': Set current value as {center_label}.")
    stdscr.addstr(h - 2, 4, f"'Shift+V': Set current value as {max_label}.")
    stdscr.addstr(h - 1, 4, "'n': Next Servo | 'q': Save & Quit", curses.A_BOLD)
    stdscr.refresh()

def calibration_loop(stdscr, servo_info, calibration_data):
    """Main interactive loop for calibrating a single servo."""
    name, pin, stype = servo_info['name'], servo_info['pin'], servo_info['type']
    if name not in calibration_data:
        calibration_data[name] = {'pin': pin, 'type': stype}
    center_key = "center" if stype == 180 else "stop"
    current_value = float(calibration_data[name].get(center_key, 90.0))
    servo.move(pin, int(round(current_value)))
    while True:
        draw_ui(stdscr, servo_info, calibration_data, current_value)
        key = stdscr.getkey()
        if key == 'w': current_value += 1.0
        elif key == 's': current_value -= 1.0
        elif key == 'e': current_value += 0.1
        elif key == 'd': current_value -= 0.1
        elif key == 'X':
            key_name = "min" if stype == 180 else "ccw_max"
            calibration_data[name][key_name] = round(current_value, 1)
        elif key == 'C':
            calibration_data[name][center_key] = round(current_value, 1)
        elif key == 'V':
            key_name = "max" if stype == 180 else "cw_max"
            calibration_data[name][key_name] = round(current_value, 1)
        elif key in ('n', 'q'):
            center_val = calibration_data[name].get(center_key, 90)
            servo.move(pin, int(round(center_val)))
            return 'quit' if key == 'q' else 'next'
        current_value = max(0.0, min(180.0, current_value))
        servo.move(pin, int(round(current_value)))

def main_calibration_ui(stdscr):
    """The main function wrapped by curses to run the calibration UI."""
    curses.curs_set(0); stdscr.nodelay(False); stdscr.timeout(-1)
    calibration_data = load_calibration_data()
    for servo_info in servos_to_calibrate:
        if calibration_loop(stdscr, servo_info, calibration_data) == 'quit':
            break
    save_calibration_data(calibration_data)

# --- Testing Phase Functions ---

def center_all_servos(calibration_data):
    """Moves all defined servos to their calibrated center/stop position."""
    print("Centering all servos...")
    for name, data in calibration_data.items():
        pin = data.get('pin')
        center_key = 'center' if data.get('type') == 180 else 'stop'
        center_val = data.get(center_key)
        if pin is not None and center_val is not None:
            servo.move(pin, int(round(center_val)))
    time.sleep(0.5)

def run_testing_loop(calibration_data):
    """Runs an interactive loop to test calibrated servo positions."""
    print("\n--- Servo Calibration Testing ---")
    print("Test servos using format: name:position")
    print("  - 'name': The name of the servo (e.g., rt0).")
    print("  - 'position': 'm' (min), 'c' (center), or 'x' (maX).")
    print("Chain commands with a semicolon ';'. Example: rt0:m; lf3:x")
    print("Type 'done' to finish testing.\n")

    while True:
        command_str = input("Enter test command (or 'done'): ").strip().lower()
        if command_str == 'done':
            break
            
        center_all_servos(calibration_data)
        
        commands = [cmd.strip() for cmd in command_str.split(';')]
        for cmd in commands:
            try:
                name, pos_key = [p.strip() for p in cmd.split(':')]
                if name not in calibration_data:
                    print(f"  Error: Servo '{name}' not found in calibration data.")
                    continue
                
                s_data = calibration_data[name]
                stype = s_data.get('type')
                pin = s_data.get('pin')

                if pos_key == 'm': key = "min" if stype == 180 else "ccw_max"
                elif pos_key == 'c': key = "center" if stype == 180 else "stop"
                elif pos_key == 'x': key = "max" if stype == 180 else "cw_max"
                else:
                    print(f"  Error: Invalid position '{pos_key}'. Use 'm', 'c', or 'x'.")
                    continue
                
                target_val = s_data.get(key)
                if target_val is None:
                    print(f"  Error: Position '{key}' is not set for servo '{name}'.")
                else:
                    print(f"  Moving '{name}' to {key} ({target_val})...")
                    servo.move(pin, int(round(target_val)))

            except ValueError:
                print(f"  Error: Invalid command format '{cmd}'. Use 'name:position'.")
            except Exception as e:
                print(f"  An unexpected error occurred: {e}")

# --- Main Execution ---

if __name__ == "__main__":
    try:
        # First, download the required library if it's not present
        if not os.path.exists('DFRobot_RaspberryPi_Expansion_Board'):
            print("DFRobot library not found. Cloning from GitHub...")
            os.system('git clone https://github.com/DFRobot/DFRobot_RaspberryPi_Expansion_Board.git')

        if get_servo_setup_from_user():
            init_hardware()
            curses.wrapper(main_calibration_ui)
            
            final_calibration_data = load_calibration_data()
            if not final_calibration_data:
                print("No calibration data found. Exiting.")
            else:
                test_choice = input("\nCalibration complete. Do you want to test the results? (y/n): ").lower()
                if test_choice == 'y':
                    run_testing_loop(final_calibration_data)
                
                print("\nProcess finished. Setting servos to center.")
                center_all_servos(final_calibration_data)
                print("Done.")

    except KeyboardInterrupt:
        print("\nProgram interrupted by user. Exiting.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()

```

### How to Use the Calibration and Testing Tool

1.  **Run the script:**
    ```bash
    python3 ninja_servo_init.py
    ```

2.  **Phase 1: Define Your Servos**
    *   You will be prompted to define your robot's servos one by one.
    *   Use the format `name,pin,type`. For example:
        *   `rt0,0,180` (Right Thigh, on pin 0, 180-degree type)
        *   `lf3,3,180` (Left Foot, on pin 3, 180-degree type)
        *   `rw2,2,360` (Right Wheel, on pin 2, 360-degree type)
    *   Type `done` when you are finished. You can also type `erase` at any time to start over.

3.  **Phase 2: Calibrate Each Servo**
    *   A full-screen interface will appear for the first servo.
    *   Use the keys to find and set the three key positions:
        *   `w`/`s`: Move the servo by `1.0`.
        *   `e`/`d`: Move the servo by `0.1` for fine-tuning.
        *   `Shift+X`: Set the current position as the **Minimum** (`-90°` or `CCW Max`).
        *   `Shift+C`: Set the current position as the **Center** (`0°` or `Stop`).
        *   `Shift+V`: Set the current position as the **Maximum** (`+90°` or `CW Max`).
    *   Press `n` to save and move to the next servo.
    *   Press `q` to save all progress and exit the calibration phase.

4.  **Phase 3: Test Your Calibration**
    *   After quitting the calibrator, you will be asked if you want to test. Type `y`.
    *   You will enter a new command prompt. The robot will hold its position while you type.
    *   **Before** each command runs, the servos will all reset to their center point.
    *   Enter commands like:
        *   `rt0:m` (Move `rt0` to its calibrated minimum)
        *   `lf3:x` (Move `lf3` to its calibrated maximum)
        *   `rt0:m; lf3:x` (Move both at the same time)
    *   Type `done` to finish testing. The servos will center one last time, and the program will end.
