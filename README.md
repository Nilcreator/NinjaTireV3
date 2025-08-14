### 1. Evaluation of Validation and Potential Conflicts

This analysis breaks down the compatibility and necessary changes for integrating the new VL6180X sensor and the Waveshare SPI LCD.

#### A. VL6180X Distance Sensor (Replacing HC-SR04)

*   **Validation:** The VL6180X is an excellent upgrade. As an I2C sensor, it is more reliable and uses fewer GPIO pins than the HC-SR04, which requires precise timing on two separate GPIO pins.
*   **Interface Change:** The most significant change is the move from a GPIO-based pulse-timing mechanism (`Ninja_Distance.py` for HC-SR04) to an I2C communication protocol.
*   **Software Impact:**
    *   The existing `Ninja_Distance.py` file must be completely replaced with logic for the VL6180X. The provided `VL6180DistanceSensor.md` contains the necessary `DFRobot_VL6180X.py` library and usage examples.
    *   The `ninja_core.py` script's `distance_checker` function will need to be updated to call the new I2C-based measurement function.
*   **Pin Impact:** This is a positive change. The HC-SR04 used `GPIO 21` and `GPIO 22`. **These two pins will be freed up**, providing more flexibility for future expansions. The VL6180X will connect to the shared I2C bus (SDA/SCL), which is already in use by the DFRobot HAT and enabled in the base setup.

#### B. Waveshare 2-inch SPI LCD

*   **Validation:** The LCD is fully compatible with the Raspberry Pi and the DFRobot HAT's pass-through headers. It will serve as an excellent robot face.
*   **Interface Change:** This component uses the SPI (Serial Peripheral Interface) bus, which was not used in the original `NinjaTire.md` project.
*   **Software Impact:**
    *   **New Libraries:** You will need to install new Python libraries that were not part of the original project: `spidev` (for SPI communication), `Pillow` (for image manipulation), and `numpy` (for data processing).
    *   **New Code Modules:** The project will need the `lcd_driver.py` and `robot_face.py` files as described in `SPILCD.md`.
    *   **System Configuration:** The SPI interface must be enabled on the Raspberry Pi using the `sudo raspi-config` tool. The original tutorial already covers enabling I2C; a similar step for SPI must be added.
    *   **Core Logic Integration:** `ninja_core.py` will need to be modified to import the new face module and include functions to control the facial expressions (e.g., `set_face_expression('happy')`).
*   **Pin Impact:** The LCD requires a number of pins. I have checked them against the existing project's pin usage.

#### C. Critical Finding: Pin Conflict

There is one major pin conflict between the new LCD and the optional I2S microphone from the original `NinjaTire.md` tutorial.

*   **Conflict:** **GPIO 18**
    *   The **Waveshare LCD** requires `GPIO 18` for its backlight (**BL**) control.
    *   The **INMP441 I2S Microphone** requires `GPIO 18` for its clock signal (**SCK**).

*   **Resolution & Recommendation:**
    The LCD face is a primary goal of this upgrade. The I2S microphone was listed as an optional, advanced component in the original tutorial, with the primary input method being the browser's microphone via the web UI.

    Therefore, **we will prioritize the LCD**. The recommended wiring below will assign `GPIO 18` to the LCD. This means the I2S microphone, as configured in the original guide, **cannot be used simultaneously**.

---

### 2. Suggested New Hardware Wiring and Pin Arrangement

Here is the new, consolidated wiring plan for the upgraded NinjaRobot. This arrangement ensures all core components (including the new sensor and LCD) can operate together without conflict. Please follow this plan carefully.

**Always disconnect the power supply before changing any wiring.**

| Component              | Pin Function        | Connection to DFRobot HAT Pin Label | Raspberry Pi Pin (BCM) | Notes                                                                   |
| :--------------------- | :------------------ | :---------------------------------- | :--------------------- | :---------------------------------------------------------------------- |
| **Servos (x4)**        | PWM Signal          | `S0`, `S1`, `S2`, `S3`              | N/A (HAT Controlled)   | No change. Connect to dedicated 3-pin servo headers.                    |
| **Buzzer**             | Signal              | `D23`                               | GPIO 23                | No change. Also connect VCC and GND.                                    |
|                        |                     |                                     |                        |                                                                         |
| **I2C Devices**        |                     | (Shared I2C Port)                   |                        |                                                                         |
| *VL6180X Sensor*       | Power (VIN)         | `3.3V`                              | 3.3V Power             | Connect to one of the I2C breakout ports on the HAT.                    |
|                        | Ground (GND)        | `GND`                               | Ground                 |                                                                         |
|                        | Clock (SCL)         | `SCL`                               | GPIO 3                 |                                                                         |
|                        | Data (SDA)          | `SDA`                               | GPIO 2                 |                                                                         |
|                        |                     |                                     |                        |                                                                         |
| **SPI Display**        |                     | (Pass-through GPIO Headers)         |                        |                                                                         |
| *Waveshare 2" LCD*     | Power (VCC)         | `3.3V`                              | 3.3V Power             | Use the main GPIO breakout pins.                                        |
|                        | Ground (GND)        | `GND`                               | Ground                 |                                                                         |
|                        | Data In (DIN)       | `SPI_MOSI`                          | GPIO 10                |                                                                         |
|                        | Clock (CLK)         | `SPI_SCLK`                          | GPIO 11                |                                                                         |
|                        | Chip Select (CS)    | `SPI_CE0`                           | GPIO 8                 |                                                                         |
|                        | Data/Command (DC)   | `D25`                               | GPIO 25                |                                                                         |
|                        | Reset (RST)         | `D17`                               | GPIO 17                |                                                                         |
|                        | **Backlight (BL)**  | `D18`                               | **GPIO 18**            | **This pin conflicts with the I2S Mic.**                                |
|                        |                     |                                     |                        |                                                                         |
| **Unused / Free Pins** | -                   | `D21`, `D22`                        | GPIO 21, 22            | Previously used by the HC-SR04 sensor, now available.                   |

***

## Upgraded Software Setup for NinjaRobot

This section details the full software setup process on a fresh Raspberry Pi, tailored for the upgraded NinjaRobot with the VL6180X I2C sensor, the Waveshare SPI LCD, and the FastAPI web framework.

**Important Note for Pi Zero Users:** Some of these steps, especially installing the Rust compiler and Python packages, are very resource-intensive and will take a **significant amount of time (potentially several hours)**. Please ensure you are using a stable, high-quality power supply (5V, >= 2.5A) to prevent crashes during installation. Be patient.

### Step 1: Raspberry Pi OS Installation (Headless Setup)

We will start by preparing the SD card with a "headless" configuration, which allows you to set up and control the Raspberry Pi from your main computer without needing a separate monitor, keyboard, or mouse.

1.  **Download Raspberry Pi Imager:** On your main computer, download and install the official [Raspberry Pi Imager](https://www.raspberrypi.com/software/).

2.  **Choose OS:** Launch the Imager.
    *   Click `CHOOSE DEVICE` and select your Raspberry Pi model (e.g., `Raspberry Pi Zero 2 W`).
    *   Click `CHOOSE OS`, select `Raspberry Pi OS (other)`, then choose **`Raspberry Pi OS (Legacy, 32-bit)`**. The "Bullseye" version is highly recommended for maximum compatibility with the hardware libraries.

3.  **Pre-configure for Headless Access:** Before writing, click the **gear icon (⚙️)** to open the "Advanced options".
    *   Check **`Set hostname`** and enter a name (e.g., `ninjarobot.local`).
    *   Check **`Enable SSH`** and select "Use password authentication".
    *   Check **`Set username and password`** and create your login credentials.
    *   Check **`Configure wireless LAN`** and enter your Wi-Fi network's SSID and password. This is crucial for connecting to the robot.
    *   **SAVE** your settings.

4.  **Write to SD Card:** Insert your microSD card into your computer, click `CHOOSE STORAGE` in the Imager, select the card, and click **`WRITE`**.

5.  **First Boot and Connect:** Once writing is complete, insert the microSD card into the Raspberry Pi and connect the power. Wait a few minutes for the first boot. From your computer's terminal, connect via SSH:
    ```bash
    ssh your_username@ninjarobot.local
    ```    (Replace `your_username` with the one you created).

    *   **SSH Troubleshooting:** If you have connected to a previous Raspberry Pi at the same address, you might see a "REMOTE HOST IDENTIFICATION HAS CHANGED" warning. The easiest fix is to run the following command on your computer (not the Pi) and then try connecting again:
        ```bash
        ssh-keygen -R ninjarobot.local
        ```

#### **Explanation of Step 1**

*   **What We Did:** We installed the Raspberry Pi operating system onto the microSD card using the official Imager tool. We pre-configured essential settings like the robot's network name (hostname), enabled remote access (SSH), and set up Wi-Fi.
*   **Why We Did It:** This "headless" approach is standard for robotics projects. It allows us to work on the robot without it being tethered to a monitor, which is essential for a mobile platform. Choosing the 32-bit Legacy OS ("Bullseye") provides a stable and well-tested environment for the specific hardware drivers we will be using.

---

### Step 2: System Update & Hardware Interface Configuration

Now we will update the system software and enable the necessary hardware communication protocols (I2C and SPI).

1.  **Update System Packages:** Run the following commands to ensure all software is up-to-date.
    ```bash
    sudo apt update
    sudo apt full-upgrade -y
    sudo apt install -y curl git
    ```

2.  **Enable I2C and SPI:** We will use the Raspberry Pi configuration tool.
    ```bash
    sudo raspi-config
    ```
    *   Navigate to `3 Interface Options`.
    *   Select `I5 SPI` and choose `<Yes>` to enable it.
    *   You will be returned to the main menu. Go back into `3 Interface Options`.
    *   Select `I4 I2C` and choose `<Yes>` to enable it.
    *   Select `<Finish>` on the main menu. If prompted to reboot, choose `<Yes>`. If not, reboot manually:
        ```bash
        sudo reboot
        ```

3.  **Reconnect via SSH** after the Pi reboots.

#### **Explanation of Step 2**

*   **What We Did:** We updated the Pi's software list and upgraded all installed packages to their latest versions. We then used the `raspi-config` tool to enable the SPI and I2C hardware interfaces.
*   **Why We Did It:**
    *   System updates are crucial for security and bug fixes.
    *   **I2C (Inter-Integrated Circuit)** is the protocol the **VL6180X distance sensor** and the DFRobot HAT use to communicate with the Pi.
    *   **SPI (Serial Peripheral Interface)** is the protocol the **Waveshare LCD screen** uses.
    *   These interfaces are disabled by default, so we must enable them to allow the Pi to talk to the new hardware.

---

### Step 3: Core System Dependencies and Build Tools

Next, we install system-level libraries that our Python packages will depend on.

1.  **Install Essential Libraries:** This single command installs development tools and libraries for I2C, audio, and general-purpose building.
    ```bash
    sudo apt install -y python3-dev python3-pip python3-venv build-essential libasound2-dev portaudio19-dev libportaudiocpp0 ffmpeg flac libatlas-base-dev python3-smbus i2c-tools
    ```

2.  **Install Rust Compiler:** Some modern Python packages (like a dependency for `google-generativeai`) require the Rust compiler. **This step will take a very long time (30-60+ minutes).**
    ```bash
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
    ```
    *   When prompted, choose `1) Proceed with installation (default)`.
    *   After it finishes, run the command it provides to configure your shell:
        ```bash
        source "$HOME/.cargo/env"
        ```
    *   Verify with `rustc --version`.

3.  **Increase Swap Space (Crucial for Pi Zero):** To prevent the Pi from running out of memory during the large Python package installation, we will temporarily increase the swap file size.
    ```bash
    # Set swap to 1GB (1024MB)
    sudo dphys-swapfile swapoff
    echo "CONF_SWAPSIZE=2048" | sudo tee /etc/dphys-swapfile
    sudo dphys-swapfile setup
    sudo dphys-swapfile swapon
    ```

#### **Explanation of Step 3**

*   **What We Did:** We installed a set of pre-compiled system libraries, the Rust programming language compiler, and temporarily increased the system's virtual memory (swap).
*   **Why We Did It:**
    *   The `apt` libraries like `python3-dev` and `build-essential` are required to compile Python packages from source code, which `pip` often needs to do on a Raspberry Pi.
    *   `python3-smbus` and `i2c-tools` provide the low-level support for I2C communication.
    *   `rustc` is a non-negotiable dependency for the Gemini AI library.
    *   The Raspberry Pi Zero has very limited RAM (512MB). Increasing the swap space acts as a temporary, slower memory reserve on the SD card, preventing the system from crashing during memory-intensive compilation tasks.

---

### Step 4: Project Directory and Python Environment

We will now create a dedicated folder for the robot's code and a virtual environment to keep its Python packages isolated.

1.  **Create Project Directory and Virtual Environment:**
    ```bash
    mkdir ~/NinjaRobot
    cd ~/NinjaRobot
    python3 -m venv .venv
    ```

2.  **Activate the Virtual Environment:** You must run this command every time you open a new terminal to work on the project.
    ```bash
    source .venv/bin/activate
    ```
    Your terminal prompt should now be prefixed with `(.venv)`.

3.  **Install All Python Libraries:** This single command will install everything needed for the upgraded robot: Gemini, FastAPI, the new hardware, and legacy components. **This step will also take a very long time (1-2+ hours).**
    ```bash
    pip install --upgrade pip
    pip install smbus smbus2 RPi.GPIO Pillow numpy spidev google-generativeai SpeechRecognition gTTS pygame google-cloud-speech fastapi "uvicorn[standard]" # For I2C communication, important for DFRobot HAT. Try smbus if smbus2 does not work
    ```

4.  **Revert Swap Space:** After the intensive installation is done, we'll return the swap space to its default size to reduce wear on the SD card.
    ```bash
    # Set swap back to 100MB (default)
    sudo dphys-swapfile swapoff
    echo "CONF_SWAPSIZE=100" | sudo tee /etc/dphys-swapfile
    sudo dphys-swapfile setup
    sudo dphys-swapfile swapon
    sudo reboot
    ```
5.  **Reconnect via SSH** after the final reboot. Remember to activate the virtual environment again before running any code:
    ```bash
    cd ~/NinjaRobot
    source .venv/bin/activate
    ```

#### **Explanation of Step 4**

*   **What We Did:** We created a project folder (`NinjaRobot`) and a Python virtual environment (`.venv`) inside it. We then activated the environment and used `pip` to install all necessary Python packages, including the newly requested `fastapi` and its server `uvicorn`. Finally, we reverted the swap space and rebooted.
*   **Why We Did It:**
    *   A **virtual environment** is a critical best practice. It creates an isolated space for the project's Python packages, preventing conflicts with system-wide packages or other projects on the same Pi.
    *   `smbus2` is the library for I2C communication (VL6180X).
    *   `Pillow`, `numpy`, and `spidev` are for the SPI LCD screen.
    *   `fastapi` replaces Flask as the web framework, and `uvicorn` is the high-performance server needed to run it.
    *   Reverting the swap space protects the microSD card from excessive writing, extending its lifespan.

Your NinjaRobot's software environment is now fully configured and ready for the next stage: integrating the new code modules.

Excellent. This is a logical and crucial next step. Creating a robust, general-purpose calibration tool is fundamental to building reliable and complex movements later. Referencing `ExpressiveRobot.md` for the calibration *process* is a great idea.


### Servo Initialization and Calibration

1.  **Hardware Interaction:** The DFRobot IO Expansion HAT is the intermediary for all servo control. Therefore, we must use the `DFRobot_RaspberryPi_Expansion_Board` Python library, specifically its `Servo` class. This class takes a pin number (0-15) and an "angle" value (0-180) to control the PWM signal. Our script will manipulate this "angle" value, which is essentially the raw data sent to the servo controller.

2.  **User Interface:** To create the real-time, key-press-driven interface described in your request, we will use Python's built-in `curses` library. This is the standard tool for creating sophisticated text-based user interfaces (TUIs) in a terminal, allowing us to capture single key presses without the user needing to hit "Enter."

3.  **Data Persistence:** The calibration data will be stored in a `servo_calibration.json` file. This file will act as a "translation layer" for all future movement scripts. For example, when a future script wants to move the "right thigh" to `-90` degrees, it will look up the calibrated raw value (e.g., `25`) in this JSON file and send that to the DFRobot HAT.

4.  **General-Purpose Design:** The script is designed to be highly flexible. By asking the user to define their servos at the start, it's not hard-coded to a specific robot build. This makes it reusable for future projects with different servo configurations.

---

### Step-by-Step Instructions

1.  **Navigate to Your Project Directory:** Connect to your Raspberry Pi via SSH and navigate to the project folder. Make sure your virtual environment is active.
    ```bash
    cd ~/NinjaRobot
    source .venv/bin/activate
    ```

2.  **Get the DFRobot Library:** The `DFRobot_RaspberryPi_Expansion_Board` library is not on `pip`. We need to download it directly into our project folder from its source.
    ```bash
    git clone https://github.com/DFRobot/DFRobot_RaspberryPi_Expansion_Board.git
    # Now, copy the necessary library file into your main project directory
    cp -r DFRobot_RaspberryPi_Expansion_Board/python/DFRobot_RaspberryPi_Expansion_Board .
    ```    *This ensures that `import DFRobot_RaspberryPi_Expansion_Board` will work in your scripts.*

3.  **Create the Script File:** Create a new Python file named `ninja_servo_init.py`.
    ```bash
    nano ninja_servo_init.py
    ```

4.  **Copy and Paste the Code:** Copy the entire Python code block below and paste it into the `nano` editor.

5.  **Save and Exit:** Press `Ctrl+X`, then `Y`, then `Enter` to save the file.

6.  **Run the Script:** Execute the script from your terminal.
    ```bash
    python3 ninja_servo_init.py
    ```

7.  **Follow the On-Screen Instructions:**
    *   First, you'll be prompted to define your servos one by one. Use the format `name,pin,type`. For example:
        *   `rt0,0,180` (A 180° servo named `rt0` on pin 0 for the right thigh)
        *   `lf3,3,180` (A 180° servo named `lf3` on pin 3 for the left foot)
        *   `rw2,2,360` (A 360° servo named `rw2` on pin 2 for the right wheel)
        *   Type `done` when you have added all your servos.
    *   Next, the calibration screen will appear for your first servo. Use the keys as instructed on-screen (`w`/`s`, `Shift+X`/`C`/`V`) to find and set the calibration points.
    *   Press `n` to move to the next servo in your list.
    *   Press `q` at any time to save all progress and quit the application. The `servo_calibration.json` file will be created or updated in your project directory.

---

### Complete Code for `ninja_servo_init.py`

```python
# -*- coding: utf-8 -*-
"""
ninja_servo_init.py

A general-purpose servo calibration tool for the NinjaRobot project.
This script allows users to:
1. Define a custom set of servos with names, pins, and types (180/360).
2. Interactively calibrate the minimum, center, and maximum positions for each servo.
3. Save the calibration data to a 'servo_calibration.json' file for use by other scripts.

This script uses the DFRobot_RaspberryPi_Expansion_Board library and the curses library
for a real-time text-based user interface (TUI).

Author: Your Name/Assistant
Date: 2024-05-25
Version: 1.0
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

def init_hardware():
    """Initializes the DFRobot Expansion Board and Servo controller."""
    global board, servo
    # Initialize the board with I2C bus 1 and address 0x10
    board = Board(1, 0x10)
    while board.begin() != board.STA_OK:
        print("Error: DFRobot Board connection failed. Check wiring and I2C.")
        print("Retrying in 3 seconds...")
        time.sleep(3)
    print("DFRobot Board connection successful.")

    # Initialize the servo controller from the board
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
    """
    Interactively prompts the user to define the servos they want to calibrate.
    This runs before the curses TUI starts.
    """
    global servos_to_calibrate
    print("\n--- Ninja Robot Servo Setup ---")
    print("Please define the servos you want to calibrate.")
    print("Use the format: name,pin,type")
    print("  - 'name': A unique name for the servo (e.g., 'right_thigh', 'rt0').")
    print("  - 'pin': The PWM pin number on the HAT (0-15).")
    print("  - 'type': The servo type, either '180' or '360'.")
    print("Example: rt0,0,180")
    print("Type 'done' when you have finished adding servos.\n")

    while True:
        user_input = input(f"Define servo #{len(servos_to_calibrate) + 1} (or type 'done'): ").strip().lower()
        if user_input == 'done':
            if not servos_to_calibrate:
                print("No servos defined. Exiting.")
                return False
            print("\nServo setup complete. Proceeding to calibration...")
            time.sleep(1)
            return True
        
        try:
            name, pin_str, type_str = user_input.split(',')
            pin = int(pin_str)
            servo_type = int(type_str)

            if not (0 <= pin <= 15):
                print("Error: Pin must be between 0 and 15.")
                continue
            if servo_type not in [180, 360]:
                print("Error: Type must be either 180 or 360.")
                continue
            
            # Check for duplicate names or pins
            if any(s['name'] == name for s in servos_to_calibrate):
                print(f"Error: Servo name '{name}' is already in use.")
                continue
            if any(s['pin'] == pin for s in servos_to_calibrate):
                print(f"Error: Pin {pin} is already in use.")
                continue

            servos_to_calibrate.append({'name': name, 'pin': pin, 'type': servo_type})
            print(f"  -> Added '{name}' (Pin: {pin}, Type: {servo_type}°)")

        except ValueError:
            print("Invalid format. Please use: name,pin,type (e.g., lf3,3,180)")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

def draw_ui(stdscr, servo_info, cal_data, current_value):
    """Renders the Text-based User Interface using curses."""
    stdscr.clear()
    h, w = stdscr.getmaxyx() # Get screen height and width

    name = servo_info['name']
    pin = servo_info['pin']
    stype = servo_info['type']
    
    # --- Title and Servo Info ---
    title = f"--- Calibrating Servo: '{name}' (Pin: {pin}, Type: {stype}°) ---"
    stdscr.addstr(0, (w - len(title)) // 2, title, curses.A_BOLD)

    # --- Current Value Display ---
    val_str = f"Current Raw Value: {current_value}"
    stdscr.addstr(3, 2, val_str, curses.A_REVERSE)

    # --- Saved Calibration Points Display ---
    stdscr.addstr(5, 2, "Saved Points:", curses.A_UNDERLINE)
    s_data = cal_data.get(name, {})
    if stype == 180:
        min_label, center_label, max_label = "-90° (Min)", "0° (Center)", "+90° (Max)"
        min_key, center_key, max_key = "min", "center", "max"
    else: # 360
        min_label, center_label, max_label = "CCW Max (-90)", "Stop (0)", "CW Max (+90)"
        min_key, center_key, max_key = "ccw_max", "stop", "cw_max"

    stdscr.addstr(6, 4, f"{min_label}:    {s_data.get(min_key, 'Not Set')}")
    stdscr.addstr(7, 4, f"{center_label}: {s_data.get(center_key, 'Not Set')}")
    stdscr.addstr(8, 4, f"{max_label}:    {s_data.get(max_key, 'Not Set')}")

    # --- Instructions ---
    stdscr.addstr(h - 6, 2, "Controls:", curses.A_UNDERLINE)
    stdscr.addstr(h - 5, 4, "'w' / 's' : Adjust value up / down by 1.")
    stdscr.addstr(h - 4, 4, f"'Shift+X' : Set current value as {min_label}.")
    stdscr.addstr(h - 3, 4, f"'Shift+C' : Set current value as {center_label}.")
    stdscr.addstr(h - 2, 4, f"'Shift+V' : Set current value as {max_label}.")
    stdscr.addstr(h - 1, 4, "'n': Next Servo | 'q': Save & Quit", curses.A_BOLD)
    
    stdscr.refresh()

def calibration_loop(stdscr, servo_info, calibration_data):
    """Main interactive loop for calibrating a single servo."""
    name = servo_info['name']
    pin = servo_info['pin']
    stype = servo_info['type']
    
    # Initialize calibration data for this servo if it doesn't exist
    if name not in calibration_data:
        calibration_data[name] = {'pin': pin, 'type': stype}
    
    # Start at the center value if it exists, otherwise default to 90
    current_value = calibration_data[name].get('center', 90)
    if stype == 360:
        current_value = calibration_data[name].get('stop', 90)
    
    # Move servo to initial position
    servo.move(pin, current_value)

    while True:
        draw_ui(stdscr, servo_info, calibration_data, current_value)
        key = stdscr.getkey()

        if key == 'w':
            current_value = min(180, current_value + 1)
        elif key == 's':
            current_value = max(0, current_value - 1)
        elif key == 'X': # Shift+X
            key_name = "min" if stype == 180 else "ccw_max"
            calibration_data[name][key_name] = current_value
        elif key == 'C': # Shift+C
            key_name = "center" if stype == 180 else "stop"
            calibration_data[name][key_name] = current_value
        elif key == 'V': # Shift+V
            key_name = "max" if stype == 180 else "cw_max"
            calibration_data[name][key_name] = current_value
        elif key == 'n':
            # Stop this servo before moving to the next
            center_val = calibration_data[name].get('center' if stype == 180 else 'stop', 90)
            servo.move(pin, center_val)
            return 'next'
        elif key == 'q':
            # Stop this servo before quitting
            center_val = calibration_data[name].get('center' if stype == 180 else 'stop', 90)
            servo.move(pin, center_val)
            return 'quit'

        # Update servo position after any change
        servo.move(pin, current_value)

def main(stdscr):
    """The main function wrapped by curses."""
    # Curses setup
    curses.curs_set(0)  # Hide the cursor
    stdscr.nodelay(False) # Wait for user input
    stdscr.timeout(-1)

    calibration_data = load_calibration_data()

    for servo_info in servos_to_calibrate:
        result = calibration_loop(stdscr, servo_info, calibration_data)
        if result == 'quit':
            break
            
    # Save data at the end
    save_calibration_data(calibration_data)

if __name__ == "__main__":
    try:
        # Get user input for which servos to set up FIRST
        if get_servo_setup_from_user():
            # If servos were defined, initialize hardware and start curses UI
            init_hardware()
            curses.wrapper(main)
    except KeyboardInterrupt:
        print("\nProgram interrupted by user. Exiting.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()

```
