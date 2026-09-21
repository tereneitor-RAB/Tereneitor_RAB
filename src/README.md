# The codes and their explanations are attached to this folder.

# explanation of robot programming

- The robot's programming is designed to allow it to navigate the track autonomously using different sensors and control systems.

- The **D500 LiDAR** is the main sensor used to detect walls and measure the distances around the robot. The program divides the surrounding area into different zones, such as front, left, right, and diagonal sections, so the robot can determine where there is available space and make decisions when it encounters a wall.

- The **Raspberry Pi camera** is mainly used to detect red and green obstacles. The image is processed using OpenCV, and only a horizontal section of the image is analyzed to reduce unnecessary processing.

- The **steering servo** controls the direction of the robot. To prevent sudden movements, the program uses different movement speeds depending on how far the servo is from its target position. This makes the steering changes smoother.

- While moving, the robot also tries to maintain a distance of approximately **550 mm from the selected wall**. The program makes small steering corrections when the robot gets too close or too far from that wall.

- When the robot detects a wall in front of it, the program analyzes different areas on both the left and right sides before deciding which direction to take. Instead of looking at only one point, it considers the side, diagonal, and near-front areas.

- Every time a new turn begins, the program increases the turn counter. The limit is set to **12 turns**. Once the robot reaches the 12th turn, it stops the motor, centers the servo, and shuts down the steering system.
The program also includes an **emergency reverse system**. If the LiDAR detects that a wall is too close, the robot stops, reverses, and checks the available space again before continuing.

- The robot starts after the start button is pressed. While waiting, the green LED stays on to indicate that the system is ready. Once the button is pressed, the motor starts moving and the program enters its main navigation loop.

- During the run, the program displays the camera feed and a **360° LiDAR radar**, along with the front, left, and right distances, detected color, number of turns completed, and the current servo position.

- Overall, the program combines the **LiDAR for navigation**, the **camera for color-based obstacle detection**, and the **servo and motor for movement**. This allows the robot to make decisions while navigating the track without manual control.

---------------------------------------------------------------------------

# Operating System Installation – Raspberry Pi 4B

Before starting the project, the Raspberry Pi 4B needs an operating system. For this project, **Raspberry Pi OS** is installed on a microSD card using **Raspberry Pi Imager**.

## 1. Download Raspberry Pi Imager

Download and install **Raspberry Pi Imager** on a computer from the official Raspberry Pi website:

[**Download Raspberry Pi Imager**](https://www.raspberrypi.com/software/)

Then connect the microSD card to the computer using a card reader.

## 2. Select the Raspberry Pi

Open Raspberry Pi Imager and select:

* **Device:** Raspberry Pi 4
* **Operating System:** Raspberry Pi OS
* **Storage:** The connected microSD card

> [!IMPORTANT]
> The microSD card will be erased during the installation. Make sure the correct card is selected.

## 3. Configure the System

Before writing the operating system, Raspberry Pi Imager allows you to configure some basic settings, such as:

* Username and password
* Wi-Fi connection
* Time zone
* Keyboard layout
* Hostname

These settings make the Raspberry Pi ready to use after the first boot.

## 4. Install the Operating System

After checking the configuration:

1. Click **Next** or **Write**.
2. Confirm that the microSD card can be erased.
3. Wait while Raspberry Pi Imager writes the operating system.
4. Wait for the verification process to finish.

This may take several minutes.

## 5. Start the Raspberry Pi

When the installation is complete:

1. Safely eject the microSD card.
2. Insert it into the Raspberry Pi 4B.
3. Connect the monitor, keyboard, and mouse if needed.
4. Connect the power supply.
5. Wait for Raspberry Pi OS to start.

Complete any remaining initial configuration.

## 6. Update the System

Open the terminal and run:

```bash
sudo apt update
sudo apt full-upgrade -y
```

Then restart the Raspberry Pi:

```bash
sudo reboot
```

## 7. Verify the Installation

To check the operating system, run:

```bash
cat /etc/os-release
```

If the Raspberry Pi OS information is displayed correctly, the installation is complete.

----------------------------------------------------------------------------

# Required Libraries Installation

After installing Raspberry Pi OS, the next step is to install the Python libraries required by the robot.

## 1. Update the System

First, update the package list:

```bash
sudo apt update
sudo apt full-upgrade -y
```

## 2. Install Python and Basic Tools

Install Python and the required development tools:

```bash
sudo apt install -y python3 python3-pip python3-dev
```

## 3. Install OpenCV

OpenCV is used for image processing and detecting colors and objects with the Raspberry Pi Camera.

```bash
sudo apt install -y python3-opencv
```

Verify the installation:

```bash
python3 -c "import cv2; print(cv2.__version__)"
```

## 4. Install Picamera2

Picamera2 is used to control the Raspberry Pi Camera.

```bash
sudo apt install -y python3-picamera2
```

Verify the installation:

```bash
python3 -c "from picamera2 import Picamera2; print('Picamera2 OK')"
```

## 5. Install GPIO Libraries

The GPIO pins are used to control the button, LEDs, motor driver, and servo.

Install RPi.GPIO:

```bash
sudo apt install -y python3-rpi.gpio
```

If the project uses GPIO Zero, it can also be installed with:

```bash
sudo apt install -y python3-gpiozero
```

## 6. Install PySerial

PySerial is used for serial communication between the Raspberry Pi and the LiDAR D500.

```bash
sudo apt install -y python3-serial
```

Verify the installation:

```bash
python3 -c "import serial; print('PySerial OK')"
```

## 7. Install NumPy

NumPy is used by OpenCV and other parts of the vision system for numerical and image data processing.

```bash
sudo apt install -y python3-numpy
```

Verify the installation:

```bash
python3 -c "import numpy; print(numpy.__version__)"
```

## 8. Install All Main Libraries

The main libraries used by the robot can also be installed together:

```bash
sudo apt install -y python3-opencv python3-picamera2 python3-rpi.gpio python3-gpiozero python3-serial python3-numpy
```

## 9. Test the Main Libraries

Run the following command to check that the main Python libraries can be imported:

```bash
python3 -c "import cv2, serial, numpy; from picamera2 import Picamera2; import RPi.GPIO; print('All main libraries installed successfully')"
```

If the terminal displays:

```text
All main libraries installed successfully
```

the main software dependencies are ready.

