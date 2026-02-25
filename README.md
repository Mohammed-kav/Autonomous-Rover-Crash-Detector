# ResQ TrafficLink: Autonomous Crash-Alert Rover

## Overview
ResQ TrafficLink is a prototype crash-alert system built on an autonomous 4WD rover.It is designed to address the delay in accident reporting by automatically detecting crash events using motion sensing and transmitting an alert with GPS coordinates to a nearby Android smartphone via Bluetooth Low Energy (BLE).


## Hardware Components
* **Arduino Mega 2560:** The main vehicle controller.
* **ESP32 DevKit V1:** Acts as the wireless communication gateway to the phone.
* **ADXL335 3-Axis Accelerometer:** Used for crash detection based on impact and tilt/rollover patterns.
* **NEO-6M GPS Module:** Provides position, speed, and time data.
* **5x IR Sensors:** Used for autonomous navigation and obstacle avoidance.
* **L298N Motor Driver & 4WD Chassis:** Controls the DC motors for rover movement.

## System Architecture & Workflow

### 1. Vehicle Control & Sensing
The Arduino Mega handles the rover's autonomous motion, reading five IR sensors to avoid obstacles and navigate.
In parallel, it continuously samples the ADXL335 accelerometer and monitors the NEO-6M GPS module.
The rover will not begin autonomous movement until a valid and recent GPS fix is acquired.

### 2. Inter-Board Communication (UART)
When the Arduino's logic detects a crash event, it packages the event flag and the latest GPS telemetry into a single structured UART message. This message is sent to the ESP32 in the following format: `CRASH, 1, speed_cms, lat_e7, lon_e7, time_s`. 

### 3. Wireless Alert Delivery (BLE GATT)
The ESP32 acts as a bridge, parsing the UART message and updating a custom BLE GATT service.
It exposes the crash flag and telemetry fields as fixed-width characteristics to the connected phone.

### 4. Android Smartphone Receiver
On the receiving end, a background automation workflow built using the "Automate" app (by LlamaLab) polls the ESP32's crash characteristic. Once a crash flag is detected, the workflow:
1.  Triggers an immediate audible crash alert notification.
2.  Reads the remaining characteristics (speed, location, time).
3.  Generates a second notification containing the crash details.
4.  Opens a Google Maps pin at the reported crash coordinates.
