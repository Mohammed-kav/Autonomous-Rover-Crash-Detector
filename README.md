# ResQ TrafficLink: Autonomous Crash-Alert Rover

## Overview
[cite_start]ResQ TrafficLink is a prototype crash-alert system built on an autonomous 4WD rover[cite: 35, 38]. [cite_start]It is designed to address the delay in accident reporting by automatically detecting crash events using motion sensing and transmitting an alert with GPS coordinates to a nearby Android smartphone via Bluetooth Low Energy (BLE)[cite: 35, 38, 42].


## Hardware Components
* [cite_start]**Arduino Mega 2560:** The main vehicle controller[cite: 100].
* [cite_start]**ESP32 DevKit V1:** Acts as the wireless communication gateway to the phone[cite: 193, 499].
* [cite_start]**ADXL335 3-Axis Accelerometer:** Used for crash detection based on impact and tilt/rollover patterns[cite: 39, 491].
* [cite_start]**NEO-6M GPS Module:** Provides position, speed, and time data[cite: 40, 493].
* [cite_start]**5x IR Sensors:** Used for autonomous navigation and obstacle avoidance[cite: 496].
* [cite_start]**L298N Motor Driver & 4WD Chassis:** Controls the DC motors for rover movement[cite: 123, 485].

## System Architecture & Workflow

### 1. Vehicle Control & Sensing
[cite_start]The Arduino Mega handles the rover's autonomous motion, reading five IR sensors to avoid obstacles and navigate[cite: 100, 496]. [cite_start]In parallel, it continuously samples the ADXL335 accelerometer and monitors the NEO-6M GPS module[cite: 100, 326, 350]. [cite_start]The rover will not begin autonomous movement until a valid and recent GPS fix is acquired[cite: 250].

### 2. Inter-Board Communication (UART)
[cite_start]When the Arduino's logic detects a crash event, it packages the event flag and the latest GPS telemetry into a single structured UART message[cite: 41, 352]. [cite_start]This message is sent to the ESP32 in the following format: `CRASH, 1, speed_cms, lat_e7, lon_e7, time_s`[cite: 352]. 

### 3. Wireless Alert Delivery (BLE GATT)
[cite_start]The ESP32 acts as a bridge, parsing the UART message and updating a custom BLE GATT service[cite: 86, 194]. [cite_start]It exposes the crash flag and telemetry fields as fixed-width characteristics to the connected phone[cite: 117].

### 4. Android Smartphone Receiver
[cite_start]On the receiving end, a background automation workflow built using the "Automate" app (by LlamaLab) polls the ESP32's crash characteristic[cite: 43, 44]. Once a crash flag is detected, the workflow:
1.  [cite_start]Triggers an immediate audible crash alert notification[cite: 259].
2.  [cite_start]Reads the remaining characteristics (speed, location, time)[cite: 44].
3.  [cite_start]Generates a second notification containing the crash details[cite: 260].
4.  [cite_start]Opens a Google Maps pin at the reported crash coordinates[cite: 45, 261].
