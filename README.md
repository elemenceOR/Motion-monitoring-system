# IoT Motion & Tempreture Monitoring System

A **WiFi-connected IoT sensor system** built using the  **Raspberry Pi Pico W** ,  **MPU6050 accelerometer/gyroscope** , and an  **SH1106 OLED display**. This project continuously collects motion and temperature data, publishes it via  **MQTT** , and visualizes it on a **Node-RED dashboard** running in a Docker container.

## Overview

This project demonstrates an end-to-end IoT workflow — from **sensor acquisition** and **data transmission** to  **visualization and analysis** :

* **Raspberry Pi Pico W** collects motion and temperature data using an **MPU6050 IMU**
* **SH1106 OLED** displays system status and sensor readings
* Data is sent securely over **MQTT** to a **Node-RED dashboard**
* All hardware connections are documented in  **KiCad schematics** , and a **3D model** of the assembly was created in **Autodesk Fusion 360**

## Features

* Seonsor initialization (OLED + IMU)
* Automatic sensor calibration for IMU
* WiFi auto-reconnect and MQTT retry mechanisms
* Real-time data streaming to Node-RED dashboard
* OLED display feedback for system status
* Modular, class-based MicroPython code architecture
* Dockerized Node-RED setup for easy deployment

## Hardware

| Component                     | Description                                                            | Notes          |
| :---------------------------- | ---------------------------------------------------------------------- | -------------- |
| **Raspberry Pi Pico W** | Microcontroller with built-in WiFi<br />Small form factor              |                |
| **MPU6050**             | 6-axis IMU (accelerometer + gyroscope)<br />In-built tempreture sensor | I²C interface |
| **SH1106 OLED Display** | 128x64 display                                                         | I²C display   |

## Connections

All hardware wiring is documented in the **KiCad schematic** files (`/hardware/kicad/`).

| Device        | SDA Pin | SCL Pin | I²C Bus |
| ------------- | ------- | ------- | -------- |
| OLED (SH1106) | GP4     | GP5     | I²C0    |
| IMU (MPU6050) | GP14    | GP15    | I²C1    |

![1761058411164](image/Untitled-1/1761058411164.png)

## 3D Model

A **3D enclosure and mounting design** was created in **Autodesk Fusion 360** to neatly package the components.

The 3D model files (`/hardware/fusion360/`) include:

* Board layout and standoffs
* OLED screen cutout
* USB access port

![1761070833520](image/Untitled-1/1761070833520.png)

## Software Architecture

### 1. MicroPython Firmware

Responsible for:

* Device initialization and calibration
* Data collection from the MPU6050
* WiFi + MQTT connectivity
* OLED display updates
* Fault recovery and watchdog-like behavior

### 2. Node-RED Dashboard

* **Runs in a Docker container**
* Subscribes to the MQTT topics published by the Pico
* Displays:
  * Real-time graphs of accelerometer and gyroscope data
  * Temperature readings
  * MQTT topics
    `pico/temperature pico/ax pico/ay pico/az pico/gx pico/gy pico/gz`

## Setting up Node-RED with Docker

Use this [link](https://nodered.org/docs/getting-started/docker) to setup nodered with docker.

* Import the flow file (`nodered/flow.json`)
* Configure MQTT input nodes to match your broker and topic names
* Deploy the flow and view live sensor data

## MQTT Configuration

By default, the system uses the **public HiveMQ broker** for demonstration:

`self.client = MQTTClient('bigles', 'broker.hivemq.com', keepalive=60)`

To use a  **custom/private broker** , update the `config.py` file:

`wifi_ssid = "YOUR_WIFI_SSID" 
wifi_password = "YOUR_WIFI_PASSWORD" 
mqtt_server = "broker.hivemq.com" 
mqtt_username = "" mqtt_password = ""`

## System Workflow

1. On power-up:
   * OLED displays “Initializing...”
   * IMU is calibrated
2. The Pico connects to WiFi and MQTT
3. Data from the IMU is read, calibrated, and published to MQTT every second
4. Node-RED receives and visualizes data in real-time
5. OLED shows the transmission status

## Calibration Process

* The system performs a 10-second calibration on startup.
* Keep the sensor **flat and still** during this period.
* Offsets are automatically calculated and used to correct readings.
