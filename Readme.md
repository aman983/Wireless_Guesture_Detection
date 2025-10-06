# ESP-IMU-Gesture-Server

A FreeRTOS-based project for the ESP8266 (or similar platform) that reads data from an MPU-6050 (Accelerometer/Gyroscope) and an HMC5883L (Magnetometer), applies sensor fusion for attitude estimation, detects gestures, and serves the detected gesture status over a simple HTTP endpoint.

## Features

* **I2C Communication:** Custom low-level I2C read/write routines.
* **Sensor Interfacing:** Drivers for **MPU-6050** (Accel/Gyro) and **HMC5883L** (Magnetometer).
* **FreeRTOS Integration:** Uses tasks (`Sensor_Task`, `Motion_Sense_Task`) and queues (`Sensor_Data_queue`, `Guesture_queue`) for concurrent operation.
* **Sensor Fusion:**
    * **Low-Pass Filter (LPF):** Applied to raw sensor data for smoothing.
    * **Complementary Filter:** Fuses MPU-6050 gyroscope and accelerometer data to estimate **pitch** and **roll** angles, balancing fast response (gyro) and long-term stability (accel).
* **Gesture Detection:** Identifies simple static and dynamic gestures based on pitch and roll angles.
* **Web Server:** Provides a simple HTTP endpoint to query the currently detected gesture.

## Hardware Requirements

* **ESP8266** Development Board (or compatible ESP-IDF device if the library calls are adapted).
* **MPU-6050** (3-axis Accelerometer and 3-axis Gyroscope).
* **HMC5883L** (3-axis Magnetometer - *Note: The magnetometer data is read but not used in the final gesture/filter logic in the provided code snippet*).
* I2C connections for the sensors (SDA, SCL).

## Software Structure & FreeRTOS Tasks

The application is structured into two main FreeRTOS tasks that communicate via queues:

| Task Name | Priority | Function | Queue Used |
| :--- | :--- | :--- | :--- |
| `Sensor_Task` | 5 | Initializes I2C and sensors. Continuously reads raw data from MPU-6050 and HMC5883L. **Producer** for raw sensor data. | `Sensor_Data_queue` (Sends) |
| `Motion_Sense_Task` | 5 | Reads raw data, applies LPF, runs the Complementary Filter to estimate pitch/roll, and detects gestures. **Consumer** of raw data and **Producer** of gestures. | `Sensor_Data_queue` (Receives), `Guesture_queue` (Sends) |

## Sensor Fusion and Gesture Logic

### Complementary Filter

The filter estimates the true attitude (pitch and roll) by combining the two sensor sources:
$$\text{Angle}_{new} = \alpha \cdot (\text{Angle}_{prev} + \text{GyroRate} \cdot \Delta t) + (1 - \alpha) \cdot \text{AccelAngle}$$
Where $\alpha$ (defined as `COMPLEMENTARY_ALPHA`) is **0.98**, heavily trusting the gyroscope for short-term changes and using the accelerometer only for drift correction.

### Gestures Defined

| Gesture | Type | Condition | Threshold |
| :--- | :--- | :--- | :--- |
| **`GESTURE_SHAKE`** | Dynamic | Quick change in roll **|&Delta; Roll|** | `SHAKE_THRESHOLD` (**20.0°**) |
| **`GESTURE_NOD`** | Dynamic | Quick change in pitch **|&Delta; Pitch|** | `NOD_THRESHOLD` (**20.0°**) |
| **`GESTURE_TILT_RIGHT`** | Static | Sustained roll angle (positive) | `TILT_THRESHOLD` (**30.0°**) |
| **`GESTURE_TILT_LEFT`** | Static | Sustained roll angle (negative) | `TILT_THRESHOLD` (**30.0°**) |
| **`GESTURE_TILT_FORWARD`** | Static | Sustained pitch angle (positive) | `TILT_THRESHOLD` (**30.0°**) |
| **`GESTURE_TILT_BACKWARD`** | Static | Sustained pitch angle (negative) | `TILT_THRESHOLD` (**30.0°**) |

## Web Server Endpoint

The detected gesture is available via a single HTTP GET endpoint.

* **Path:** `/gesture`
* **Method:** `GET`
* **Response:** A string indicating the detected gesture (e.g., `LEFT`, `RIGHT`, `FORWARD`, `BACKWARD`). Only static tilt gestures are currently handled by the HTTP handler.

### Example Request/Response

```bash
# Assuming the ESP device is on 192.168.1.10
curl [http://192.168.1.10/gesture](http://192.168.1.10/gesture)
