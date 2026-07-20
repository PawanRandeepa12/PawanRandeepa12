# Arduino PID Line Follower Robot

An Arduino-based line-following robot utilizing a **Proportional-Derivative (PD) control algorithm** with 4 analog IR sensors and dual DC motors for smooth and precise line tracking.

---

## 🚀 Features

- **PID Control System**: Implements proportional and derivative tuning for responsive steering adjustments along curves and straight paths.
- **Multi-Sensor Array**: Uses 4 analog IR sensors with weighted position detection.
- **Safety Mechanism**: Automatically stops the robot if all sensors lose the line (off-track condition).
- **Adjustable Speeds**: Configurable base motor speed and maximum speed limits.

---

## 📌 Hardware Pinout & Connections

| Component | Pin / Port | Arduino Pin | Description |
| :--- | :--- | :--- | :--- |
| **IR Sensor 1** | `IR1` | `A0` | Far-left analog IR sensor |
| **IR Sensor 2** | `IR2` | `A1` | Inner-left analog IR sensor |
| **IR Sensor 3** | `IR3` | `A2` | Inner-right analog IR sensor |
| **IR Sensor 4** | `IR4` | `A3` | Far-right analog IR sensor |
| **Left Motor Direction A** | `LMotorA` | Digital `4` | Left motor control pin A |
| **Left Motor Direction B** | `LMotorB` | Digital `5` | Left motor control pin B |
| **Left Motor PWM** | `LMotorPWM` | Digital `10` | Left motor speed control (PWM) |
| **Right Motor Direction A** | `RMotorA` | Digital `2` | Right motor control pin A |
| **Right Motor Direction B** | `RMotorB` | Digital `3` | Right motor control pin B |
| **Right Motor PWM** | `RMotorPWM` | Digital `9` | Right motor speed control (PWM) |

---

## ⚙️ Control Parameters & Tuning

You can tune the robot's performance in the code by adjusting the following variables:

```cpp
float Kp = 5.5;  // Proportional gain
float Kd = 10;   // Derivative gain
float Ki = 0;    // Integral gain

int MotorBasespeed = 100; // Base speed for both motors (0-255)
int MAX_SPEED = 150;      // Maximum speed limit
```

- **Sensor Weights**: `{-20, -10, -5, 5}` determines the influence of each IR sensor on the error calculation.

---

## 📂 Code Structure (`README.ino`)

- **`setup()`**: Initializes serial communication (`9600` baud), configures pin modes, and initiates a 2-second forward movement test.
- **`loop()`**: Continuously reads sensor values, checks for line loss (stops if all sensors read `0`), computes PID steering corrections, and updates motor speeds.
- **`PID_control()`**: Calculates error based on sensor weights, computes P, I, and D terms, and generates the `speedAdjust` value.
- **`read_IR()`**: Reads analog values from pins `A0` through `A3`.
- **`set_speed()` / `set_forward()` / `stop()`**: Motor driver helper functions for movement and speed application.

---

## 🛠️ How to Use

1. Open `README.ino` in the **Arduino IDE**.
2. Select your Arduino board (e.g., Arduino Uno) and the correct COM port.
3. Verify and upload the sketch to your board.
