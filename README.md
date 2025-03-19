## Right-Hand Rule Maze Robot

### Overview
This project implements an autonomous **Right-Hand Rule Maze Robot**, which navigates using the **right-hand rule**, a common technique for solving mazes and avoiding obstacles by always keeping the right side close to a wall. The robot is equipped with **motors, encoders, an IMU (MPU6050), and time-of-flight (VL53L0X) sensors** to facilitate precise movement and environmental awareness.

### Hardware Components
- **Microcontroller**: ESP32
- **Motor Drivers**: H-Bridge for controlling left and right motors
- **Encoders**: Used for tracking wheel rotations and distance traveled
- **IMU Sensor (MPU6050)**: Provides gyroscope data for heading correction
- **Time-of-Flight Sensors (VL53L0X)**: Used for detecting distances to walls and obstacles
- **Power Supply**: Battery-powered for autonomous movement

### I2C Communication Protocol
The **I2C (Inter-Integrated Circuit)** protocol is used for communication between the microcontroller and sensors like the **MPU6050 IMU** and **VL53L0X Lidar sensors**. I2C is a **two-wire communication** protocol that consists of:
- **SDA (Serial Data Line)**: Transfers data between devices.
- **SCL (Serial Clock Line)**: Synchronizes the communication.

Each device on the I2C bus has a unique address, allowing multiple devices to communicate over the same two lines. The **VL53L0X sensors** use custom I2C addresses to prevent conflicts, and the **MPU6050** provides accelerometer and gyroscope data via I2C.

### Right-Hand Rule Logic
The right-hand rule is applied to navigate an environment by following these steps:
1. **Check Right Wall**: If there is an open space on the right side, turn **right** and move forward.
2. **Check Front Wall**: If there is an obstacle ahead, turn **left**.
3. **Align with Right Wall**: If the robot detects a nearby wall within the alignment threshold, it adjusts its position to maintain a **consistent distance**.
4. **Move Forward**: If no turns are necessary, continue moving forward along the path.

The robot checks the path every **20 cm** to decide whether to turn, align, or move forward.

#### Navigation Algorithm
```cpp
void rightHandLogic() {
    float distRight, distFront;
    readSensors();
    distRight = measureRight.RangeMilliMeter;
    distFront = measureFront.RangeMilliMeter;

    if (distRight > 200.0) {
        Serial.println("Opening to the RIGHT -> turning right");
        turnRight90();
        moveForwardDistance(MOVE_DISTANCE);
    } else if (65.0 < distRight < 150) {
        Serial.println("Aligning to right wall");
        alignToRightWall();
    } else if (distFront < 80.0) {
        Serial.println("Blocked in front -> turning left");
        turnLeft90();
    } else {
        Serial.println("Moving forward...");
        moveForwardDistance(MOVE_DISTANCE);
    }
}
```

### PID-Based Motion Control
The robot implements **PID control** for:
- **Straight-line motion** using the IMU heading data.
- **Turn precision** by integrating gyroscope readings.

#### What is PID Control?
**PID (Proportional-Integral-Derivative) Control** is a feedback mechanism used to maintain stability and precision in movement.

#### PID Formula:
$$ \[
    u(t) = K_p \cdot e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}
\] $$

- **Proportional (Kp)**: Corrects based on the current error.
- **Integral (Ki)**: Adjusts based on accumulated past errors.
- **Derivative (Kd)**: Predicts future errors and applies correction.

PID Constants Used:
```cpp
const float Kp = 1.0;
const float Ki = 0.005;
const float Kd = 0.25;
```

#### Movement Functions
- `moveForwardDistance(float distance_cm)`: Moves the robot forward based on a target distance.
- `turnRight90()`: Executes a 90-degree turn to the right.
- `turnLeft90()`: Executes a 90-degree turn to the left.
- `alignToRightWall()`: Adjusts position to maintain a set distance from the right wall.

### Conclusion
This robot autonomously navigates an environment using the **right-hand rule**, PID-based heading correction, and distance measurements from **VL53L0X sensors**. It ensures **efficient obstacle avoidance** and can traverse structured pathways or simple mazes effectively.
