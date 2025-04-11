# Arduino-Based Robot-Car Project

## Overview

This project involves the development of an Arduino-based robot car with two operating modes:

1. **Manual Mode (Bluetooth Control via Mobile App)**: The robot car is controlled via a mobile app developed using MIT App Inventor, allowing precise control over its movement through a Bluetooth connection.

2. **Autonomous Mode (Obstacle Avoidance)**: The robot car automatically navigates its environment using ultrasonic sensors, maintaining a set distance of 7 cm from objects.


## Features

- **Mobile App Control**: A custom mobile app built with MIT App Inventor sends control signals to the robot via Bluetooth, allowing users to manually control the car's movement.
- **Autonomous Mode**: The car uses ultrasonic sensors to detect obstacles and adjust its movements, ensuring it maintains a 7 cm distance from objects.
- **PID Control Algorithm**: A Proportional-Integral-Derivative (PID) control system is employed to maintain a precise distance from obstacles during autonomous operation.


## Hardware Components

- **Arduino Uno**: The central microcontroller that handles inputs from sensors, the Bluetooth module, and controls the motors.
- **Motors and Motor Driver**: Two DC motors controlled via PWM outputs for speed control.
- **Ultrasonic Sensors**: Two ultrasonic sensors (HC-SR04) to measure distances to obstacles in autonomous mode.
- **Bluetooth Module**: The HC-05 Bluetooth module receives control commands from the mobile app.
- **Power Supply**: Two 9V batteries powering the car.


## Mobile App Features

Developed using MIT App Inventor, the app provides an easy-to-use interface for controlling the robot car.

- **Joystick Control**: The app includes a joystick feature to send X and Y axis data to the Arduino via Bluetooth.


## Code Functionality

- **Manual Mode (Mobile App Control)**:
  - The robot reads Bluetooth joystick inputs from the mobile app.
  - The X and Y axis values determine the robot's speed and direction.

- **Autonomous Mode**:
  - Ultrasonic sensors measure distances to obstacles.
  - A PID control system adjusts motor speeds to maintain a 7 cm distance from objects.


## Control Logic

The robot enters manual mode when the mode selector (analog pin A0) reads values below 511. The mobile app sends joystick commands to control the robot's movement.


## Possible Future Improvements

- **Enhancing the Mobile App**: Add additional features, such as speed control sliders and status indicators for battery life.
- **Improving PID Tuning**: Optimize the PID algorithm for smoother and more efficient autonomous navigation.
- **Adding Additional Sensors**: Incorporate extra sensors for more advanced obstacle detection and avoidance.

---
