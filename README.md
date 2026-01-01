
# ESP32 MPU6050 Jump Game 🎮

A motion-controlled browser game where a ball moves and jumps using real-time motion data from an MPU6050 sensor connected to an ESP32.

## Project Overview
This project combines IoT hardware with a web-based game interface.  
Tilting the MPU6050 controls the ball movement, and tilting upward makes the ball jump.

## Hardware Used
- ESP32-WROOM
- MPU6050 (Accelerometer + Gyroscope)
- LDR
- Breadboard & jumper wires

## Software / Technologies
- Arduino (ESP32)
- HTML, CSS, JavaScript
- WebSockets
- HTML Canvas

## How It Works
- ESP32 reads accelerometer and gyroscope data from MPU6050
- Sensor data is sent to the browser using WebSockets over WiFi
- The browser game reacts in real time to sensor movements

## Game Controls
- Tilt Left / Right → Move ball
- Tilt Backward (Up) → Jump
- Avoid red obstacles and reach the blue finish line

## Team
- Dharshini priya.S
- Kiruthik.G
  

