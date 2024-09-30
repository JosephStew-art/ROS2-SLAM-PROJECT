# ROS 2 SLAM Project
Converting a differential drive robot to ROS 2 and implementing SLAM with as few sensors as possible.

<div style="display: flex; justify-content: space-between;">
  <img src="https://github.com/user-attachments/assets/26666ae7-dfb7-417f-a60e-602a96935341" alt="Robot Base Front View" style="width: 49%;">
  <img src="https://github.com/user-attachments/assets/ffbd1a9c-8ed7-4ff4-92c9-08b66369995b" alt="Robot Base Side View" style="width: 48%;">
</div>


## Requirements
- Raspberry Pi 4 B or similar running Ubuntu Server 22.04
- Laptop running Ubuntu 22.04
- Webcam
- WiFi network
- Differential drive robot (DFROBOT Turtle 2WD was used)
- DC motors x2
- Encoders (10 tooth rotary encoder) x2
- L298N H-bridge
- Battery (4S Li-ion pack)
- Carbon rod/wooden dowel (75/80mm diameter, 300mm long)
- Jumper wire female to female x15
- 5V 6S UBEC
- USB-A to USB-C (if required for webcam)
- 32GB SD card

## Installation
The installation is going to be split into three sections. It will start with the installation of all the software requirements for the Raspberry Pi, then it will move onto the software installation/s for the laptop base station and then thirdly it will detail the instructions for setting up the robot and integrating all the parts as well as runnign the robot.

### Raspberry Pi Software Installation
