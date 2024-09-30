# ROS 2 SLAM Project
Converting a differential drive robot to ROS 2 and implementing SLAM with as few sensors as possible.

<div style="display: flex; justify-content: space-between;">
  <img src="https://github.com/user-attachments/assets/ffbd1a9c-8ed7-4ff4-92c9-08b66369995b" alt="Robot Base Side View" style="width: 65%;">
</div>

## Dependencies
- ORB-SLAM3
- OpenCv v4
- ROS 2 Humble
- Pangolin
- Eigen3

## Requirements
- Raspberry Pi 4 B or similar running Ubuntu Server 22.04
- Laptop running Ubuntu 22.04
- Webcam (Logitech BRIO)
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
- SD card reader

## User Manual
For a detailed installation and usage guide consult [User Guide](https://github.com/JosephStew-art/ROS2-SLAM-PROJECT/blob/main/Docs/User%20Manual.md).

### Raspberry Pi Software Installation
1. Download the Raspberry Pi Imager [Raspberry Pi Software](https://www.raspberrypi.com/software/)
2. Insert the SD card into the SD card reader and plug it into the laptop
3. Open Raspbbery Pi Imager
4. Select the Pi you have (8GB RPi 4 Model B is recommended)
5. Select Ubuntu Server 22.04 for the OS in the 
