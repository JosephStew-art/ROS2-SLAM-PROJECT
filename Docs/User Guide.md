# ROS 2 SLAM Robot User Guide
The installation section of the user guide has been seperated into three subsections which is the software installation on the Raspberry Pi, the software installation on the base station (laptop) and finally the wiring diagrams/hardware integration.

## Raspberry Pi Software Installation
### Ubuntu 22.04 Server installation
1. Download the Raspberry Pi Imager [Raspberry Pi Software](https://www.raspberrypi.com/software/)
2. Insert the SD card into the SD card reader and plug it into the laptop
3. Open Raspbbery Pi Imager
4. Select the Pi you have (8GB RPi 4 Model B is recommended)
5. Select Ubuntu Server 22.04 for the OS
6. Select the SD card in the drives section of the install
7. When prompted to enter optional configuration information select yes
8. Ensure a WiFi networks is configured as well as a user and password
9. Flash the SD card
### ROS 2 Humble installation
1. Set locale
```markdown
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
2. Setup sources\
```markdown
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Add the ROS 2 GPG key using apt
```markdown
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Add the repo to the sources list
```markdown
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
3. Install ROS 2 packages
```markdown
sudo apt update
sudo apt upgrade
```
```markdown
sudo apt install ros-humble-desktop
```
```markdown
sudo apt install ros-dev-tools
```
4. Edit the .bashrc file in the route directory and add the following line at the bottom
```markdown
source /opt/ros/humble/setup.bash
```

## Base Station Software Installation
The laptop must be running Ubuntu 22.04 (Tier 1 Humble support)

### ROS 2 Humble installation
See the method above on how to install [ROS 2 Humble](https://github.com/JosephStew-art/ROS2-SLAM-PROJECT/blob/main/Docs/User%20Guide.md#ros-2-humble-installation)

### ORB-SLAM3 installation
1. Install build essentials
```markdown
sudo apt-get install build-essentials make cmake
```
2. Install Eigen3
```markdown
sudo apt install libeigen3-dev
```
3. Install Pangolin 0.8
```markdown
wget https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.8.zip
```
```markdown
unzip v0.8.zip
```
```markdown
cd Pangolin-0.8/scripts/
```
```markdown
sudo ./install_prerequisites.sh
```
```markdown
cd ..
```
```markdown
mkdir build
```
```markdown
cd build/
```
```markdown
sudo cmake ..
```
```markdown
sudo cmake --build .
```
4. Install OpenCV dependencies
```markdown
sudo apt install libgtk2.0-dev libcanberra-gtk-module
```
