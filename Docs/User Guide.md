# ROS 2 SLAM Robot User Guide
The installation section of the user guide has been seperated into three subsections which is the software installation on the Raspberry Pi, the software installation on the base station (laptop) and finally the wiring diagrams/hardware integration.

**Pi Installation**
1. [Ubuntu Installation](https://github.com/JosephStew-art/ROS2-SLAM-PROJECT/blob/main/Docs/User%20Guide.md#ubuntu-2204-server-installation)
2. [ROS 2 Installation]()

**Base Station Installation**
1. [ROS 2 Installation]()
2. [ROS 2 Node Installation]()
3. [LDSO Installation]()

**Dataset Simulation**
1. [ORB-SLAM3 Dataset Simulations]()
2. [LDSO Dataset Simulations]()

## Raspberry Pi Software Installation
### Ubuntu 22.04 Server installation
1. Download the Raspberry Pi Imager [Raspberry Pi Software](https://www.raspberrypi.com/software/)
2. Insert the SD card into the SD card reader and plug it into the laptop
3. Open Raspbbery Pi Imager
4. Select the Pi you have (8GB RPi 4 Model B is recommended)
5. Select Ubuntu Server 22.04 for the OS
6. Select the SD card in the drives section of the install
7. When prompted to enter optional configuration information select yes
8. Ensure a WiFi network is configured as well as a user and password
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
```markdown
sudo apt install libunwind-dev
```
```markdown
sudo apt -y install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev gfortran openexr libatlas-base-dev python3-dev python3-numpy libtbb2 libtbb-dev libopenexr-dev libgstreamer-plugins-base1.0-dev libdc1394-dev
```
5. Clone ORB-SLAM3
```markdown
git clone https://github.com/JosephStew-art/ORB-SLAM3-Pi.git
```
6. Build ORB-SLAM3
```markdown
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

### ORB-SLAM3 ROS 2 node installation
1. Clone the ROS 2 ORB-SLAM3 wrapper
```markdown
mkdir -p colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/JosephStew-art/ORB-SLAM3-ROS2-Wrapper-Pi.git
```
2. Install the ROS 2 package
```markdown
cd ~/colcon_ws
colcon build --symlink-install --packages-select orbslam3
```
3. Add the source to bashrc
Edit the .bashrc file in the root directory and add teh following line to the end of the file
```markdown
source ~/colcon_ws/install/local_setup.bash
```

### LDSO installation
1. Clone the forked LDSO repo
```markdown
git clone https://github.com/JosephStew-art/LDSO.git
```
2. Install the dependencies
```markdown
./install_dependencies.sh
```
3. Install Pangolin (part of the ORB-SLAM3 installation)
4. Compile the project
```markdown
./make_project.sh
```

## Dataset Simulations and Evaluations
There are many public datasets that can be run on both of the mentioned SLAM systems, most prominent being the EuRoC, Kitti-Mono and TUM. The EuRoC dataset will be used to evaluate the performance of the systems.

### ORB-SLAM3 simulation
1. Download and unzip the dataset
```markdown
cd ~
mkdir -p Datasets/EuRoc
cd Datasets/EuRoc/
wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
mkdir MH01
unzip MH_01_easy.zip -d MH01/
```
2. Replace the corrupted images
```markdown
cd ~/Datasets/EuRoc/MH01/mav0/cam0/data
rm 1403636689613555456.png
cp 1403636689663555584.png 1403636689613555456.png
rm 1403636722213555456.png
cp 1403636722263555584.png 1403636722213555456.png
```
3. Go to the root ORB-SLAM3 directory
4. Run the EuRoC dataset
```markdown
./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt dataset-MH01_mono
```

### LDSO simulation
1. Download and unzip the dataset (see ORB-SLAM3 simulation instructions)
2. Run the dataset
```markdown
./bin/run_dso_euroc \
    preset=0 \
    files=XXXX/EuRoC/MH_01_easy/mav0/cam0/
```
