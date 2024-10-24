# ROS 2 SLAM Robot User Guide
The installation section of the user guide has been seperated into four subsections which is the software installation on the Raspberry Pi, the software installation on the base station (laptop), the wiring diagrams/hardware integration and the running of SLAM in a live environment.

**Pi Installation**
1. [Ubuntu Installation](https://github.com/JosephStew-art/ROS2-SLAM-PROJECT/blob/main/Docs/User%20Guide.md#ubuntu-2204-server-installation)
2. [ROS 2 Installation](https://github.com/JosephStew-art/ROS2-SLAM-PROJECT/blob/main/Docs/User%20Guide.md#ros-2-humble-installation)

**Base Station Installation**
1. [ROS 2 Installation](https://github.com/JosephStew-art/ROS2-SLAM-PROJECT/blob/main/Docs/User%20Guide.md#ros-2-humble-installation-1)
2. [ORB-SLAM3 Installation](https://github.com/JosephStew-art/ROS2-SLAM-PROJECT/blob/main/Docs/User%20Guide.md#orb-slam3-installation)
3. [ROS 2 Node Installation](https://github.com/JosephStew-art/ROS2-SLAM-PROJECT/blob/main/Docs/User%20Guide.md#orb-slam3-ros-2-node-installation)
4. [LDSO Installation](https://github.com/JosephStew-art/ROS2-SLAM-PROJECT/blob/main/Docs/User%20Guide.md#ldso-installation)

**Dataset Simulation**
1. [ORB-SLAM3 Dataset Simulations](https://github.com/JosephStew-art/ROS2-SLAM-PROJECT/blob/main/Docs/User%20Guide.md#orb-slam3-simulation)
2. [LDSO Dataset Simulations](https://github.com/JosephStew-art/ROS2-SLAM-PROJECT/blob/main/Docs/User%20Guide.md#ldso-simulation)

**Component Installation and Wiring**
[Component Installation and Wiring]([https://github.com/JosephStew-art/ROS2-SLAM-PROJECT/blob/main/Docs/User%20Guide.md#component-installation-and-wiring])

**Live SLAM**
[Live SLAM Instructions]([https://github.com/JosephStew-art/ROS2-SLAM-PROJECT/blob/main/Docs/User%20Guide.md#live-slam-using-a-webcam])


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
2. Change to the livestream branch
```markdown
git checkout livestream
```
4. Install the ROS 2 package
```markdown
cd ~/colcon_ws
colcon build --symlink-install --packages-select orbslam3
```
4. Add the source to bashrc
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

## Component Installation and Wiring
Consult the image of the system in the root README.md file that shows the positioning of the components. Follow the wiring diagram in x to connect the components together. 





## Live SLAM Using A Webcam
Setting up the respective SLAM frameworks to run in live SLAM environments by using a webcam for a monocular video feed.

### Webcam calibration
To be able to use a webcam with any visual-based SLAM it needs to be calibrated and its intrinsic parameters found.
1. Run the python calibration script [Calibration Script](https://github.com/JosephStew-art/ROS2-SLAM-PROJECT/blob/main/Camera%20Calibration/camera_calibration.py)
2. Position a 7x9 checkerboard 2-3 meters away from the camera and use the keyboard shortcuts to take the 15 pictures, be sure to change the position of the checkerboard between pictures for a thorough result.
3. The script will print out the intrinsic parameters after the images have been captured and annotated. Copy these and replace the values in the used camera calibration yaml file ([Calibration File](https://github.com/JosephStew-art/ORB-SLAM3-ROS2-Wrapper-Pi/blob/main/config/monocular/brio.yaml)) with these new values.
4. For the LDSO camera calibration the values need to be adjusted before they can be used for the system. The RadTan format will be used. fx, fy, cx, cy need to be adjusted before being put into the new calibration file. fx = Camera.fx / Camera.width where fx is the fx value from step 3. Do the rest for: fy = Camera.fy / Camera.height, cx = Camera.cx / Camera.width, cy = Camera.cy / Camera.height. The following: k1, k2, p1, p2 remain the same where r1 = p1 and r2 = p2. The layout is as follows:
```markdown
RadTan fx fy cx cy k1 k2 r1 r2
in_width in_height
"crop" / "full" / "fx fy cx cy 0"
out_width out_height
```
For example:
```markdown
RadTan 1.1480625110012206 1.5441734314381511 0.4899086033719993 0.43901736252522903 0.016376049307530022 0.020272006835536698 -0.011690874094127815 -0.0006645146840101754
640 480
crop
640 480
```

### Initialalising The Robot and Controlling It

Once the camera has been calibrated the robot can be run.

1. Run the launch file on the Pi (ensure that the workspace is courced)
```markdown
ros2 launch ros2_robot_package robot_nodes.launch.py
```
2. Wait for a few seconds for the Pi to initialise all the nodes
3. Run the launch file on the base station (ensure that the workspace is courced)
```markdown
ros2 launch orbslam3 slam_and_control.launch.py
```
4. Wait for the ORB-SLAM3 vocaulary to load and all systems to initialise
5. Control the robot via keyboard presses on the base station (w, a, s, d and x for forwards, anti-clockwise rotation, backwards, clockwise rotation and to stop respectively)
6. To end the SLAM system click on the "stop" button on the map viewer GUI followed by ctrl + c in the base station terminal and lastly followed by ctrl + c in the Pi terminal. 
