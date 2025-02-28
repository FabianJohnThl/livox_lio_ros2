# Livox-LIO with MID360 using ROS2-Humble on Ubuntu 22.04 Tutorial

- This Tutorial provides information to get started with MID360 Lidar

## Ubuntu

- new installation of Ubuntu 22.04 Desktop with default packages
- set the network interface where the MID360 is connected with to:
```
IP: 192.168.1.5
SUBNET: 255.255.255.0
Gateway: 192.168.1.1
```
- enable performance-mode

## ROS-HUMBLE

- installation steps from [ROS humble install](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

```
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop -y
sudo apt install ros-humble-ros-base -y
sudo apt install ros-dev-tools -y

echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
cd ~
# create ros2_ws (Workspace)
mkdir ros2_ws && cd ros2_ws
mkdir src && cd src
cd ~
```

- NOW: Test the installation with the talker (should produce some Hello World messages)

```
ros2 run demo_nodes_cpp talker
```

## livox SDK from source

```
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
cd ~
```

## livox ROS2 driver

```
cd ~
cd ros2_ws/src/
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
./build.sh humble
```

## livox LIO for ROS2

### Prerequisites

- suitesparse, PCL

```
sudo apt install libsuitesparse-dev libpcl-dev -y
sudo apt-get install ros-humble-pcl-ros
```

- google log

```
cd ~
sudo rm -r glog
git clone https://github.com/google/glog.git
cd glog
mkdir build && cd build
cmake .. && make -j && sudo make install
cd ~
```

- ceres in v 2.1.0 from source:
```
cd ~
sudo rm -r ceres-solver
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver
git checkout 2.1.0
git submodule update --init --recursive
mkdir build && cd build
cmake .. && make -j && sudo make install
cd ~
```
### installation

```
cd ~
cd ros2_ws/src
sudo rm -r livox_lio_ros2
git clone https://github.com/FabianJohnThl/livox_lio_ros2.git
cd livox_lio_ros2
git checkout ros2
cd .. && cd ..
source ./install/setup.bash # necessary to locate livox_ros_driver2 installation
colcon build --packages-select lio_livox
ln ~/start_bundle.sh src/livox_lio_ros2/start-scripts/start_bundle.sh
sudo chmod +X ~/start_bundle.sh
sudo chmod 777 ~/start_bundle.sh
ln ~/start_lidar.sh src/livox_lio_ros2/start-scripts/start_lidar.sh
ln ~/start_lio.sh src/livox_lio_ros2/start-scripts/start_lio.sh
ln src/livox_ros_driver2/launch_ROS2/MID360_LIO_launch.py src/livox_lio_ros2/ros_driver_conf/launch_ROS2/MID360_LIO_launch.py
ln src/livox_ros_driver2/config/MID360_LIO.json src/livox_lio_ros2/ros_driver_conf/launch_ROS2/MID360_LIO.json
```

## Configuration

- We use the Livox Custom Msg format here. Therefore we use the launch file: `~/ros2_ws/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`
  ```
  nano ~/ros2_ws/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py
  # change the publish_freq to 20.0
  # check that xfer_format is 1

  nano ~/ros2_ws/src/livox_ros_driver2/config/MID360_config.json
  # check the correctness of the IP-Address of your host and Lidar
  # apply your specific extrinsic_parameters
  ```
  

## Run everything

- If you use the default pathes from here, you can use the following bash scripts to start everything (make sure, screen is installed `sudo apt install screen`)
- The start_bundle.sh starts everything in a screen. With start_bundle.sh the old screens are terminated and it can be used to restart everything. Also colcon to build the packages is performed with the shell scripts. Changes in the `src` are taken 

```
~/start_bundle.sh

```
