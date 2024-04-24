# quori_ros

This catkin workspace contains all of the ROS packages necessary for full operation of the Quori robot platform from UPenn.

## Prerequisites

The package is compatible with **Ubuntu 20.04 / ROS Noetic**, built with the **Catkin** system. Make sure you have the right environment configured. This documentation assumes you are using noetic, so replace it with kinetic if needed.

## Initial Setup

1. Install Visual Studio Code through the Software Center

2. Install Github Desktop (optional, but nice to have)
```
sudo wget https://github.com/shiftkey/desktop/releases/download/release-3.1.1-linux1/GitHubDesktop-linux-3.1.1-linux1.deb
sudo apt-get install gdebi-core 
sudo gdebi GitHubDesktop-linux-3.1.1-linux1.deb
```

3. Follow the ROS Noetic Installation (http://wiki.ros.org/noetic/Installation/Ubuntu)

4. Clone Repository
```
mkdir quori_files
cd quori_files
git clone https://github.com/CMU-RASL/quori_ros.git
cd quori_ros
git submodule init
```

5. Add repository to Github Desktop (optional)

6. Install helper repositorities
```
sudo apt-get install ros-noetic-sound-play ros-noetic-rgbd-launch ros-noetic-libuvc-camera ros-noetic-libuvc-ros
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control
sudo apt-get install python3-catkin-tools ros-noetic-rqt-joint-trajectory-controller python3-catkin-tools
sudo apt-get install python3-pip libglfw3 libglfw3-dev
rosdep update
rosdep install --from-paths src --ignore-src -r -y
pip install mediapipe fastdtw scikit-learn fer moviepy tensorflow
pip install pygame gtts pyinput
```

7. Build the workspace
```
catkin config --init
catkin build
source devel/setup.bash
```
## Set-up sound
Go to Settings in Ubuntu and make sure the output is set to Headphones and the volume is all the way up


## Running on the robot + second computer
```
cd quori_files/quori_ros
export ROS_IP=$(hostname -I | awk '{print $1;}')
export ROS_MASTER_URI=http://quori.wifi.local.cmu.edu:11311 
export ROS_HOSTNAME=$ROS_IP
source devel/setup.bash
```

## Running only on one machine
```
cd quori_files/quori_ros
export ROS_IP=localhost
export ROS_MASTER_URI=http://localhost:11311/ 
export ROS_HOSTNAME=$ROS_IP
source devel/setup.bash
```

## Exercise Session setup

### Robot
- Terminal 1 `roslaunch quori_launch quori_robot_main.launch`
- Terminal 2 (to record the video) `rosbag record astra_ros/devices/default/color/image_color`

### Second Computer
- Terminal 1 (to check the camera frame) `rosrun image_view image_view image:=/astra_ros/devices/default/color/image_color`

- Terminal 2 
1. `rosrun quori_exercises test_intake.py`
2. `rosrun quori_exercises study_session.py`

## General Troubleshooting

## Fix the Wifi on new laptop
```
sudo apt update
sudo apt install git build-essential
git clone https://git.kernel.org/pub/scm/linux/kernel/git/iwlwifi/backport-iwlwifi.git
cd backport-iwlwifi/
make defconfig-iwlwifi-public
sed -i 's/CPTCFG_IWLMVM_VENDOR_CMDS=y/# CPTCFG_IWLMVM_VENDOR_CMDS is not set/' .config
make -j4
sudo make install
sudo modprobe iwlwifi
```

### Fix the Astra Issue
`export ASTRA_ROOT=/opt/AstraSDK-v2.1.1-24f74b8b15-20200426T014025Z-Ubuntu18.04-x86_64`
