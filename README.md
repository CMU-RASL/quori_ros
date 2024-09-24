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


## Running on the robot + second computer at CMU
```
cd quori_files/quori_ros
export ROS_IP=$(hostname -I | awk '{print $1;}')
export ROS_MASTER_URI=http://quori.wifi.local.cmu.edu:11311 
export ROS_HOSTNAME=$ROS_IP
source devel/setup.bash
```

## Running on two computers or robot + second computer not at CMU
- First run `hostname -I` to determine the hostname on the master computer
```
export ROS_IP=$(hostname -I | awk '{print $1;}')
export ROS_MASTER_URI=http://{{HOSTNAME HERE}}:11311 
export ROS_HOSTNAME=$ROS_IP
source devel/setup.bash
```

export ROS_IP=$(hostname -I | awk '{print $1;}')
export ROS_MASTER_URI=http://192.168.1.88:11311 
export ROS_HOSTNAME=$ROS_IP
source devel/setup.bash

## Running only on one machine
```
cd quori_files/quori_ros
export ROS_IP=localhost
export ROS_MASTER_URI=http://localhost:11311/ 
export ROS_HOSTNAME=$ROS_IP
source devel/setup.bash
```

## Useful commands

- To record the video `rosbag record astra_ros/devices/default/color/image_color`

- Check the camera frame `rosrun image_view image_view image:=/astra_ros/devices/default/color/image_color`

- Check the sound `rostopic pub /quori_sound std_msgs/String "Hello, my name is Quori"`

- Convert bag file to mp4 `python rosbag2video.py {{BAG FILE NAME}}`

## Different run cases

1. Run a study with HR monitor with Quori
    - Quori: Run the pose tracking, camera, sound, face, HR `roslaunch quori_launch study_session_Quori.launch`
        - This runs the rotated face `simVectors_Quori.py`
        - This runs the pose_tracking with the Quori directories `pose_tracking_Quori.py` and `config_Quori.py`
    - RASL Computer: Run the exercise session: `rosrun quori_exercises study_session_Quori.py`
        - This pulls the config file with the computer directories `config_computer.py`
        - This uses the full ExerciseController with HR
    - TBD Computer: Run the wizard of oz: `rosrun quori_exercises wizard_of_oz_speech.py`
2. Run a demo with Quori and no HR
    - Quori: Run the pose tracking, camera, sound, face (no HR) `roslaunch quori_launch demo_session_Quori.launch`
        - This runs the rotated face `simVectors_Quori.py`
        - This runs the pose_tracking with the Quori directories `pose_tracking_Quori.py` and `config_Quori.py`
    - RASL Computer: Run the demo session: `rosrun quori_exercises demo_session.py`
        - This pulls the config file with the computer directories `config_computer.py`
        - This uses the modified ExerciseController with no HR `ExerciseController_computer`
    - TBD Computer: Run the wizard of oz: `rosrun quori_exercises wizard_of_oz_speech.py`
3. Run a demo with only computers/monitors `roslaunch quori_launch demo_session_main_computer.launch`
    - TBD Computer (Master): Run the USB camera, not rotated face, and sound
        - This runs the not rotated `simVectors_computer_demo.py`
        - This uses a USB camera - make sure you have the correct device id
    - RASL Computer: Run the demo session: `roslaunch quori_launch demo_session_second_computer.launch`
        - This runs the pose_tracking with the computer directories `pose_tracking_computer.py` which pulls the config with computer directories `config_computer.py`
        - This uses the modified ExerciseController with no HR `ExerciseController_computer`

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
