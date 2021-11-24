# uav_simulator
## Prerequisites
+ download webots_2021a_amd64.deb from https://github.com/cyberbotics/webots/releases/tag/R2021a
+ install webots and other packages
```Bash
$ sudo dpkg -i webots_2021a_amd64.deb
$ sudo echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt-get install ros-melodic-webots-ros
$ sudo apt-get install ros-melodic-joy
```
## Build
```Bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/TaoYibo1866/uav_simulator.git
$ cd ~/catkin_ws
$ catkin_make
```
## Run
```Bash
$ roslaunch uav_webots joy_control.launch
```