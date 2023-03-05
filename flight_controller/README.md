# flight controller

This is a doc for studying mavros+PX4 flight controller;

Instead of the typical control law design, I mainly focus on autonomous flight using ros messages and PX4 controller.

## PX4-Autopilot configuration

PX4 has upgraded to 1.14 beta until March 5th 2023, and keeps updating.
So it's quite different for the SITL configuration docs that you can see before this stage,
but basically they are changes of file locations.

**make sure you have ROS+gazebo+mavros!**

```shell
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
sudo bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
# change your ros version
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
sudo apt install axel
axel https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
make px4_sitl gazebo
```

If everything goes well here, adding PATH to `~/.bashrc` so that PX4 can link up with ros.

```shell
# source ros
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
# source PX4
source ~/Applications/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/Applications/PX4-Autopilot/ ~/Applications/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Applications/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Applications/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
```

This part has changed a lot(if you ever used PX4 before).

Configuration done here.