#call with sudo: 
#sudo bash grunt_install.sh

#install  required packages
#expects the normally installed base ROS2 packages are already installed

#imu_tools: imu_complementary_filter | imu_filter_madgwick | rviz_imu_plugin
#See https://github.com/CCNYRoboticsLab/imu_tools/tree/humble
apt install ros-${ROS_DISTRO}-imu-tools

apt install ros-${ROS_DISTRO}-robot-localization

#
#get git packages to be installed from source
#

#imu tools if  installed from  source
#git clone -b humble https://github.com/CCNYRoboticsLab/imu_tools.git

#Vizanti visualizer and mission planner
cd ~/ros2_ws/src
git clone -b ros2 https://github.com/MoffKalast/vizanti.git

cd ..
rosdep install -i --from-path src/vizanti -y
colcon build

