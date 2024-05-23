#This is an example of how to get the dds config file into the necessary environment variable from this folder
#Can simply copy the following lines to the end of ~/.bashrc

# Source the ROS 2 installation
source /opt/ros/foxy/setup.bash

# Source your workspace
source ~/ros2_ws/install/setup.bash

# Get the path to the fastrtps_profiles.xml file
config_path=$(python3 -c "import os; from ament_index_python.packages import get_package_share_directory; print(os.path.join(get_package_share_directory('grunt_bringup'), 'config', 'fastrtps_profiles.xml'))")

# Export the DDS environment variable
export FASTRTPS_DEFAULT_PROFILES_FILE=$config_path