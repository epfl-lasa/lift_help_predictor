sudo apt-get update
# To connect with motion capture 
sudo apt-get install ros-$(rosversion -d)-vrpn-client-ros
# To display images incoming through usb cam
sudo apt-get install ros-$(rosversion -d)-image-view
# To record rosbag files 
roscd lift_help_predictor/../
git clone https://github.com/epfl-lasa/record_ros

# Setup model and dependencies for the Lifting from the deep algorithm.
roscd lift_help_predictor/src/Lifting-from-the-Deep-release
source setup.sh