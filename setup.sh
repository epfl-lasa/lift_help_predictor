sudo apt-get update
# To connect with motion capture 
sudo apt-get install ros-$(rosversion -d)-vrpn-client-ros
# To display images incoming through usb cam
sudo apt-get install ros-$(rosversion -d)-image-view
# To record rosbag files 
roscd lift_help_predictor/../
git clone https://github.com/epfl-lasa/record_ros

# Create directories to store parsed data
roscd lift_help_predictor/ 
mkdir -p data/csv

# Setup model and dependencies for the Lifting from the deep algorithm.
source src/Lifting-from-the-Deep-release/setup.sh