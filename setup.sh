sudo apt-get update
# To connect with motion capture 
sudo apt-get install ros-$(rosversion -d)-vrpn-client-ros
# To display images incoming through usb cam
sudo apt-get install ros-$(rosversion -d)-image-view
# To record rosbag files 
roscd lift_help_predictor/../
git clone https://github.com/epfl-lasa/record_ros

# Create directories to store recorded and parsed data
roscd lift_help_predictor/ 
mkdir -p data/csv
mkdir -p data/bag