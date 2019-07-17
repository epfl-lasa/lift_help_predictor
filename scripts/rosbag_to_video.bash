#!/bin/bash
# BASED ON TUTORIAL: http://wiki.ros.org/rosbag/Tutorials/Exporting%20image%20and%20video%20data
VIDEO_FILE_NAME=$1
BAG_FILE_PATH=/home/leonardo/catkin_ws/src/lift_help_predictor/data/bag/test_2019-07-09-14-03-05.bag
 
PACKAGE_PATH=$(rospack find lift_help_predictor)
TEMP_PATH=/data/tmp/
VIDEO_PATH=/data/video/
BAG_PATH=/data/bag

cd $PACKAGE_PATH$TEMP_PATH

# Convert to images in a temporary folder
rosrun image_view extract_images image:=/usb_cam/image_raw _sec_per_frame:=0.01 __name:=extract_images &
rosbag play $BAG_FILE_PATH
rosnode kill extract_images

#Generate video at 30fpm
ffmpeg -framerate 30 -i frame%04d.jpg -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p "$VIDEO_FILE_NAME"

#Cleanup TEMP 
rm -rf frame*.jpg

# Move to VIDEO_PATH and play result
mv $PACKAGE_PATH$TEMP_PATH$VIDEO_FILE_NAME $PACKAGE_PATH$VIDEO_PATH$VIDEO_FILE_NAME
cd $PACKAGE_PATH/$VIDEO_PATH
vlc $VIDEO_FILE_NAME

