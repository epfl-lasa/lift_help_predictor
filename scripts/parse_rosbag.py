#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
  CONVERTS ROSBAG RECORDINGS INTO SINGLE .CSV FILES FOR EACH RECORDING.
  
  Note: running this process overwrites current .csv files by same name.
  Author: Jayden Leong 
"""
import __init__
import time
import signal
import rospy
import rosbag
import cv2
import matplotlib.pyplot as plt
import csv
import os
import string
import numpy as np

from lifting import PoseEstimator
from lifting.utils import draw_limbs
from lifting.utils import plot_pose
from os.path import dirname, realpath
from cv_bridge import CvBridge
from rospkg import RosPack

PROJECT_PATH = RosPack().get_path('lift_help_predictor') + '/src/Lifting-from-the-Deep-release'

# Data needed for 'Lifting from the deep' algorithm 
SAVED_SESSIONS_DIR = PROJECT_PATH + '/data/saved_sessions'
SESSION_PATH = SAVED_SESSIONS_DIR + '/init_session/init'
PROB_MODEL_PATH = SAVED_SESSIONS_DIR + '/prob_model/prob_model_params.mat'

# Ensure data in correct folder in 'lift_help_predictor' package.
BAG_DIR_PATH = RosPack().get_path('lift_help_predictor') + '/data/bag'
CSV_DIR_PATH = RosPack().get_path('lift_help_predictor') + '/data/csv'

# The joints given by the 'lifting from the deep' algorithm, in order.
UCL_JOINT_NAMES = [
    'pelvis', 'right_hip', 'right_knee', 'right_foot', 'left_hip', 'left_knee',
    'left_foot', 'abdomen', 'neck', 'chin', 'forehead', 'left_shoulder', 'left_elbow',
    'left_hand', 'right_shoulder', 'right_elbow', 'right_hand'
]

def main():
  parse_rosbag()

def parse_rosbag(): 
  rospy.init_node("parse_rosbag", anonymous=True)
  participant_name = rospy.get_param("~participant") # can make this for multiple people
  topic_names = rospy.get_param('~topic_names')

  # Fetch image_topic
  for topic in topic_names: 
    if 'image_raw' in topic:
      image_topic = topic 

  # Process all ROSBAG recordings for each participant. 
  for file_name in os.listdir(os.path.join(BAG_DIR_PATH)):
    if participant_name in file_name:
      print("Processing " + file_name + "...")
      last_mocap_msgs = {}  # Used to downsample motion capture to camera sampling rate
      modelInitialized = False

      bag_file_path = os.path.join(os.path.join(BAG_DIR_PATH), file_name)
      bag = rosbag.Bag(bag_file_path)
      msgs = bag.read_messages(topic_names)
      
      csv_dir = os.path.join(CSV_DIR_PATH)
      csv_file_name = file_name.split('.bag')[0] + ".csv"

      with open(os.path.join(csv_dir, csv_file_name), 'wb') as csvfile:
        csv_writer = csv.writer(csvfile, delimiter=',', quotechar=' ', quoting=csv.QUOTE_MINIMAL)
        write_csv_field_names(csv_writer, UCL_JOINT_NAMES, topic_names)

        for subtopic, msg, t in msgs:
          # Downsample to rate of camera. Take last sample for motion capture. 
          if subtopic == image_topic:   
            image = CvBridge().imgmsg_to_cv2(msg)

            if not modelInitialized:
              pose_estimator = PoseEstimator(image.shape, SESSION_PATH, PROB_MODEL_PATH)
              pose_estimator.initialise()
              modelInitialized = True
              print("Processing " + file_name + "...")

            # Run 'lifting from the deep algorithm'
            pose_2d, visibility, pose_3d = pose_estimator.estimate(image) 

            # Write motion capture and 'Lifting from the deep' output to .csv.
            write_data_to_csv(csv_writer, pose_3d, last_mocap_msgs, topic_names, t)
          else:
            last_mocap_msgs[subtopic] = msg  # Save most recent sample for downsampling

  # close model for lifting from the deep' algorithm
  pose_estimator.close()

def write_data_to_csv(csv_writer, ucl_joints, last_mocap_msgs, topic_names, timestamp):
  row = []
  row.append(str(timestamp.secs) + "." + str(timestamp.nsecs))

  # 'Lifting from the deep' output to .csv
  if( isinstance(ucl_joints, np.ndarray) ): 
    single_person = ucl_joints[0]
    for joint in range(0,len(UCL_JOINT_NAMES)):
      if(joint < len(single_person[0])):
        row.append(single_person[0][joint]) 
        row.append(single_person[1][joint])
        row.append(single_person[2][joint])
      else:
        for i in range(0, 2):
          row.append(0.0) #handle case where missing joint data.
  else:
    for k in range(0, len(UCL_JOINT_NAMES)*3):
        row.append(0.0) # handle case where no joints show up.
  
  # Motion capture data to .csv
  for topic in topic_names:
    if 'image' not in topic:
      if topic in last_mocap_msgs:
        msg  = last_mocap_msgs[topic]
        row.append(msg.pose.position.x)
        row.append(msg.pose.position.y)
        row.append(msg.pose.position.z)
      else:
        row.append(0.0) # replace with default zeros if marker doesn't appear at first.
        row.append(0.0)
        row.append(0.0)

  csv_writer.writerow(row)

# Write variable names into the .csv file to define format
def write_csv_field_names(csv_writer, joint_names, topic_names):  
  top_row = ['/time/unix_epoch_secs'] 

  # 'Lifting from the deep' field names
  for joint in joint_names:
    top_row.append("/ucl/" + joint + "/x")
    top_row.append("/ucl/" + joint + "/y")
    top_row.append("/ucl/" + joint + "/z") 

  # Motion capture field names 
  for topic in topic_names:
    if 'image' not in topic:
      top_row.append(topic + "/x") 
      top_row.append(topic + "/y")
      top_row.append(topic + "/z")

  csv_writer.writerow(top_row)

def display_results(in_image, data_2d, joint_visibility, data_3d):
    """
      FOR VISUALIZATION OF 'LIFTING FROM THE DEEP' OUTPUT FOR ONE FRAME
      Plot 2D and 3D poses for each of the people in the image.
    """
    plt.figure()
    draw_limbs(in_image, data_2d, joint_visibility)
    plt.imshow(in_image)
    plt.axis('off')

    # Show 3D poses
    for single_3D in data_3d:
        # or plot_pose(Prob3dPose.centre_all(single_3D))
        plot_pose(single_3D)

    plt.show()

# From here: https://stackoverflow.com/questions/25027122/break-the-function-after-certain-time
class TimeoutException(Exception):   # Custom exception class
    pass

def timeout_handler(signum, frame):   # Custom signal handler
    raise TimeoutException

if __name__ == '__main__':
    import sys
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
