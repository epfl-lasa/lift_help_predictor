#!/usr/bin/env python
# -*- coding: utf-8 -*-
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
import numpy as np

# #TODO: switch out so this is not needed.
# SAMPLING_RATE_POSE = 120.0 

# RUNNING_AVE_SAMPLES = 10
rospy.init_node('compute_feature_metrics', anonymous=True)
SAMPLING_RATE_POSE = rospy.get_param("~optitrack_rate_hz")
RUNNING_AVE_SAMPLES = rospy.get_param('~samples_in_average')

# Dictionary to hold past samples to compute velocity, acceleration, etc. 
past = {} 

# Holds all publisher objects for accel and velocity.
pubs = {}

def main():
    published_topics = rospy.get_published_topics()

    for topic in published_topics:
        topic_name = topic[0]
        topic_type = topic[1]
        
        # Compute metrics for all PoseStamped Ros messages in Rosbag file.
        if(topic_type == "geometry_msgs/PoseStamped"):
            # Store publishers in a data structure
            pubs[topic_name] = {}
            pubs[topic_name]['accel'] = rospy.Publisher(topic_name.split("/pose")[0] + '/accel', AccelStamped, queue_size=10)
            pubs[topic_name]['vel'] = rospy.Publisher(topic_name.split("/pose")[0] + '/vel', TwistStamped, queue_size=10)

            past[topic_name] = {}

            rospy.Subscriber(
                topic_name,
                PoseStamped,
                callback,
                topic_name
            )

    rospy.spin()

def callback(data, topic_name): 
    vel = compute_velocity(data, topic_name)
    pubs[topic_name]['vel'].publish(vel)
    accel = compute_accel(vel, topic_name)
    pubs[topic_name]['accel'].publish(accel)

# TODO: merge with compute_vel to avoid boilerplate code
def compute_accel(current_vel, topic_name):
    current_accel = AccelStamped()
    current_accel.header.stamp.secs = current_vel.header.stamp.secs
    current_accel.header.stamp.nsecs = current_vel.header.stamp.nsecs

    ave_accel = AccelStamped()
    ave_accel.header.stamp.secs = current_vel.header.stamp.secs
    ave_accel.header.stamp.nsecs = current_vel.header.stamp.nsecs

    # Only compute once we have two samples to compute acceleration.
    if 'ave_vel' in past[topic_name]:
        past_vel = past[topic_name]['ave_vel']

        # Calculate times, measured in seconds
        current_t = float(current_vel.header.stamp.secs + 1e-9*current_vel.header.stamp.nsecs)
        past_t = float(past_vel.header.stamp.secs + 1e-9*past_vel.header.stamp.nsecs)
        delta_t = current_t - past_t

        #Calculate current acceleration
        current_accel.accel.linear.x = (current_vel.twist.linear.x - past_vel.twist.linear.x)/delta_t
        current_accel.accel.linear.y = (current_vel.twist.linear.y - past_vel.twist.linear.y)/delta_t
        current_accel.accel.linear.z = (current_vel.twist.linear.z - past_vel.twist.linear.z)/delta_t
        
        # Update acceleration buffer
        if(len(past[topic_name]['accel']) < RUNNING_AVE_SAMPLES):
            past[topic_name]['accel'].append(current_accel)
        else:
            del past[topic_name]['accel'][0]
            past[topic_name]['accel'].append(current_accel)

        # Compute average
        sum = np.array([0.0, 0.0, 0.0])
        for i in range(0, len(past[topic_name]['accel'])):
            sample = past[topic_name]['accel'][i]
            sum[0] = sum[0] + sample.accel.linear.x
            sum[1] = sum[1] + sample.accel.linear.y
            sum[2] = sum[2] + sample.accel.linear.z
        average_accel = sum/RUNNING_AVE_SAMPLES

        ave_accel.accel.linear.x = average_accel[0]
        ave_accel.accel.linear.y = average_accel[1]
        ave_accel.accel.linear.z = average_accel[2]
    else:
        past[topic_name]['accel'] = []

    past[topic_name]['ave_vel'] = current_vel

    return ave_accel

def compute_velocity(current_pose, topic_name):
    current_vel = TwistStamped()
    current_vel.header.stamp.secs = current_pose.header.stamp.secs
    current_vel.header.stamp.nsecs = current_pose.header.stamp.nsecs

    ave_vel = TwistStamped()
    ave_vel.header.stamp.secs = current_pose.header.stamp.secs
    ave_vel.header.stamp.nsecs = current_pose.header.stamp.nsecs
    
    # Only compute once we have saved a sample.
    if past[topic_name]:
        past_pose = past[topic_name]['pos']

        # Calculate times, measured in seconds
        current_t = float(current_pose.header.stamp.secs + 1e-9*current_pose.header.stamp.nsecs)
        past_t = float(past[topic_name]['pos'].header.stamp.secs + 1e-9*past[topic_name]['pos'].header.stamp.nsecs)
        delta_t = current_t - past_t

        current_vel.twist.linear.x = (current_pose.pose.position.x - past_pose.pose.position.x)/delta_t
        current_vel.twist.linear.y = (current_pose.pose.position.y - past_pose.pose.position.y)/delta_t
        current_vel.twist.linear.z = (current_pose.pose.position.z -past_pose.pose.position.z)/delta_t
        
        # Ensure delta_t is greater than sample rate, as timestamps can come in erroneously close together.
        if(delta_t >= 1.0/SAMPLING_RATE_POSE):
            # Fill up buffer of past instantaneous velocities, and average later.
            if(len(past[topic_name]['vel']) < RUNNING_AVE_SAMPLES):
                past[topic_name]['vel'].append(current_vel)
            else:
                del past[topic_name]['vel'][0]
                past[topic_name]['vel'].append(current_vel)

        #Compute average
        sum = np.array([0.0, 0.0, 0.0])
        for i in range(0, len(past[topic_name]['vel'])):
            sample = past[topic_name]['vel'][i]
            sum[0] = sum[0] + sample.twist.linear.x
            sum[1] = sum[1] + sample.twist.linear.y
            sum[2] = sum[2] + sample.twist.linear.z
        average = sum/RUNNING_AVE_SAMPLES

        ave_vel.twist.linear.x = average[0]
        ave_vel.twist.linear.y = average[1]
        ave_vel.twist.linear.z = average[2]
    else:
        past[topic_name]['pos'] = {}
        past[topic_name]['vel'] = []

    # Save the last pose, so we can get velocity.
    past[topic_name]['pos'] = current_pose

    return ave_vel 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
