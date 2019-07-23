#!/usr/bin/env python
# -*- coding: utf-8 -*-
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped, Vector3Stamped, Vector3
import numpy as np
import tf

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
            pubs[topic_name]['euler'] = rospy.Publisher(topic_name.split("/pose")[0] + '/euler', Vector3Stamped, queue_size=10 )

            past[topic_name] = {}

            # call the callback function each time we recieve a PoseStamped message. 
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
    euler = compute_euler_angles(data, topic_name)
    pubs[topic_name]['euler'].publish(euler)

def compute_euler_angles(current_pose,topic_name): 
    euler = Vector3Stamped()
    euler.header.stamp.secs = current_pose.header.stamp.secs
    euler.header.stamp.nsecs = current_pose.header.stamp.nsecs
    euler.vector = euler_from_orientation(current_pose.pose.orientation)

    return euler

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

        current_accel.accel.angular.x = (current_vel.twist.angular.x - past_vel.twist.angular.x)/delta_t
        current_accel.accel.angular.y = (current_vel.twist.angular.y - past_vel.twist.angular.y)/delta_t
        current_accel.accel.angular.z = (current_vel.twist.angular.z - past_vel.twist.angular.z)/delta_t

        # Update acceleration buffer
        if(len(past[topic_name]['accel']) < RUNNING_AVE_SAMPLES):
            past[topic_name]['accel'].append(current_accel)
        else:
            del past[topic_name]['accel'][0]
            past[topic_name]['accel'].append(current_accel)

        # Compute average
        sum = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        for i in range(0, len(past[topic_name]['accel'])):
            sample = past[topic_name]['accel'][i]
            sum[0] = sum[0] + sample.accel.linear.x
            sum[1] = sum[1] + sample.accel.linear.y
            sum[2] = sum[2] + sample.accel.linear.z
            sum[3] = sum[3] + sample.accel.angular.x
            sum[4] = sum[4] + sample.accel.angular.y
            sum[5] = sum[5] + sample.accel.angular.z
        average_accel = sum/RUNNING_AVE_SAMPLES

        ave_accel.accel.linear.x = average_accel[0]
        ave_accel.accel.linear.y = average_accel[1]
        ave_accel.accel.linear.z = average_accel[2]
        ave_accel.accel.angular.x = average_accel[3]
        ave_accel.accel.angular.y = average_accel[4]
        ave_accel.accel.angular.z = average_accel[5]
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

        # Convert to euler from quaternion
        current_euler = euler_from_orientation(current_pose.pose.orientation)
        past_euler = euler_from_orientation(past_pose.pose.orientation)

        # Angular velocity
        current_vel.twist.angular.x = (current_euler.x - past_euler.x)/delta_t
        current_vel.twist.angular.y = (current_euler.y - past_euler.y)/delta_t
        current_vel.twist.angular.z = (current_euler.z - past_euler.z)/delta_t

        #linear velocity
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
        sum = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        for i in range(0, len(past[topic_name]['vel'])):
            sample = past[topic_name]['vel'][i]
            sum[0] = sum[0] + sample.twist.linear.x
            sum[1] = sum[1] + sample.twist.linear.y
            sum[2] = sum[2] + sample.twist.linear.z
            sum[3] = sum[3] + sample.twist.angular.x
            sum[4] = sum[4] + sample.twist.angular.y
            sum[5] = sum[5] + sample.twist.angular.z
        average = sum/RUNNING_AVE_SAMPLES

        ave_vel.twist.linear.x = average[0]
        ave_vel.twist.linear.y = average[1]
        ave_vel.twist.linear.z = average[2]
        ave_vel.twist.angular.x = average[3]
        ave_vel.twist.angular.y = average[4]
        ave_vel.twist.angular.z = average[5]
    else:
        past[topic_name]['pos'] = {}
        past[topic_name]['vel'] = []

    # Save the last pose, so we can get velocity.
    past[topic_name]['pos'] = current_pose

    return ave_vel 

# Convert to Euler angles from PoseStamped's default quaternions.
def euler_from_orientation(orientation):
    euler = Vector3()

    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    )

    euler_arr = tf.transformations.euler_from_quaternion(
        quaternion, axes='sxyz')

    euler.x = euler_arr[0]
    euler.y = euler_arr[1]
    euler.z = euler_arr[2]

    return euler
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
