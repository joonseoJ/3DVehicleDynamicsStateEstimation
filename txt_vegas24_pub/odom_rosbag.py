#!/usr/bin/env python2

import rosbag
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3
import numpy as np
import re

def extract_vector3(data_str):
    """
    Extract x, y, z values from Vector3 string
    """
    match = re.search(r'x=([-\d.]+),\s*y=([-\d.]+),\s*z=([-\d.]+)', data_str)
    if match:
        return Vector3(
            x=float(match.group(1)),
            y=float(match.group(2)),
            z=float(match.group(3))
        )
    return Vector3()

def extract_point(data_str):
    """
    Extract x, y, z values from Point string
    """
    match = re.search(r'x=([-\d.]+),\s*y=([-\d.]+),\s*z=([-\d.]+)', data_str)
    if match:
        return Point(
            x=float(match.group(1)),
            y=float(match.group(2)),
            z=float(match.group(3))
        )
    return Point()

def extract_quaternion(data_str):
    """
    Extract x, y, z, w values from Quaternion string
    """
    match = re.search(r'x=([-\d.]+),\s*y=([-\d.]+),\s*z=([-\d.]+),\s*w=([-\d.]+)', data_str)
    if match:
        return Quaternion(
            x=float(match.group(1)),
            y=float(match.group(2)),
            z=float(match.group(3)),
            w=float(match.group(4))
        )
    return Quaternion()

def extract_array(data_str):
    """
    Extract array values from numpy array string
    """
    match = re.search(r'array\(\[(.*?)\]\)', data_str)
    if match:
        values = [float(x.strip()) for x in match.group(1).split(',')]
        return np.array(values)
    return np.zeros(36)  # Odometry covariance is 6x6 matrix (36 elements)

def extract_pose(data_str):
    """
    Extract position and orientation from Pose string
    """
    pose = Pose()
    
    position_match = re.search(r'position=geometry_msgs\.msg\.Point\((.*?)\)', data_str)
    if position_match:
        pose.position = extract_point(position_match.group(1))
        
    orientation_match = re.search(r'orientation=geometry_msgs\.msg\.Quaternion\((.*?)\)', data_str)
    if orientation_match:
        pose.orientation = extract_quaternion(orientation_match.group(1))
        
    return pose

def extract_twist(data_str):
    """
    Extract linear and angular velocities from Twist string
    """
    twist = Twist()
    
    linear_match = re.search(r'linear=geometry_msgs\.msg\.Vector3\((.*?)\)', data_str)
    if linear_match:
        twist.linear = extract_vector3(linear_match.group(1))
        
    angular_match = re.search(r'angular=geometry_msgs\.msg\.Vector3\((.*?)\)', data_str)
    if angular_match:
        twist.angular = extract_vector3(angular_match.group(1))
        
    return twist

def parse_odom_line(line):
    """
    Parse a single line of Odometry data from text file
    Returns topic name, timestamp, and Odometry message
    """
    # Split line into topic, timestamp, and data
    match = re.match(r'(.*?)\s*\[(\d+)\]:\s*(.*)', line)
    if not match:
        return None, None, None
    
    topic, timestamp, data = match.groups()
    
    # Create Odometry message
    odom_msg = Odometry()
    
    # Extract child_frame_id
    frame_id_match = re.search(r"child_frame_id='([^']*)'", data)
    if frame_id_match:
        odom_msg.child_frame_id = frame_id_match.group(1)
    
    # Extract pose
    pose_match = re.search(r'pose=geometry_msgs\.msg\.PoseWithCovariance\(pose=(.*?), covariance=(.*?)\)', data)
    if pose_match:
        pose_data = pose_match.group(1)
        pose_cov_data = pose_match.group(2)
        
        # Set pose
        odom_msg.pose.pose = extract_pose(pose_data)
        # Set pose covariance
        odom_msg.pose.covariance = extract_array(pose_cov_data)
    
    # Extract twist
    twist_match = re.search(r'twist=geometry_msgs\.msg\.TwistWithCovariance\(twist=(.*?), covariance=(.*?)\)', data)
    if twist_match:
        twist_data = twist_match.group(1)
        twist_cov_data = twist_match.group(2)
        
        # Set twist
        odom_msg.twist.twist = extract_twist(twist_data)
        # Set twist covariance
        odom_msg.twist.covariance = extract_array(twist_cov_data)
    
    return topic, int(timestamp), odom_msg

def convert_txt_to_rosbag(input_txt, output_bag):
    """
    Convert text file containing Odometry data to rosbag
    
    Args:
        input_txt (str): Input text file path
        output_bag (str): Output rosbag file path
    """
    with open(input_txt, 'r') as f_in, rosbag.Bag(output_bag, 'a') as bag:
        for line in f_in:
            line = line.strip()
            if not line:
                continue
                
            topic, timestamp, odom_msg = parse_odom_line(line)
            if topic is None:
                continue
            
            # Convert timestamp to ROS time
            ros_time = rospy.Time(secs=timestamp // 1000000000,
                                nsecs=timestamp % 1000000000)
            odom_msg.header.stamp = ros_time
            
            # Write to bag file
            bag.write(topic, odom_msg, ros_time)

if __name__ == '__main__':
    # Input and output file paths
    input_file = 'odom_data.txt'  # Change this to your input file path
    output_file = 'vegas24_pub.bag'  # Change this to your desired output path
    
    # Initialize ROS node (needed for rospy.Time)
    rospy.init_node('txt_to_rosbag_converter', anonymous=True)
    
    try:
        convert_txt_to_rosbag(input_file, output_file)
        print("Successfully converted text file to rosbag!")
    except Exception as e:
        print("Error converting file: {}".format(e))