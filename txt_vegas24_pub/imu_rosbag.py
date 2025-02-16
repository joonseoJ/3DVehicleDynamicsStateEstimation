#!/usr/bin/env python2

import rosbag
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
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
    return np.zeros(9)

def parse_imu_line(line):
    """
    Parse a single line of IMU data from text file
    Returns topic name, timestamp, and IMU message
    """
    # Split line into topic, timestamp, and data
    match = re.match(r'(.*?)\s*\[(\d+)\]:\s*(.*)', line)
    if not match:
        return None, None, None
    
    topic, timestamp, data = match.groups()
    
    # Create IMU message
    imu_msg = Imu()
    
    # Extract orientation
    orientation_match = re.search(r'orientation=geometry_msgs\.msg\.Quaternion\((.*?)\)', data)
    if orientation_match:
        imu_msg.orientation = extract_quaternion(orientation_match.group(1))
    
    # Extract orientation covariance
    orientation_cov_match = re.search(r'orientation_covariance=(.*?\])', data)
    if orientation_cov_match:
        imu_msg.orientation_covariance = extract_array(orientation_cov_match.group(1))
    
    # Extract angular velocity
    angular_vel_match = re.search(r'angular_velocity=geometry_msgs\.msg\.Vector3\((.*?)\)', data)
    if angular_vel_match:
        imu_msg.angular_velocity = extract_vector3(angular_vel_match.group(1))
    
    # Extract angular velocity covariance
    angular_vel_cov_match = re.search(r'angular_velocity_covariance=(.*?\])', data)
    if angular_vel_cov_match:
        imu_msg.angular_velocity_covariance = extract_array(angular_vel_cov_match.group(1))
    
    # Extract linear acceleration
    linear_acc_match = re.search(r'linear_acceleration=geometry_msgs\.msg\.Vector3\((.*?)\)', data)
    if linear_acc_match:
        imu_msg.linear_acceleration = extract_vector3(linear_acc_match.group(1))
    
    # Extract linear acceleration covariance
    linear_acc_cov_match = re.search(r'linear_acceleration_covariance=(.*?\])', data)
    if linear_acc_cov_match:
        imu_msg.linear_acceleration_covariance = extract_array(linear_acc_cov_match.group(1))
    
    return topic, int(timestamp), imu_msg

def convert_txt_to_rosbag(input_txt, output_bag):
    """
    Convert text file containing IMU data to rosbag
    
    Args:
        input_txt (str): Input text file path
        output_bag (str): Output rosbag file path
    """
    with open(input_txt, 'r') as f_in, rosbag.Bag(output_bag, 'a') as bag:
        for line in f_in:
            line = line.strip()
            if not line:
                continue
                
            topic, timestamp, imu_msg = parse_imu_line(line)
            if topic is None:
                continue
            
            # Convert timestamp to ROS time
            ros_time = rospy.Time(secs=timestamp // 1000000000,
                                nsecs=timestamp % 1000000000)
            imu_msg.header.stamp = ros_time
            
            # Write to bag file
            bag.write(topic, imu_msg, ros_time)

if __name__ == '__main__':
    # Input and output file paths
    input_file = 'imu_data.txt'  # Change this to your input file path
    output_file = 'vegas24_pub.bag'  # Change this to your desired output path
    
    # Initialize ROS node (needed for rospy.Time)
    rospy.init_node('txt_to_rosbag_converter', anonymous=True)
    
    try:
        convert_txt_to_rosbag(input_file, output_file)
        print("Successfully converted text file to rosbag!")
    except Exception as e:
        print("Error converting file: {}".format(e))