#!/usr/bin/env python2

import rosbag
import rospy
import re
import genpy
from msgs.msg import TUMFloat64PerWheelStamped, TUMFloat64PerWheel

def parse_wheel_data(data_str):
    """
    Parse TUMFloat64PerWheel data from string
    """
    wheel_data = TUMFloat64PerWheel()
    
    # Extract values for each wheel
    fl_match = re.search(r'front_left=([-\d.]+)', data_str)
    fr_match = re.search(r'front_right=([-\d.]+)', data_str)
    rl_match = re.search(r'rear_left=([-\d.]+)', data_str)
    rr_match = re.search(r'rear_right=([-\d.]+)', data_str)
    
    if fl_match:
        wheel_data.front_left = float(fl_match.group(1))
    if fr_match:
        wheel_data.front_right = float(fr_match.group(1))
    if rl_match:
        wheel_data.rear_left = float(rl_match.group(1))
    if rr_match:
        wheel_data.rear_right = float(rr_match.group(1))
    
    return wheel_data

def parse_wheel_line(line):
    """
    Parse a single line of TUMFloat64PerWheelStamped data from text file
    Returns topic name, timestamp, and message
    """
    # Split line into topic, timestamp, and data
    match = re.match(r'(.*?)\s*\[(\d+)\]:\s*(.*)', line)
    if not match:
        return None, None, None
    
    topic, timestamp, data = match.groups()
    
    # Create message
    msg = TUMFloat64PerWheelStamped()
    
    # Extract stamp
    stamp_match = re.search(r'stamp=.*?sec=(\d+),\s*nanosec=(\d+)', data)
    # if stamp_match:
    #     msg.stamp.sec = int(stamp_match.group(1))
    #     msg.stamp.nsec = int(stamp_match.group(2))
    
    # Extract wheel data
    data_match = re.search(r'data=msgs\.msg\.TUMFloat64PerWheel\((.*?)\)', data)
    if data_match:
        msg.data = parse_wheel_data(data_match.group(1))
    
    return topic, int(timestamp), msg

def convert_txt_to_rosbag(input_txt, output_bag):
    """
    Convert text file containing TUMFloat64PerWheelStamped data to rosbag
    
    Args:
        input_txt (str): Input text file path
        output_bag (str): Output rosbag file path
    """
    with open(input_txt, 'r') as f_in, rosbag.Bag(output_bag, 'a') as bag:
        for line in f_in:
            line = line.strip()
            if not line:
                continue
                
            topic, timestamp, msg = parse_wheel_line(line)
            if topic is None:
                continue
            
            # Convert timestamp to ROS time
            ros_time = rospy.Time(secs=timestamp // 1000000000,
                                nsecs=timestamp % 1000000000)
            msg.stamp = ros_time
            
            # Write to bag file
            bag.write(topic, msg, ros_time)

if __name__ == '__main__':
    # Input and output file paths
    input_file = 'TUMFloat_data.txt'  # Change this to your input file path
    output_file = 'vegas24_pub.bag'  # Change this to your desired output path
    
    # Initialize ROS node (needed for rospy.Time)
    rospy.init_node('txt_to_rosbag_converter', anonymous=True)
    
    try:
        convert_txt_to_rosbag(input_file, output_file)
        print("Successfully converted text file to rosbag!")
    except Exception as e:
        print("Error converting file: {}".format(e))