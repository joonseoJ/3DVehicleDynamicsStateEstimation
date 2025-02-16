#!/usr/bin/env python2

import rosbag
import rospy
from std_msgs.msg import Float32
import re

def parse_float32_line(line):
    """
    Parse a single line of Float32 data from text file
    Returns topic name, timestamp, and Float32 message
    """
    # Split line into topic, timestamp, and data
    match = re.match(r'(.*?)\s*\[(\d+)\]:\s*(.*)', line)
    if not match:
        return None, None, None
    
    topic, timestamp, data = match.groups()
    
    # Create Float32 message
    float_msg = Float32()
    
    # Extract float value
    value_match = re.search(r'data=([-\d.]+)', data)
    if value_match:
        float_msg.data = float(value_match.group(1))
    
    return topic, int(timestamp), float_msg

def convert_txt_to_rosbag(input_txt, output_bag):
    """
    Convert text file containing Float32 data to rosbag
    
    Args:
        input_txt (str): Input text file path
        output_bag (str): Output rosbag file path
    """
    with open(input_txt, 'r') as f_in, rosbag.Bag(output_bag, 'a') as bag:
        for line in f_in:
            line = line.strip()
            if not line:
                continue
                
            topic, timestamp, float_msg = parse_float32_line(line)
            if topic is None:
                continue
            
            # Convert timestamp to ROS time
            ros_time = rospy.Time(secs=timestamp // 1000000000,
                                nsecs=timestamp % 1000000000)
            
            # Write to bag file
            bag.write(topic, float_msg, ros_time)

if __name__ == '__main__':
    # Input and output file paths
    input_file = 'float_data.txt'  # Change this to your input file path
    output_file = 'vegas24_pub.bag'  # Change this to your desired output path
    
    # Initialize ROS node (needed for rospy.Time)
    rospy.init_node('txt_to_rosbag_converter', anonymous=True)
    
    try:
        convert_txt_to_rosbag(input_file, output_file)
        print("Successfully converted text file to rosbag!")
    except Exception as e:
        print("Error converting file: {}".format(e))