#!/usr/bin/env python2

import rosbag
import rospy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import re

def parse_diagnostic_line(line):
    """
    Parse a single line of DiagnosticStatus data from text file
    Returns topic name, timestamp, and DiagnosticStatus message
    """
    # Split line into topic, timestamp, and data
    match = re.match(r'(.*?)\s*\[(\d+)\]:\s*(.*)', line)
    if not match:
        return None, None, None
    
    topic, timestamp, data = match.groups()
    
    # Create DiagnosticStatus message
    status_msg = DiagnosticStatus()
    
    # Extract level (handle byte string format)
    level_match = re.search(r"level=b'\\x([0-9a-fA-F]{2})'", data)
    if level_match:
        status_msg.level = int(level_match.group(1), 16)
    
    # Extract name
    name_match = re.search(r"name='([^']*)'", data)
    if name_match:
        status_msg.name = name_match.group(1)
    
    # Extract message
    message_match = re.search(r"message='([^']*)'", data)
    if message_match:
        status_msg.message = message_match.group(1)
    
    # Extract hardware_id
    hardware_id_match = re.search(r"hardware_id='([^']*)'", data)
    if hardware_id_match:
        status_msg.hardware_id = hardware_id_match.group(1)
    
    # Extract values list (if not empty)
    values_match = re.search(r"values=\[(.*?)\]", data)
    if values_match and values_match.group(1):
        # If there are values, parse them
        values_str = values_match.group(1)
        # Split by comma and parse each KeyValue pair
        value_pairs = values_str.split(',')
        for pair in value_pairs:
            key_value = KeyValue()
            key_match = re.search(r"key='([^']*)'", pair)
            value_match = re.search(r"value='([^']*)'", pair)
            if key_match:
                key_value.key = key_match.group(1)
            if value_match:
                key_value.value = value_match.group(1)
            status_msg.values.append(key_value)
    
    return topic, int(timestamp), status_msg

def convert_txt_to_rosbag(input_txt, output_bag):
    """
    Convert text file containing DiagnosticStatus data to rosbag
    
    Args:
        input_txt (str): Input text file path
        output_bag (str): Output rosbag file path
    """
    with open(input_txt, 'r') as f_in, rosbag.Bag(output_bag, 'a') as bag:
        for line in f_in:
            line = line.strip()
            if not line:
                continue
                
            topic, timestamp, status_msg = parse_diagnostic_line(line)
            if topic is None:
                continue
            
            # Convert timestamp to ROS time
            ros_time = rospy.Time(secs=timestamp // 1000000000,
                                nsecs=timestamp % 1000000000)
            
            # Write to bag file
            bag.write(topic, status_msg, ros_time)

if __name__ == '__main__':
    # Input and output file paths
    input_file = 'diag_data.txt'  # Change this to your input file path
    output_file = 'vegas24_pub.bag'  # Change this to your desired output path
    
    # Initialize ROS node (needed for rospy.Time)
    rospy.init_node('txt_to_rosbag_converter', anonymous=True)
    
    try:
        convert_txt_to_rosbag(input_file, output_file)
        print("Successfully converted text file to rosbag!")
    except Exception as e:
        print("Error converting file: {}".format(e))