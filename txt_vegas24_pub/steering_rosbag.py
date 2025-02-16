#!/usr/bin/env python2

import rosbag
import rospy
import re
from msgs.msg import SteeringReport

def parse_steering_line(line):
    """
    Parse a single line of SteeringReport data from text file
    Returns topic name, timestamp, and message
    """
    # Split line into topic, timestamp, and data
    match = re.match(r'(.*?)\s*\[(\d+)\]:\s*(.*)', line)
    if not match:
        return None, None, None
    
    topic, timestamp, data = match.groups()
    
    # Create message
    msg = SteeringReport()
    
    # Extract stamp
    stamp_match = re.search(r'stamp=.*?sec=(\d+),\s*nanosec=(\d+)', data)
    # if stamp_match:
    #     msg.stamp.sec = int(stamp_match.group(1))
    #     msg.stamp.nsec = int(stamp_match.group(2))
    
    # Extract steering tire angle
    angle_match = re.search(r'steering_tire_angle=([-\d.]+)', data)
    if angle_match:
        msg.steering_tire_angle = float(angle_match.group(1))
    
    return topic, int(timestamp), msg

def convert_txt_to_rosbag(input_txt, output_bag):
    """
    Convert text file containing SteeringReport data to rosbag
    
    Args:
        input_txt (str): Input text file path
        output_bag (str): Output rosbag file path
    """
    with open(input_txt, 'r') as f_in, rosbag.Bag(output_bag, 'a') as bag:
        for line in f_in:
            line = line.strip()
            if not line:
                continue
                
            topic, timestamp, msg = parse_steering_line(line)
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
    input_file = 'steering_data.txt'  # Change this to your input file path
    output_file = 'vegas24_pub.bag'  # Change this to your desired output path
    
    # Initialize ROS node (needed for rospy.Time)
    rospy.init_node('txt_to_rosbag_converter', anonymous=True)
    
    try:
        convert_txt_to_rosbag(input_file, output_file)
        print("Successfully converted text file to rosbag!")
    except Exception as e:
        print("Error converting file: {}".format(e))