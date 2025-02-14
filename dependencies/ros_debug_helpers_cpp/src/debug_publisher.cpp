// Copyright 2023 Simon Sagmeister
#include "ros_debug_helpers_cpp/debug_publisher.hpp"

#include <ros/serialization.h>
namespace tam::ROS
{
void DebugPublisher::publish_signal_names()
{
  _signal_name_publisher.publish(_signal_names_msg);
  _signal_names_last_published = ros::Time::now();
}

std::string DebugPublisher::get_validated_channel_name(const std::string & debug_channel_name)
{
  auto validated_channel_name =
    tam::types::common::TUMDebugContainer::validate_signal_name(debug_channel_name);
  if (validated_channel_name.empty()) {
    throw std::invalid_argument(
      std::string("Given channel name is not valid: ") + debug_channel_name);
  }
  validated_channel_name.insert(0, 1, '/');
  return validated_channel_name;
}

void DebugPublisher::connect_to_node(ros::NodeHandle node_handle)
{
  // if (_node_handle) {
  //   RCLCPP_ERROR(
  //     _node_handle->get_logger(), "ERROR: The debug publisher is already connnected to a node");
  //   return;
  // }
  _node_handle = node_handle;
  
  _signal_name_publisher = node_handle.advertise<msgs::TUMDebugSignalNames>(
    std::string("/debug") + ros::this_node::getName() + _channel_name + "/signal_names",
    qos_settings::DEBUG_SIGNAL_NAMES);

  _value_publisher = node_handle.advertise<msgs::TUMDebugValues>(
    std::string("/debug") + ros::this_node::getName() + _channel_name + "/values",
    qos_settings::DEBUG_VALUES);
}

void DebugPublisher::publish_debug_outputs(tam::types::common::TUMDebugContainer * debug_container)
{
  // if (!_node_handle) {
  //   RCLCPP_ERROR(
  //     rclcpp::get_logger("DebugPublisherFallbackLogger"),
  //     "ERROR: The debug publisher is not connnected to a node. Call the "
  //     "DebugPublisher::connect_to_node function beforehand");
  //   return;
  // }
  if (!_initialized) {
    _initialized = true;
    _config_hash = debug_container->get_config_hash();
    // Create and serialize the debug message once to not have to serialize the
    // vector of strings every time since we saw quite some performance
    // hit from the serialization
    _signal_names_msg.names = debug_container->get_signal_names();
    publish_signal_names();
  }

  if (_config_hash != debug_container->get_config_hash()) {
    // Do nothing if the config hash differs from the first published container
    ROS_ERROR("ERROR publishing debug outputs. The config hash of the container changed");
    return;
  }

  // Update the messages
  _values_msg.values = debug_container->get_values();

  // Do not publish values any more
  if (
    (ros::Time::now() - _signal_names_last_published).toSec() >=
    1.0 / _signal_name_pub_frequency) {
    publish_signal_names();
  }

  // Publish the value message
  _value_publisher.publish(_values_msg);
}
}  // namespace tam::ROS
