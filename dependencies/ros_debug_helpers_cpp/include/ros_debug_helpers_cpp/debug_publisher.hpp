// Copyright 2023 Simon Sagmeister
#pragma once

#include <memory>
#include <ros/ros.h>
#include <string>
#include <msgs/TUMDebugSignalNames.h>
#include <msgs/TUMDebugValues.h>
#include "tum_types_cpp/common.hpp"
namespace tam::ROS::qos_settings
{
  static const int DEBUG_SIGNAL_NAMES = 1;
  static const int DEBUG_VALUES = 1;
}  // namespace qos_settings

namespace tam::ROS
{
class DebugPublisher
{
private:
  static constexpr double _signal_name_pub_frequency = 0.5;
  bool _initialized = false;
  std::string _channel_name = "";
  std::size_t _config_hash;
  ros::Time _signal_names_last_published;
  ros::NodeHandle _node_handle;
  ros::Publisher _signal_name_publisher;
  ros::Publisher _value_publisher;

  // rcl_serialized_message_t _rcl_serialized_signal_name_message;

  // We have to store this as well, otherwise the destrctor of this object cleans up the above
  // object
  // rclcpp::SerializedMessage _serialized_signal_name_message;

  // ROS1 serialized message is stored as a vector of uint8_t
  msgs::TUMDebugSignalNames _signal_names_msg;

  msgs::TUMDebugValues _values_msg;

  // Helper functions
  static std::string get_validated_channel_name(const std::string & debug_channel_name);
  void publish_signal_names();
  void connect_to_node(ros::NodeHandle node_handle);

public:
  /// @brief Constructs a debug publisher for a non default channel. This is especially useful if a
  /// node wants to publish multiple debug messages asynchronously (e.g. on subscription) of another
  /// message. The topics published by this publisher will be:
  /// - /debug/${fully_qualified_node_name}/${debug_channel_name}/signal_names
  /// - /debug/${fully_qualified_node_name}/${debug_channel_name}/values
  /// @param node_handle
  explicit DebugPublisher(ros::NodeHandle node_handle) { connect_to_node(node_handle); }
  /// @brief Constructs a debug publisher for a non default channel. This is especially useful if a
  /// node wants to publish multiple debug messages asynchronously (e.g. on subscription) of another
  /// message. The topics published by this publisher will be:
  /// - /debug/${fully_qualified_node_name}/${debug_channel_name}/signal_names
  /// - /debug/${fully_qualified_node_name}/${debug_channel_name}/values
  /// @param node_handle
  /// @param debug_channel_name
  explicit DebugPublisher(ros::NodeHandle node_handle, const std::string & debug_channel_name)
  : _channel_name(get_validated_channel_name(debug_channel_name))
  {
    connect_to_node(node_handle);
  }
  void publish_debug_outputs(tam::types::common::TUMDebugContainer * debug_container);
  // Stuff below this line is just here for backwards compatibility
  // ======================================================================

  /// DEPRECATED: DO NOT USE ANY MORE
  void connect_to_node(std::shared_ptr<ros::NodeHandle> node_handle)
  {
    ROS_WARN(
      "DEPRECATION WARNING: DebugPubliser | Do not explicitly use the connect_to_node function any "
      "more. This syntax will be removed. The new version is to directly pass the node handle in "
      "the constructor of the debug publisher");
    connect_to_node(*node_handle);
  };
  /// DEPRECATED: DO NOT USE ANY MORE
  DebugPublisher() = default;
  /// DEPRECATED: DO NOT USE ANY MORE
  explicit DebugPublisher(const std::string & debug_channel_name)
  : _channel_name(get_validated_channel_name(debug_channel_name))
  {
  }
};
}  // namespace tam::ROS
