// Copyright 2023 Philipp Pitschi
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"

#include "tum_type_conversions_ros_cpp/orientation.hpp"

namespace tam::type_conversions
{
uint64_t header_stamp_type_from_msg(const ros::Time& msg)
{
  return msg.sec * 1e9 + msg.nsec;
}

ros::Time header_stamp_msg_from_type(const uint64_t& header_stamp)
{
  ros::Time msg;
  msg.sec = floor(header_stamp / 1e9);
  msg.nsec = header_stamp % (int)1e9;
  return msg;
}

unsigned char diagnostic_level_from_type(const tam::types::ErrorLvl& lvl)
{
  switch (lvl) {
    case tam::types::ErrorLvl::OK:
      return diagnostic_msgs::DiagnosticStatus::OK;
    case tam::types::ErrorLvl::STALE:
      return diagnostic_msgs::DiagnosticStatus::STALE;
    case tam::types::ErrorLvl::ERROR:
      return diagnostic_msgs::DiagnosticStatus::ERROR;
    case tam::types::ErrorLvl::WARN:
      return diagnostic_msgs::DiagnosticStatus::WARN;
    default:
      ROS_ERROR("Unknown Error Level");
      return diagnostic_msgs::DiagnosticStatus::ERROR;
  }
}

tam::types::ErrorLvl error_type_from_diagnostic_level(unsigned char lvl)
{
  switch (lvl) {
    case diagnostic_msgs::DiagnosticStatus::OK:
      return tam::types::ErrorLvl::OK;
    case diagnostic_msgs::DiagnosticStatus::WARN:
      return tam::types::ErrorLvl::WARN;
    case diagnostic_msgs::DiagnosticStatus::ERROR:
      return tam::types::ErrorLvl::ERROR;
    case diagnostic_msgs::DiagnosticStatus::STALE:
      return tam::types::ErrorLvl::STALE;
    default:
      ROS_ERROR("Unknown Error Level");
      return tam::types::ErrorLvl::ERROR;
  }
}

tam::types::control::Odometry odometry_type_from_imu_msg(const sensor_msgs::Imu& msg)
{
  tam::types::control::Odometry odometry;
  odometry.angular_velocity_radps.x = msg.angular_velocity.x;
  odometry.angular_velocity_radps.y = msg.angular_velocity.y;
  odometry.angular_velocity_radps.z = msg.angular_velocity.z;
  return odometry;
}

tam::types::control::AccelerationwithCovariances acceleration_with_covariances_type_from_imu_msg(
  const sensor_msgs::Imu& msg)
{
  tam::types::control::AccelerationwithCovariances acceleration;
  acceleration.acceleration_mps2.x = msg.linear_acceleration.x;
  acceleration.acceleration_mps2.y = msg.linear_acceleration.y;
  acceleration.acceleration_mps2.z = msg.linear_acceleration.z;
  return acceleration;
}

tam::types::control::Odometry odometry_type_from_msg(const nav_msgs::Odometry& msg)
{
  tam::types::control::Odometry odometry;
  odometry.position_m.x = msg.pose.pose.position.x;
  odometry.position_m.y = msg.pose.pose.position.y;
  odometry.position_m.z = msg.pose.pose.position.z;

  tf::Quaternion q;
  tf::quaternionMsgToTF(msg.pose.pose.orientation, q);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  odometry.orientation_rad = tam::types::common::Vector3D<double>{roll, pitch, yaw};

  // odometry.pose_covariance = msg.pose.covariance;
  std::copy(msg.pose.covariance.begin(), msg.pose.covariance.end(), odometry.pose_covariance.begin());
  odometry.angular_velocity_radps.x = msg.twist.twist.angular.x;
  odometry.angular_velocity_radps.y = msg.twist.twist.angular.y;
  odometry.angular_velocity_radps.z = msg.twist.twist.angular.z;
  odometry.velocity_mps.x = msg.twist.twist.linear.x;
  odometry.velocity_mps.y = msg.twist.twist.linear.y;
  odometry.velocity_mps.z = msg.twist.twist.linear.z;
  // odometry.velocity_covariance = msg.twist.covariance;
  std::copy(msg.twist.covariance.begin(), msg.twist.covariance.end(), odometry.velocity_covariance.begin());
  return odometry;
}

nav_msgs::Odometry odometry_msg_from_type(const tam::types::control::Odometry& odometry)
{
  nav_msgs::Odometry odometry_msg;
  odometry_msg.pose.pose.position.x = odometry.position_m.x;
  odometry_msg.pose.pose.position.y = odometry.position_m.y;
  odometry_msg.pose.pose.position.z = odometry.position_m.z;

  tf::Quaternion q;
  q.setRPY(odometry.orientation_rad.x, odometry.orientation_rad.y, odometry.orientation_rad.z);
  tf::quaternionTFToMsg(q, odometry_msg.pose.pose.orientation);

  // odometry_msg.pose.covariance = odometry.pose_covariance;
  std::copy(odometry.pose_covariance.begin(), odometry.pose_covariance.end(), odometry_msg.pose.covariance.begin());
  odometry_msg.twist.twist.linear.x = odometry.velocity_mps.x;
  odometry_msg.twist.twist.linear.y = odometry.velocity_mps.y;
  odometry_msg.twist.twist.linear.z = odometry.velocity_mps.z;
  odometry_msg.twist.twist.angular.x = odometry.angular_velocity_radps.x;
  odometry_msg.twist.twist.angular.y = odometry.angular_velocity_radps.y;
  odometry_msg.twist.twist.angular.z = odometry.angular_velocity_radps.z;
  // odometry_msg.twist.covariance = odometry.velocity_covariance;
  std::copy(odometry.velocity_covariance.begin(), odometry.velocity_covariance.end(), odometry_msg.twist.covariance.begin());
  return odometry_msg;
}
}  // namespace tam::type_conversions

namespace tam::type::conversions::cpp
{
tam::types::control::Odometry Odometry_type_from_msg(const nav_msgs::Odometry::ConstPtr& msg)
{
  return tam::type_conversions::odometry_type_from_msg(*msg);
}
}  // namespace tam::type::conversions::cpp
