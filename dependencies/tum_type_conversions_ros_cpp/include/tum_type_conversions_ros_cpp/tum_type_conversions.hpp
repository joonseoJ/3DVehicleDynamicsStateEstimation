// Copyright 2023 Philipp Pitschi
#pragma once
#include <math.h>

// ROS
#include <tf/transform_datatypes.h>
#include <ros/ros.h>

// messages
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <msgs/TUMFloat64PerWheel.h>

// types
#include "tum_helpers_cpp/type_conversion.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

namespace tam::type_conversions
{
// Basic types
uint64_t header_stamp_type_from_msg(const ros::Time& msg);
ros::Time header_stamp_msg_from_type(const uint64_t& header);
tam::types::common::Vector3D<double> vector_3d_type_from_msg(
  const geometry_msgs::Vector3& msg);
tam::types::common::Vector3D<double> vector_3d_type_from_msg(const geometry_msgs::Point& msg);
geometry_msgs::Vector3 vector_3d_msg_from_type(
  const tam::types::common::Vector3D<double>& vector_3d);
geometry_msgs::Point point_msg_from_type(
  const tam::types::common::Vector3D<double>& vector_3d);
tam::types::ErrorLvl error_type_from_diagnostic_level(unsigned char lvl);
unsigned char diagnostic_level_from_type(const tam::types::ErrorLvl& lvl);

// State Estimation
tam::types::control::AccelerationwithCovariances accel_with_covariance_stamped_type_from_msg(
  const geometry_msgs::AccelWithCovarianceStamped& msg);
geometry_msgs::AccelWithCovarianceStamped accel_with_covariance_stamped_msg_from_type(
  const tam::types::control::AccelerationwithCovariances& acceleration);
tam::types::control::AccelerationwithCovariances acceleration_with_covariances_type_from_imu_msg(
  const sensor_msgs::Imu& msg);
tam::types::control::Odometry odometry_type_from_msg(const nav_msgs::Odometry& odometry);
nav_msgs::Odometry odometry_msg_from_type(const tam::types::control::Odometry& odometry);
tam::types::control::Odometry odometry_type_from_imu_msg(const sensor_msgs::Imu& msg);

tam::types::control::Odometry odometry_type_from_imu_msg(
  const sensor_msgs::Imu::ConstPtr& msg);
tam::types::control::AccelerationwithCovariances acceleration_with_covariances_type_from_imu_msg(
  const sensor_msgs::Imu::ConstPtr& msg);
}  // namespace tam::type_conversions

namespace tam::type::conversions::cpp
{
tam::types::control::Odometry Odometry_type_from_msg(const nav_msgs::Odometry::ConstPtr& msg);
}  // namespace tam::type::conversions::cpp
