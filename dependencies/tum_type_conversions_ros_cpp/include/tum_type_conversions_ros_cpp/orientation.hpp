// Copyright 2023 Simon Sagmeister
#pragma once

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

#include <geometry_msgs/Quaternion.h>
#include "tum_types_cpp/common.hpp"

namespace tam::type_conversions
{
tam::types::common::EulerYPR quaternion_msg_to_euler_type(
  const geometry_msgs::Quaternion & msg);

geometry_msgs::Quaternion euler_type_to_quaternion_msg(
  const tam::types::common::EulerYPR & euler_angles);
}  // namespace tam::type_conversions

// Backwards compatibility
// ========================================
namespace tam::types::conversion
{
using tam::type_conversions::euler_type_to_quaternion_msg;
using tam::type_conversions::quaternion_msg_to_euler_type;
}  // namespace tam::types::conversion
