// Copyright 2023 TUMWFTM
#include "tum_type_conversions_ros_cpp/orientation.hpp"

namespace tam::type_conversions
{
tam::types::common::EulerYPR quaternion_msg_to_euler_type(
  const geometry_msgs::Quaternion & msg)
{
  tf::Quaternion q(msg.x, msg.y, msg.z, msg.w);
  tf::Matrix3x3 m(q.normalized());
  double yaw, pitch, roll;
  m.getEulerYPR(yaw, pitch, roll);
  return tam::types::common::EulerYPR(yaw, pitch, roll);
}

geometry_msgs::Quaternion euler_type_to_quaternion_msg(
  const tam::types::common::EulerYPR & euler_angles)
{
  tf::Matrix3x3 m;
  tf::Quaternion q;
  m.setEulerYPR(euler_angles.yaw, euler_angles.pitch, euler_angles.roll);
  q.setRPY(euler_angles.roll, euler_angles.pitch, euler_angles.yaw);  // ROS1의 경우 setRPY 사용
  q.normalize();
  
  geometry_msgs::Quaternion q_msg;
  q_msg.x = q.x();
  q_msg.y = q.y();
  q_msg.z = q.z();
  q_msg.w = q.w();
  
  return q_msg;
}
}  // namespace tam::type_conversions
