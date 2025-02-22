// Copyright 2024 Sven Goblirsch
#pragma once
#include "ssa_estimation_node_cpp/ssa_estimation_node.hpp"

using namespace std::chrono_literals;
// using std::placeholders::_1;
// using std::placeholders::_2;

template <class TConfig> SSAEstimationNode<TConfig>::SSAEstimationNode(
  std::unique_ptr<tam::core::ssa::SSAEstimationBase> && ssa_estimation,
  ros::NodeHandle nh)
:ssa_estimation_(std::move(ssa_estimation))
{
  nh_ = nh;
  debug_channel_publishers_ = {
    tam::ROS::DebugPublisher(nh_, "state_machine"),
    tam::ROS::DebugPublisher(nh_, "kalman_filter"),
  };

  // initialize the topic watchdog
  topic_watchdog_ = std::make_unique<tam::core::TopicWatchdog>(nh);

  // model step callback
  model_update_timer_ =
    nh.createTimer(ros::Duration(0.01), &SSAEstimationNode::function_queue_callback, this);

  // subscriber
  topic_watchdog_->add_subscription<nav_msgs::Odometry>(
    "/core/state/odometry", 1, 
    std::bind(&SSAEstimationNode::odometry_callback, this, std::placeholders::_1),
    std::bind(&SSAEstimationNode::odometry_timeout_callback, this, std::placeholders::_1, std::placeholders::_2), 500ms);

  topic_watchdog_->add_subscription<sensor_msgs::Imu>(
    "/vehicle/sensor/imu1", 1, 
    std::bind(&SSAEstimationNode::imu_1_callback, this, std::placeholders::_1),
    std::bind(&SSAEstimationNode::imu_1_timeout_callback, this, std::placeholders::_1, std::placeholders::_2), 500ms);

  topic_watchdog_->add_subscription<sensor_msgs::Imu>(
    "/vehicle/sensor/imu2", 1, 
    std::bind(&SSAEstimationNode::imu_2_callback, this, std::placeholders::_1),
    std::bind(&SSAEstimationNode::imu_2_timeout_callback, this, std::placeholders::_1, std::placeholders::_2), 500ms);

  topic_watchdog_->add_subscription<sensor_msgs::Imu>(
    "/vehicle/sensor/imu3", 1, 
    std::bind(&SSAEstimationNode::imu_3_callback, this, std::placeholders::_1),
    std::bind(&SSAEstimationNode::imu_3_timeout_callback, this, std::placeholders::_1, std::placeholders::_2), 500ms);

  topic_watchdog_->add_subscription<sensor_msgs::Imu>(
    "/vehicle/sensor/backup_imu", 1, 
    std::bind(&SSAEstimationNode::imu_backup_callback, this, std::placeholders::_1),
    std::bind(&SSAEstimationNode::imu_backup_timeout_callback, this, std::placeholders::_1, std::placeholders::_2), 500ms);

  topic_watchdog_->add_subscription<msgs::SteeringReport>(
    "/vehicle/sensor/steering_report", 1,
    std::bind(&SSAEstimationNode::steering_report_callback, this, std::placeholders::_1),
    std::bind(&SSAEstimationNode::steering_report_timeout_callback, this, std::placeholders::_1, std::placeholders::_2), 500ms);

  topic_watchdog_->add_subscription<msgs::TUMFloat64PerWheelStamped>(
    "/vehicle/sensor/wheelspeed_radps", 1,
    std::bind(&SSAEstimationNode::wheelspeed_report_callback, this, std::placeholders::_1),
    std::bind(&SSAEstimationNode::wheelspeed_report_timeout_callback, this, std::placeholders::_1, std::placeholders::_2), 500ms);

  topic_watchdog_->add_subscription<diagnostic_msgs::DiagnosticStatus>(
    "/vehicle/sensor/wheelspeed_status", 1,
    std::bind(&SSAEstimationNode::wheelspeed_status_callback, this, std::placeholders::_1),
    std::bind(&SSAEstimationNode::wheelspeed_status_timeout_callback, this, std::placeholders::_1, std::placeholders::_2), 500ms);

  topic_watchdog_->add_subscription<std_msgs::Float32>(
    "/vehicle/sensor/drivetrain_trq_Nm", 1,
    std::bind(&SSAEstimationNode::drivetrain_torque_report_callback, this, std::placeholders::_1),
    std::bind(&SSAEstimationNode::drivetrain_torque_report_timeout_callback, this, std::placeholders::_1, std::placeholders::_2), 500ms);

  topic_watchdog_->add_subscription<msgs::TUMFloat64PerWheelStamped>(
    "/vehicle/sensor/brake_pressure_Pa", 1,
    std::bind(&SSAEstimationNode::brake_pressure_report_callback, this, std::placeholders::_1),
    std::bind(&SSAEstimationNode::brake_pressure_report_timeout_callback, this, std::placeholders::_1, std::placeholders::_2), 500ms);

  // publisher for the ssa estimation output
  pub_odometry_ = nh_.advertise<nav_msgs::Odometry>("/core/ssa/odometry", 1);
  pub_status_ = nh_.advertise<diagnostic_msgs::DiagnosticStatus>("/core/ssa/status", 1);
}

template <class TConfig>
void SSAEstimationNode<TConfig>::function_queue_callback(const ros::TimerEvent &event)
{
  topic_watchdog_->check_timeouts();
  model_update_callback();
}

template <class TConfig>
void SSAEstimationNode<TConfig>::model_update_callback()
{
  if (initialize_) {
    initialize_ = false;
    ROS_INFO("SSA Estimation has been initialized");
  }

  // step the ssa estimation
  ssa_estimation_->step();
  // get the ssa estimation output
  odometry_output_ = ssa_estimation_->get_odometry();
  ros::Time time_pub = ros::Time::now();
  // publish ssa estimation output
  publish_odometry(time_pub);
  diagnostic_msgs::DiagnosticStatus ssa_status_msg;
  ssa_status_msg.level = tam::type_conversions::diagnostic_level_from_type(ssa_estimation_->get_status());
  pub_status_.publish(ssa_status_msg);
  // publish debug messages of the model
  debug_channel_publishers_[debug_channels::state_machine].publish_debug_outputs(
    ssa_estimation_->get_state_machine_debug_output().get());
  debug_channel_publishers_[debug_channels::kalman_filter].publish_debug_outputs(
    ssa_estimation_->get_kalman_filter_debug_output().get());
}

template <class TConfig>
void SSAEstimationNode<TConfig>::publish_odometry(ros::Time time_pub)
{
  // convert odometry type to a nav_msgs::Odometry
  nav_msgs::Odometry odometry_msg =
    tam::type_conversions::odometry_msg_from_type(odometry_output_);

  // construct message header
  odometry_msg.header.stamp = time_pub;
  odometry_msg.header.frame_id = "local_cartesian";
  odometry_msg.child_frame_id = "baselink";

  pub_odometry_.publish(odometry_msg);
}

// Callbacks
template <class TConfig>
void SSAEstimationNode<TConfig>::odometry_callback(
  const nav_msgs::Odometry::ConstPtr odometry_msg)
{
  tam::types::control::Odometry odom_in =
    tam::type_conversions::odometry_type_from_msg(*odometry_msg);
  ssa_estimation_->set_input_vehicle_orientation(odom_in);
  ssa_estimation_->set_input_orientation_status(tam::types::ErrorLvl::OK);
}

template <class TConfig>
void SSAEstimationNode<TConfig>::odometry_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    ssa_estimation_->set_orientation_timeout();
  }
}

template <class TConfig>
void SSAEstimationNode<TConfig>::imu_1_callback(const sensor_msgs::Imu::ConstPtr msg)
{
  tam::types::control::AccelerationwithCovariances input_acceleration =
    tam::type_conversions::acceleration_with_covariances_type_from_imu_msg(*msg);
  tam::types::control::Odometry input_odometry =
    tam::type_conversions::odometry_type_from_imu_msg(*msg);
  // set the imu input valid state OK (0)
  ssa_estimation_->set_input_imu_status(tam::types::ErrorLvl::OK, 0);
  // set acceleration input
  ssa_estimation_->set_input_acceleration(input_acceleration, 0);
  // set angular velocity input
  ssa_estimation_->set_input_angular_velocity(input_odometry, 0);
  received_imu_[0] = true;
}

template <class TConfig>
void SSAEstimationNode<TConfig>::imu_1_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout && received_imu_[0]) {
    ssa_estimation_->set_imu_timeout(0);
  }
}

template <class TConfig>
void SSAEstimationNode<TConfig>::imu_2_callback(const sensor_msgs::Imu::ConstPtr msg)
{
  tam::types::control::AccelerationwithCovariances input_acceleration =
    tam::type_conversions::acceleration_with_covariances_type_from_imu_msg(*msg);
  tam::types::control::Odometry input_odometry =
    tam::type_conversions::odometry_type_from_imu_msg(*msg);
  //   set the imu input valid state OK (0)
  ssa_estimation_->set_input_imu_status(tam::types::ErrorLvl::OK, 1);
  //   set acceleration input
  ssa_estimation_->set_input_acceleration(input_acceleration, 1);
  //   set angular velocity input
  ssa_estimation_->set_input_angular_velocity(input_odometry, 1);
  received_imu_[1] = true;
}

template <class TConfig>
void SSAEstimationNode<TConfig>::imu_2_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout && received_imu_[1]) {
    ssa_estimation_->set_imu_timeout(1);
  }
}

template <class TConfig>
void SSAEstimationNode<TConfig>::imu_3_callback(const sensor_msgs::Imu::ConstPtr msg)
{
  tam::types::control::AccelerationwithCovariances input_acceleration =
    tam::type_conversions::acceleration_with_covariances_type_from_imu_msg(*msg);
  tam::types::control::Odometry input_odometry =
    tam::type_conversions::odometry_type_from_imu_msg(*msg);
  //   set the imu input valid state OK (0)
  ssa_estimation_->set_input_imu_status(tam::types::ErrorLvl::OK, 2);
  //   set acceleration input
  ssa_estimation_->set_input_acceleration(input_acceleration, 2);
  //   set angular velocity input
  ssa_estimation_->set_input_angular_velocity(input_odometry, 2);

  received_imu_[2] = true;
}

template <class TConfig>
void SSAEstimationNode<TConfig>::imu_3_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout && received_imu_[2]) {
    ssa_estimation_->set_imu_timeout(2);
  }
}

template <class TConfig> void SSAEstimationNode<TConfig>::imu_backup_callback(
  const sensor_msgs::Imu::ConstPtr msg)
{
  tam::types::control::AccelerationwithCovariances input_acceleration =
    tam::type_conversions::acceleration_with_covariances_type_from_imu_msg(*msg);
  tam::types::control::Odometry input_odometry =
    tam::type_conversions::odometry_type_from_imu_msg(*msg);
  // set the imu input valid state OK (0)
  ssa_estimation_->set_input_imu_status(tam::types::ErrorLvl::OK,
    TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT - 1);
  // set acceleration input
  ssa_estimation_->set_input_acceleration(input_acceleration,
    TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT - 1);
  // set angular velocity input
  ssa_estimation_->set_input_angular_velocity(input_odometry,
    TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT - 1);
  received_imu_[TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT - 1] = true;
}

template <class TConfig> void SSAEstimationNode<TConfig>::imu_backup_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout && received_imu_[TConfig::NUM_IMU_MEASUREMENT
      + TConfig::NUM_BACKUP_IMU_MEASUREMENT - 1]) {
    ssa_estimation_->set_imu_timeout(
      TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT - 1);
  }
}

template <class TConfig>
void SSAEstimationNode<TConfig>::wheelspeed_report_callback(
  const msgs::TUMFloat64PerWheelStamped::ConstPtr msg)
{
  // input velocity per wheel
  tam::types::common::DataPerWheel<double> input;
  input.front_left = msg->data.front_left;
  input.front_right = msg->data.front_right;
  input.rear_left = msg->data.rear_left;
  input.rear_right = msg->data.rear_right;
  // set the wheelspeed input
  ssa_estimation_->set_input_wheelspeeds(input);
}

template <class TConfig>
void SSAEstimationNode<TConfig>::wheelspeed_status_callback(
  const diagnostic_msgs::DiagnosticStatus::ConstPtr msg)
{
  // set sensor status
  ssa_estimation_->set_input_wheelspeed_status(
    tam::type_conversions::error_type_from_diagnostic_level(msg->level));
}

template <class TConfig>
void SSAEstimationNode<TConfig>::wheelspeed_report_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    ssa_estimation_->set_wheelspeed_timeout();
  }
}

template <class TConfig>
void SSAEstimationNode<TConfig>::wheelspeed_status_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    ssa_estimation_->set_wheelspeed_timeout();
  }
}

template <class TConfig>
void SSAEstimationNode<TConfig>::steering_report_callback(
  const msgs::SteeringReport::ConstPtr msg)
{
  // set the steering angle input and the status
  ssa_estimation_->set_input_steering_angle(msg->steering_tire_angle);
  ssa_estimation_->set_input_steering_angle_status(tam::types::ErrorLvl::OK);
}

template <class TConfig>
void SSAEstimationNode<TConfig>::steering_report_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    ssa_estimation_->set_steering_angle_timeout();
  }
}

template <class TConfig>
void SSAEstimationNode<TConfig>::drivetrain_torque_report_callback(
  const std_msgs::Float32::ConstPtr msg)
{
  // set the drivetrain torque input and the status
  ssa_estimation_->set_input_drivetrain_torque(msg->data);
  ssa_estimation_->set_input_drivetrain_torque_status(tam::types::ErrorLvl::OK);
}

template <class TConfig>
void SSAEstimationNode<TConfig>::drivetrain_torque_report_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    ssa_estimation_->set_drivetrain_torque_timeout();
  }
}

template <class TConfig>
void SSAEstimationNode<TConfig>::brake_pressure_report_callback(
  const msgs::TUMFloat64PerWheelStamped::ConstPtr msg)
{
  // input brake pressure per wheel
  tam::types::common::DataPerWheel<double> input;
  input.front_left = msg->data.front_left;
  input.front_right = msg->data.front_right;
  input.rear_left = msg->data.rear_left;
  input.rear_right = msg->data.rear_right;
  // set the brake pressure input and the status
  ssa_estimation_->set_input_brake_pressure(input);
  ssa_estimation_->set_input_brake_pressure_status(tam::types::ErrorLvl::OK);
}

template <class TConfig>
void SSAEstimationNode<TConfig>::brake_pressure_report_timeout_callback(
  bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
{
  if (timeout) {
    ssa_estimation_->set_brake_pressure_timeout();
  }
}
