// Copyright 2024 Sven Goblirsch
#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <functional>

// ROS
#include <ros/ros.h>

// type definitions
#include "tum_helpers_cpp/containers.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
#include "tum_types_cpp/data_per_wheel.hpp"

// type conversions
#include "tum_type_conversions_ros_cpp/orientation.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"

// messages
#include "msgs/SteeringReport.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "msgs/TUMFloat64PerWheel.h"
#include "msgs/TUMFloat64PerWheelStamped.h"

// side slip angle estimation
#include "ssa_estimation_base/ssa_estimation_base.hpp"
#include "ssa_estimation_cpp/ssa_estimation.hpp"

// State Estimation constants / template input
#include "ssa_estimation_constants/UKF_STM.hpp"

// debug helper
#include "ros_debug_helpers_cpp/debug_publisher.hpp"

// topic watchdog
#include "ros1_watchdog_cpp/topic_watchdog.hpp"

template <class TConfig>
class SSAEstimationNode
{
public:
  explicit SSAEstimationNode(
    std::unique_ptr<tam::core::ssa::SSAEstimationBase> && ssa_estimation,
    ros::NodeHandle nh);

private:
  /**
   * @brief ssa estimation class
   */
  std::unique_ptr<tam::core::ssa::SSAEstimationBase> ssa_estimation_;

  /**
   * @brief parameter manager
   */
  // std::shared_ptr<tam::interfaces::ParamManagerBase> param_manager_;
  // OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  ros::NodeHandle nh_;

  /**
   * @brief model update callback
   */
  ros::Timer model_update_timer_;

  /**
   * @brief topic watchdog
   */
  tam::core::TopicWatchdog::UniquePtr topic_watchdog_;

  /**
   * @brief initialize the initial state in the ssa estimation
   */
  bool initialize_{true};

  /**
  * @brief array containing the information if a imu message on a topic was received once
  */
  std::array<bool, TConfig::NUM_IMU_MEASUREMENT
    + TConfig::NUM_BACKUP_IMU_MEASUREMENT> received_imu_{false};

  // ssa estimation output
  /**
   * @brief ssa estimation odometry output
   */
  tam::types::control::Odometry odometry_output_;

  /**
   * @brief struct defining the number and name of the dubug channels
   */
  struct debug_channels
  {
    enum Channel {
      state_machine,
      kalman_filter,
      NUM_DEBUG_CHANNELS
    };
  };

  /**
   * @brief array containing all debug channels
   */
  std::array<tam::ROS::DebugPublisher, debug_channels::NUM_DEBUG_CHANNELS>
    debug_channel_publishers_;

  // Publisher
  /**
   * @brief nav_msgs::Odometry publisher
   */
  ros::Publisher pub_odometry_;

  /**
   * @brief diagnostic_msgs::DiagnosticStatus publisher
   */
  ros::Publisher pub_status_;
  
  // Functions
  // Timer Callbacks
  /**
   * @brief Function queue to handle topic watchdog and ssa estimation step
   */
  void function_queue_callback(const ros::TimerEvent &event);

  /**
   * @brief Step the side slip angle estimation once and publish the outputs
   */
  void model_update_callback(void);

  /**
   * @brief publish odometry output of the state estimation as nav_msgs::Odometry
   */
  void publish_odometry(ros::Time time_pub);

  // Subscriber
  void odometry_callback(  const nav_msgs::Odometry::ConstPtr odometry_msg);
  void odometry_timeout_callback(bool timeout, [[maybe_unused]]
    std::chrono::milliseconds timeout_now);
  void imu_1_callback(const sensor_msgs::Imu::ConstPtr msg);
  void imu_1_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);
  void imu_2_callback(const sensor_msgs::Imu::ConstPtr msg);
  void imu_2_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);
  void imu_3_callback(const sensor_msgs::Imu::ConstPtr msg);
  void imu_3_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);
  void imu_backup_callback(const sensor_msgs::Imu::ConstPtr msg);
  void imu_backup_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);

  void steering_report_callback(
    const msgs::SteeringReport::ConstPtr msg);
  void steering_report_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);
  
  void wheelspeed_report_callback(const msgs::TUMFloat64PerWheelStamped::ConstPtr msg);
  void wheelspeed_status_callback(const diagnostic_msgs::DiagnosticStatus::ConstPtr msg);
  void wheelspeed_report_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);
  void wheelspeed_status_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);

  void drivetrain_torque_report_callback(
    const std_msgs::Float32::ConstPtr msg);
  void drivetrain_torque_report_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);
  void brake_pressure_report_callback(
    const msgs::TUMFloat64PerWheelStamped::ConstPtr msg);
  void brake_pressure_report_timeout_callback(bool timeout, std::chrono::milliseconds timeout_now);
};
#include "ssa_estimation_cpp/ssa_estimation_impl.hpp"
