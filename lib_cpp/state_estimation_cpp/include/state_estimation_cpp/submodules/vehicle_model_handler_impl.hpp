// Copyright 2023 Marcel Weinmann
#pragma once
#include "state_estimation_cpp/submodules/vehicle_model_handler.hpp"

template <class TConfig> tam::core::state::VehicleModelHandler<TConfig>::VehicleModelHandler(
  const std::string & vehicle_model, ros::NodeHandle nh)
{
  nh_ = nh;

  nh_.setParam("tyreradius_front_m", 0.293475);
  nh_.setParam("tyreradius_rear_m", 0.307435);
  nh_.setParam("l_WheelbaseF_m", 1.724);
  nh_.setParam("l_WheelbaseR_m", 1.247);

  if (vehicle_model == "kinematic") {
    kinematic_model_ = true;
  } else if (vehicle_model == "non-holonomic") {
    non_holonomic_model_ = true;
  } else if (vehicle_model == "single-track-model") {
    single_track_model_ = true;
  } else {
    throw std::invalid_argument(
      "[StateEstimationCPP]: Given vehicle model is not implemented");
  }
}

/**
 * @brief Update the State Estimaion Wheelspeed Odometry input
 * 
 * @param[in] wheel             - tam::types::common::DataPerWheel<double>:
 *                                angular velocity per wheel 
 */
template <class TConfig>
void tam::core::state::VehicleModelHandler<TConfig>::input_wheel_angular_velocities(
  const tam::types::common::DataPerWheel<double> & wheel)
{
  // calculate the average velocity over all wheels and apply the wheelspeed scale
  // if a vehicle model is configured
  if (!kinematic_model_) {
    double tyreradius_front_m, tyreradius_rear_m
    double average_absolute_velocity = (
      (wheel.front_left + wheel.front_right) * tyreradius_front_m +
      (wheel.rear_left + wheel.rear_right) * tyreradius_rear_m
    ) / 4;

    // average velocity over all wheels in x-direction
    if (non_holonomic_model_) {
      vehicle_odometry_.velocity_mps.x = average_absolute_velocity;
      vehicle_odometry_.velocity_mps.y = 0.0;
      vehicle_odometry_.velocity_mps.z = 0.0;
    }

    // velocity in x- and y-direction calculated from the linear STM
    if (single_track_model_) {
      vehicle_odometry_.velocity_mps.x = average_absolute_velocity * cos(side_slip_angle_);
      vehicle_odometry_.velocity_mps.y = average_absolute_velocity * sin(side_slip_angle_);
      vehicle_odometry_.velocity_mps.z = 0.0;
    }
  }
}

/**
 * @brief Update the status of the wheelspeed signal
 * 
 * @param[in] status            - tam::types::ErrorLvl:
 *                                Wheelspeed status
 */
template <class TConfig>
void tam::core::state::VehicleModelHandler<TConfig>::input_wheelspeed_status(
  const tam::types::ErrorLvl status)
{
  wheelspeed_status_ = status;
}

/**
 * @brief Update the State Estimaion Wheelspeed Odometry input
 * 
 * @param[in] steering_angle    - double:
 *                                steering angle in rad
 */
template <class TConfig>
void tam::core::state::VehicleModelHandler<TConfig>::input_steering_angle(double steering_angle)
{
  // update the side slip angle using a linear STM
  if (single_track_model_) {
    double l_WheelbaseF_m, l_WheelbaseR_m;
    nh_.getParam("l_WheelbaseF_m", l_WheelbaseF_m);
    nh_.getParam("l_WheelbaseR_m", l_WheelbaseR_m);
    side_slip_angle_ = std::atan(
      std::tan(steering_angle)* l_WheelbaseR_m / (l_WheelbaseF_m + l_WheelbaseR_m)
    );
  }
}

/**
 * @brief Update the status of the steering angle status signal
 * 
 * @param[in] status            - tam::types::ErrorLvl:
 *                                Wheelspeed status
 */
template <class TConfig>
void tam::core::state::VehicleModelHandler<TConfig>::input_steering_angle_status(
  const tam::types::ErrorLvl status)
{
  steering_angle_status_ = status;
}

/**
 * @brief Get the current linear velocities calculated form the angular velocities of the wheels
 * 
 * @param[out] result           - tam::types::control::Odometry:
 *                                [vx_VelCoG_mps, vy_VelCoG_mps]
 */
template <class TConfig> const tam::types::control::Odometry &
tam::core::state::VehicleModelHandler<TConfig>::get_vehicle_model_odometry(void)
{
  return vehicle_odometry_;
}

/**
 * @brief Get the current sensor status w.r.t. the vehicle model chosen
 * 
 * @param[out] result           - tam::types::ErrorLvl:
 */
template <class TConfig>
tam::types::ErrorLvl tam::core::state::VehicleModelHandler<TConfig>::get_status(void)
{
  // only return OK if the output of the vehicle model should be fused
  // and all needed sensors are OK
  if (non_holonomic_model_ && wheelspeed_status_ == tam::types::ErrorLvl::OK) {
    return tam::types::ErrorLvl::OK;
  }

  if (single_track_model_ && wheelspeed_status_ == tam::types::ErrorLvl::OK
      && steering_angle_status_ == tam::types::ErrorLvl::OK) {
    return tam::types::ErrorLvl::OK;
  }

  // else return an error such that the solution is not fused
  return tam::types::ErrorLvl::ERROR;
}

/**
 * @brief returns a pointer to the param manager
 *
 * @param[out]                  - ros::NodeHandle
 */
template <class TConfig> ros::NodeHandle
  tam::core::state::VehicleModelHandler<TConfig>::get_param_handler(void)
{
  return nh_;
}