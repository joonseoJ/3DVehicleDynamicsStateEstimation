// Copyright 2023 Marcel Weinmann
#pragma once
#include <vector>
#include <string>

namespace tam::core::state
{
struct EKF_3D : public KF_BASE
{
  /**
   * @brief Kalman filter base constants: Size of the input vector
   */
  enum {
    INPUT_DPHI_RADPS,
    INPUT_DTHETA_RADPS,
    INPUT_DPSI_RADPS,
    INPUT_AX_MPS2,
    INPUT_AY_MPS2,
    INPUT_AZ_MPS2,
    INPUT_VECTOR_SIZE
  };

  /**
   * @brief Kalman filter base constants: Size of the process noise vector
   *        (diagonal elements of the matrix)
   */
  enum {
    Q_DPHI_RADPS,
    Q_DTHETA_RADPS,
    Q_DPSI_RADPS,
    Q_AX_MPS2,
    Q_AY_MPS2,
    Q_AZ_MPS2,
    Q_VECTOR_SIZE
  };

  /**
   * @brief Kalman filter base constants: Size of the position measurement vector
   */
  enum {
    MEASUREMENT_POS_X_M,
    MEASUREMENT_POS_Y_M,
    MEASUREMENT_POS_Z_M,
    POS_MEASUREMENT_VECTOR_SIZE
  };

  /**
   * @brief Kalman filter base constants: Size of the orientation measurement vector
   */
  enum {
    MEASUREMENT_PHI_RAD,
    MEASUREMENT_THETA_RAD,
    MEASUREMENT_PSI_RAD,
    ORIENTATION_MEASUREMENT_VECTOR_SIZE
  };

  /**
   * @brief Kalman filter base constants: Size of the linear velocity measurement vector
   */
  enum {
    MEASUREMENT_VX_MPS,
    MEASUREMENT_VY_MPS,
    MEASUREMENT_VZ_MPS,
    VEL_MEASUREMENT_VECTOR_SIZE
  };

  /**
   * @brief Kalman filter base constants: Entire length of the position measurement vector
   */
  static inline constexpr int MEASUREMENT_VECTOR_OFFSET_ORIENTATION =
    POS_MEASUREMENT_VECTOR_SIZE * NUM_POS_MEASUREMENT;

  /**
   * @brief Kalman filter base constants: 
   *        Entire length of the position and orientationmeasurement vector
   */
  static inline constexpr int MEASUREMENT_VECTOR_OFFSET_VEL =
    POS_MEASUREMENT_VECTOR_SIZE * NUM_POS_MEASUREMENT
    + ORIENTATION_MEASUREMENT_VECTOR_SIZE * NUM_ORIENTATION_MEASUREMENT;

  /**
   * @brief Kalman filter base constants: Entire length of the measurement vector
   */
  static inline constexpr int MEASUREMENT_VECTOR_SIZE =
    POS_MEASUREMENT_VECTOR_SIZE * NUM_POS_MEASUREMENT
    + ORIENTATION_MEASUREMENT_VECTOR_SIZE * NUM_ORIENTATION_MEASUREMENT
    + VEL_MEASUREMENT_VECTOR_SIZE * NUM_VEL_MEASUREMENT;

  /**
   * @brief Extended Kalman filter constants: Size of the state vector
   */
  enum {
    STATE_POS_X_M,
    STATE_POS_Y_M,
    STATE_POS_Z_M,
    STATE_PHI_RAD,
    STATE_THETA_RAD,
    STATE_PSI_RAD,
    STATE_VX_MPS,
    STATE_VY_MPS,
    STATE_VZ_MPS,
    STATE_VECTOR_SIZE
  };

  /**
   * @brief Extended Kalman filter constants: indicator which index of the state vector
   *                                          is a angle and has to be normalized
   */
  static inline std::vector<bool>
    STATE_VECTOR_ANGLE_INDICATOR = {false, false, false, true, true, true, false, false, false};
};
}  // namespace tam::core::state
