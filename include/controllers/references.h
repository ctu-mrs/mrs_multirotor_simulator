#ifndef REFERENCES_H
#define REFERENCES_H

#include <eigen3/Eigen/Eigen>

namespace mrs_multirotor_simulator
{

namespace reference
{

typedef struct
{

  /**
   * @brief vector of motor throttles scaled as [0, 1]
   */
  Eigen::VectorXd motors;
} Actuators;

typedef struct
{
  /**
   * @brief the applied roll (around body-X) torque normalized to [-1, 1]
   */
  double roll;

  /**
   * @brief the applied pitch (around body-Y) torque normalized to [-1, 1]
   */
  double pitch;

  /**
   * @brief the applied yaw (around body-Z) torque normalized to [-1, 1]
   */
  double yaw;

  /**
   * @brief the collective throttle along body-Z normalized to [-1, 1]
   */
  double throttle;
} ControlGroup;

typedef struct
{
  /**
   * @brief angular rate around body-x in [rad]
   */
  double rate_x;

  /**
   * @brief angular rate around body-y in [rad]
   */
  double rate_y;

  /**
   * @brief angular rate around body-z in [rad]
   */
  double rate_z;

  /**
   * @brief the collective throttle along body-Z normalized to [-1, 1]
   */
  double throttle;
} AttitudeRate;

typedef struct
{
  Eigen::Matrix3d orientation;

  /**
   * @brief the collective throttle along body-Z normalized to [-1, 1]
   */
  double throttle;
} Attitude;

}  // namespace reference

}  // namespace mrs_multirotor_simulator

#endif  // REFERENCES_H
