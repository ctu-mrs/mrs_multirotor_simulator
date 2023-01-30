#ifndef REFERENCES_H
#define REFERENCES_H

#include <eigen3/Eigen/Eigen>

namespace mrs_multirotor_simulator
{

namespace reference
{

/* Actuators //{ */

typedef struct
{

  /**
   * @brief vector of motor throttles scaled as [0, 1]
   */
  Eigen::VectorXd motors;
} Actuators;

//}

/* ControlGroup //{ */

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

//}

/* AttitudeRate //{ */

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

//}

/* Attitude //{ */

typedef struct
{
  Eigen::Matrix3d orientation;

  /**
   * @brief the collective throttle along body-Z normalized to [-1, 1]
   */
  double throttle;
} Attitude;

//}

/* Acceleration //{ */

typedef struct
{
  Eigen::Vector3d acceleration;

  /**
   * @brief atan2 of body-x axis projected to the ground plane
   */
  double heading;
} Acceleration;

//}

/* Velocity //{ */

typedef struct
{
  Eigen::Vector3d velocity;

  /**
   * @brief atan2 of body-x axis projected to the ground plane
   */
  double heading;
} Velocity;

//}

/* Position //{ */

typedef struct
{
  Eigen::Vector3d position;

  /**
   * @brief atan2 of body-x axis projected to the ground plane
   */
  double heading;
} Position;

//}

}  // namespace reference

}  // namespace mrs_multirotor_simulator

#endif  // REFERENCES_H
