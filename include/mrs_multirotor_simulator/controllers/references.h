#ifndef REFERENCES_H
#define REFERENCES_H

#include <eigen3/Eigen/Eigen>

namespace mrs_multirotor_simulator
{

namespace reference
{

/* Actuators //{ */

class Actuators {

public:
  /**
   * @brief vector of motor throttles scaled as [0, 1]
   */
  Eigen::VectorXd motors;
};

//}

/* ControlGroup //{ */

class ControlGroup {
public:
  /**
   * @brief the applied roll (around body-X) torque normalized to [-1, 1]
   */
  double roll = 0;

  /**
   * @brief the applied pitch (around body-Y) torque normalized to [-1, 1]
   */
  double pitch = 0;

  /**
   * @brief the applied yaw (around body-Z) torque normalized to [-1, 1]
   */
  double yaw = 0;

  /**
   * @brief the collective throttle along body-Z normalized to [-1, 1]
   */
  double throttle = 0;
};

//}

/* AttitudeRate //{ */

class AttitudeRate {
public:
  /**
   * @brief angular rate around body-x in [rad]
   */
  double rate_x = 0;

  /**
   * @brief angular rate around body-y in [rad]
   */
  double rate_y = 0;

  /**
   * @brief angular rate around body-z in [rad]
   */
  double rate_z = 0;

  /**
   * @brief the collective throttle along body-Z normalized to [-1, 1]
   */
  double throttle = 0;
};

//}

/* Attitude //{ */

class Attitude {
public:
  Attitude() {
    this->orientation = Eigen::Matrix3d::Identity();
  }

  Eigen::Matrix3d orientation;

  /**
   * @brief the collective throttle along body-Z normalized to [-1, 1]
   */
  double throttle = 0;
};

//}

/* Acceleration //{ */

class Acceleration {
public:
  Acceleration() {
    this->acceleration = Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d acceleration;

  /**
   * @brief atan2 of body-x axis projected to the ground plane
   */
  double heading = 0;
};

//}

/* Velocity //{ */

class Velocity {
public:
  Velocity() {
    this->velocity = Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d velocity;

  /**
   * @brief atan2 of body-x axis projected to the ground plane
   */
  double heading = 0;
};

//}

/* Position //{ */

class Position {
public:
  Position() {
    this->position = Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d position;

  /**
   * @brief atan2 of body-x axis projected to the ground plane
   */
  double heading = 0;
};

//}

}  // namespace reference

}  // namespace mrs_multirotor_simulator

#endif  // REFERENCES_H
