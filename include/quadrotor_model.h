/**
 * @brief Dynamic simulation of a multirotor helicopter.
 *
 * Acknowledgement:
 * * https://github.com/HKUST-Aerial-Robotics/Fast-Planner
 */
#ifndef MULTIROTOR_GENERIC_MODEL_H
#define MULTIROTOR_GENERIC_MODEL_H

#define N_INTERNAL_STATES 18

#include <controllers/references.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/array.hpp>
#include <iostream>
#include <boost/bind.hpp>
#include <ode/boost/numeric/odeint.hpp>

namespace mrs_multirotor_simulator
{

typedef struct
{

  int    n_motors;
  double g;
  double mass;
  double kf;
  double prop_radius;
  double arm_length;
  double body_height;
  double motor_time_constant;
  double max_rpm;
  double min_rpm;
  double propulsion_force_constant;
  double propulsion_moment_constant;
  double air_resistance_coeff;

  Eigen::Matrix3d J;
  Eigen::MatrixXd allocation_matrix;
  Eigen::MatrixXd mixing_matrix;

  bool   ground_enabled;
  double ground_z;

  bool takeoff_patch_enabled;

} ModelParams_t;

class QuadrotorModel {

public:
  struct State
  {
    Eigen::Vector3d x;
    Eigen::Vector3d v;
    Eigen::Vector3d v_prev;
    Eigen::Matrix3d R;
    Eigen::Vector3d omega;
    Eigen::VectorXd motor_rpm;
  };

  QuadrotorModel();

  QuadrotorModel(const ModelParams_t& params, const Eigen::Vector3d& initial_pos);

  const QuadrotorModel::State& getState(void) const;

  void setState(const QuadrotorModel::State& state);

  void setStatePos(const Eigen::Vector3d& Pos);

  const Eigen::Vector3d& getExternalForce(void) const;
  void                   setExternalForce(const Eigen::Vector3d& force);

  const Eigen::Vector3d& getExternalMoment(void) const;
  void                   setExternalMoment(const Eigen::Vector3d& moment);

  void setInput(const reference::Actuators& input);

  void step(const double& dt);

  typedef boost::array<double, N_INTERNAL_STATES> InternalState;

  void operator()(const QuadrotorModel::InternalState& x, QuadrotorModel::InternalState& dxdt, const double t);

  Eigen::Vector3d getImuAcceleration() const;

private:
  void updateInternalState(void);

  QuadrotorModel::State state_;

  Eigen::Vector3d imu_acceleration_;

  Eigen::VectorXd input_;
  Eigen::Vector3d external_force_;
  Eigen::Vector3d external_moment_;

  Eigen::Vector3d _initial_pos_;

  ModelParams_t params_;

  InternalState internal_state_;
};

}  // namespace mrs_multirotor_simulator

#endif
