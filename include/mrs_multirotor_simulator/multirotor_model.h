/**
 * @brief Dynamic simulation of a multirotor helicopter.
 *
 * Acknowledgement:
 * * https://github.com/HKUST-Aerial-Robotics/Fast-Planner
 */
#ifndef MULTIROTOR_DYNAMICS_H
#define MULTIROTOR_DYNAMICS_H

#define N_INTERNAL_STATES 18

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/array.hpp>
#include <iostream>
#include <boost/bind.hpp>
#include <ode/boost/numeric/odeint.hpp>

#include <mrs_multirotor_simulator/controllers/references.h>

#include <mrs_lib/attitude_converter.h>

namespace mrs_multirotor_simulator
{

class ModelParams {
public:
  ModelParams() {

    // | -------- default parameters of the x500 quadrotor -------- |
    // it is recommended to load them through the setParams() method

    n_motors             = 4;
    g                    = 9.81;
    mass                 = 2.0;
    kf                   = 0.00000022970;
    km                   = 0.07;
    prop_radius          = 0.15;
    arm_length           = 0.25;
    body_height          = 0.1;
    motor_time_constant  = 0.03;
    max_rpm              = 8951;
    min_rpm              = 895;
    air_resistance_coeff = 0.30;

    J       = Eigen::Matrix3d::Zero();
    J(0, 0) = mass * (3.0 * arm_length * arm_length + body_height * body_height) / 12.0;
    J(1, 1) = mass * (3.0 * arm_length * arm_length + body_height * body_height) / 12.0;
    J(2, 2) = (mass * arm_length * arm_length) / 2.0;

    allocation_matrix = Eigen::MatrixXd::Zero(4, 4);

    // clang-format off
    allocation_matrix <<
      -0.707, 0.707, 0.707,  -0.707,
      -0.707, 0.707, -0.707, 0.707,
      -1,     -1,    1,      1,
      1,      1,     1,      1;
    // clang-format on

    allocation_matrix.row(0) *= arm_length * kf;
    allocation_matrix.row(1) *= arm_length * kf;
    allocation_matrix.row(2) *= km * (3.0 * prop_radius) * kf;
    allocation_matrix.row(3) *= kf;

    ground_enabled        = false;
    takeoff_patch_enabled = true;
  }

  int    n_motors;
  double g;
  double mass;
  double kf;
  double km;
  double prop_radius;
  double arm_length;
  double body_height;
  double motor_time_constant;
  double max_rpm;
  double min_rpm;
  double air_resistance_coeff;

  Eigen::Matrix3d J;
  Eigen::MatrixXd allocation_matrix;

  bool   ground_enabled;
  double ground_z;

  bool takeoff_patch_enabled;
};

class MultirotorModel {

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

  MultirotorModel();

  MultirotorModel(const ModelParams& params, const Eigen::Vector4d& spawn);

  const MultirotorModel::State& getState(void) const;

  void setState(const MultirotorModel::State& state);

  void setStatePos(const Eigen::Vector4d& pos);

  const Eigen::Vector3d& getExternalForce(void) const;
  void                   setExternalForce(const Eigen::Vector3d& force);

  const Eigen::Vector3d& getExternalMoment(void) const;
  void                   setExternalMoment(const Eigen::Vector3d& moment);

  void setInput(const reference::Actuators& input);

  void step(const double& dt);

  typedef boost::array<double, N_INTERNAL_STATES> InternalState;

  void operator()(const MultirotorModel::InternalState& x, MultirotorModel::InternalState& dxdt, const double t);

  Eigen::Vector3d getImuAcceleration() const;

  ModelParams getParams(void);
  void        setParams(const ModelParams& params);

private:
  void initializeState(void);

  void updateInternalState(void);

  MultirotorModel::State state_;

  Eigen::Vector3d imu_acceleration_;

  Eigen::VectorXd input_;
  Eigen::Vector3d external_force_;
  Eigen::Vector3d external_moment_;

  Eigen::Vector3d _initial_pos_;

  ModelParams params_;

  InternalState internal_state_;
};

}  // namespace mrs_multirotor_simulator

#endif
