#ifndef MULTIROTOR_GENERIC_MODEL_H
#define MULTIROTOR_GENERIC_MODEL_H

#include <controllers/references.h>
#include <Eigen/Core>
#include <boost/array.hpp>

namespace mrs_multirotor_simulator
{

typedef struct
{

  int    n_motors;
  double g;  // gravity acceleration
  double mass;
  double kf;
  double prop_radius;
  double arm_length;
  double body_height;
  double motor_time_constant;  // unit: sec
  double max_rpm;
  double min_rpm;
  double propulsion_force_constant;
  double propulsion_moment_constant;
  double air_resistance_coeff;

  Eigen::Matrix3d J;
  Eigen::MatrixXd allocation_matrix;
  Eigen::MatrixXd mixing_matrix;

} ModelParams_t;

class QuadrotorModel {

public:
  struct State
  {
    Eigen::Vector3d x;
    Eigen::Vector3d v;
    Eigen::Matrix3d R;
    Eigen::Vector3d omega;
    Eigen::VectorXd motor_rpm;
  };

  QuadrotorModel();

  QuadrotorModel(const ModelParams_t& params);

  const QuadrotorModel::State& getState(void) const;

  void setState(const QuadrotorModel::State& state);

  void setStatePos(const Eigen::Vector3d& Pos);

  double getMass(void) const;
  void   setMass(double mass);

  double getGravity(void) const;
  void   setGravity(double g);

  const Eigen::Vector3d& getExternalForce(void) const;
  void                   setExternalForce(const Eigen::Vector3d& force);

  const Eigen::Vector3d& getExternalMoment(void) const;
  void                   setExternalMoment(const Eigen::Vector3d& moment);

  void setInput(const reference::Actuators& input);

  // Runs the actual dynamics simulation with a time step of dt
  void step(const double& dt);

  // For internal use, but needs to be public for odeint
  typedef boost::array<double, 18> InternalState;
  void                             operator()(const QuadrotorModel::InternalState& x, QuadrotorModel::InternalState& dxdt, const double t);

  Eigen::Vector3d getAcc() const;

private:
  void updateInternalState(void);

  QuadrotorModel::State state_;

  Eigen::Vector3d acc_;

  Eigen::VectorXd input_;
  Eigen::Vector3d external_force_;
  Eigen::Vector3d external_moment_;

  ModelParams_t params_;

  InternalState internal_state_;
};

}  // namespace mrs_multirotor_simulator

#endif
