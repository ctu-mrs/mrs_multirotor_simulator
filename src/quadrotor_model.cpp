#include <quadrotor_model.h>
#include <ode/boost/numeric/odeint.hpp>
#include <Eigen/Geometry>
#include <boost/bind.hpp>
#include <iostream>

#include <ros/ros.h>

namespace odeint = boost::numeric::odeint;

namespace mrs_multirotor_simulator
{

/* constructor QuadrotorModel //{ */

QuadrotorModel::QuadrotorModel(void) {
}

QuadrotorModel::QuadrotorModel(const ModelParams_t& params) {

  params_ = params;

  state_.x     = Eigen::Vector3d::Zero();
  state_.v     = Eigen::Vector3d::Zero();
  state_.R     = Eigen::Matrix3d::Identity();
  state_.omega = Eigen::Vector3d::Zero();

  state_.motor_rpm = Eigen::VectorXd::Zero(params.n_motors);
  input_           = Eigen::VectorXd::Zero(params.n_motors);

  external_force_.setZero();
  external_moment_.setZero();

  updateInternalState();
}

//}

/* step() //{ */

void QuadrotorModel::step(const double& dt) {

  auto save = internal_state_;

  boost::numeric::odeint::runge_kutta4<InternalState> rk;

  odeint::integrate_n_steps(rk, boost::ref(*this), internal_state_, 0.0, dt, 1);

  for (int i = 0; i < 18; ++i) {
    if (std::isnan(internal_state_[i])) {

      std::cout << "dump " << i << " << pos ";

      for (int j = 0; j < 18; ++j) {
        std::cout << save[j] << " ";
      }

      std::cout << std::endl;
      internal_state_ = save;
      break;
    }
  }

  for (int i = 0; i < 3; i++) {
    state_.x(i)     = internal_state_[0 + i];
    state_.v(i)     = internal_state_[3 + i];
    state_.R(i, 0)  = internal_state_[6 + i];
    state_.R(i, 1)  = internal_state_[9 + i];
    state_.R(i, 2)  = internal_state_[12 + i];
    state_.omega(i) = internal_state_[15 + i];
  }

  double filter_const = exp((-dt) / (params_.motor_time_constant));

  state_.motor_rpm = filter_const * state_.motor_rpm + (1 - filter_const) * input_;

  // Re-orthonormalize R (polar decomposition)
  Eigen::LLT<Eigen::Matrix3d> llt(state_.R.transpose() * state_.R);

  Eigen::Matrix3d P = llt.matrixL();
  Eigen::Matrix3d R = state_.R * P.inverse();
  state_.R          = R;

  // Don't go below zero, simulate floor
  if (state_.x(2) < 0.0 && state_.v(2) < 0) {
    state_.x(2) = 0;
    state_.v(2) = 0;
  }

  updateInternalState();
}

//}

/* operator() //{ */

void QuadrotorModel::operator()(const QuadrotorModel::InternalState& x, QuadrotorModel::InternalState& dxdt, const double t) {

  State cur_state;

  for (int i = 0; i < 3; i++) {
    cur_state.x(i)     = x[0 + i];
    cur_state.v(i)     = x[3 + i];
    cur_state.R(i, 0)  = x[6 + i];
    cur_state.R(i, 1)  = x[9 + i];
    cur_state.R(i, 2)  = x[12 + i];
    cur_state.omega(i) = x[15 + i];
  }

  Eigen::LLT<Eigen::Matrix3d> llt(cur_state.R.transpose() * cur_state.R);
  Eigen::Matrix3d             P = llt.matrixL();
  Eigen::Matrix3d             R = cur_state.R * P.inverse();

  Eigen::Vector3d x_dot;
  Eigen::Vector3d v_dot;
  Eigen::Vector3d omega_dot;
  Eigen::Matrix3d R_dot;

  Eigen::Matrix3d omega_tensor(Eigen::Matrix3d::Zero());

  omega_tensor(2, 1) = cur_state.omega(0);
  omega_tensor(1, 2) = -cur_state.omega(0);
  omega_tensor(0, 2) = cur_state.omega(1);
  omega_tensor(2, 0) = -cur_state.omega(1);
  omega_tensor(1, 0) = cur_state.omega(2);
  omega_tensor(0, 1) = -cur_state.omega(2);

  Eigen::VectorXd motor_rpm_sq = state_.motor_rpm.array().square();

  Eigen::Vector4d torque_thrust = params_.mixing_matrix * (params_.kf * motor_rpm_sq);
  double          thrust        = torque_thrust(3);

  double resistance = params_.air_resistance_coeff * M_PI * (params_.arm_length) * (params_.arm_length) * cur_state.v.norm() * cur_state.v.norm();

  Eigen::Vector3d vnorm = cur_state.v;
  if (vnorm.norm() != 0) {
    vnorm.normalize();
  }

  x_dot = cur_state.v;
  v_dot = -Eigen::Vector3d(0, 0, params_.g) + thrust * R.col(2) / params_.mass + external_force_ / params_.mass - resistance * vnorm / params_.mass;

  acc_ = v_dot;

  R_dot = R * omega_tensor;

  omega_dot = params_.J.inverse() * (torque_thrust.topRows(3) - cur_state.omega.cross(params_.J * cur_state.omega) + external_moment_);

  for (int i = 0; i < 3; i++) {
    dxdt[0 + i]  = x_dot(i);
    dxdt[3 + i]  = v_dot(i);
    dxdt[6 + i]  = R_dot(i, 0);
    dxdt[9 + i]  = R_dot(i, 1);
    dxdt[12 + i] = R_dot(i, 2);
    dxdt[15 + i] = omega_dot(i);
  }

  for (int i = 0; i < 18; ++i) {
    if (std::isnan(dxdt[i])) {
      dxdt[i] = 0;
    }
  }
}

//}

/* updatedInternalState() //{ */

void QuadrotorModel::updateInternalState(void) {

  for (int i = 0; i < 3; i++) {
    internal_state_[0 + i]  = state_.x(i);
    internal_state_[3 + i]  = state_.v(i);
    internal_state_[6 + i]  = state_.R(i, 0);
    internal_state_[9 + i]  = state_.R(i, 1);
    internal_state_[12 + i] = state_.R(i, 2);
    internal_state_[15 + i] = state_.omega(i);
  }
}

//}

// | ------------------- setters and getters ------------------ |

/* setInput() //{ */

void QuadrotorModel::setInput(const reference::Motors& input) {

  for (int i = 0; i < params_.n_motors; i++) {

    double val = input.motors(i);

    if (!std::isfinite(val)) {
      ROS_ERROR("[QuadrotorModel] NaN detected in motor input!!!");
      val = 0;
    }

    if (val < 0.0) {
      val = 0.0;
    } else if (val > 1.0) {
      val = 1.0;
    }

    input_(i) = params_.min_rpm + (params_.max_rpm - params_.min_rpm) * val;
  }
}

//}

/* getState() //{ */

const QuadrotorModel::State& QuadrotorModel::getState(void) const {
  return state_;
}

//}

/* setState() //{ */

void QuadrotorModel::setState(const QuadrotorModel::State& state) {

  state_.x         = state.x;
  state_.v         = state.v;
  state_.R         = state.R;
  state_.omega     = state.omega;
  state_.motor_rpm = state.motor_rpm;

  updateInternalState();
}

//}

/* setStatePos() //{ */

void QuadrotorModel::setStatePos(const Eigen::Vector3d& Pos) {

  state_.x = Pos;

  updateInternalState();
}

//}

/* getExternalForce() //{ */

const Eigen::Vector3d& QuadrotorModel::getExternalForce(void) const {
  return external_force_;
}

//}

/* setExternalForce() //{ */

void QuadrotorModel::setExternalForce(const Eigen::Vector3d& force) {
  external_force_ = force;
}

//}

/* getExternalMoment() //{ */

const Eigen::Vector3d& QuadrotorModel::getExternalMoment(void) const {
  return external_moment_;
}

//}

/* setExternalMoment() //{ */

void QuadrotorModel::setExternalMoment(const Eigen::Vector3d& moment) {
  external_moment_ = moment;
}

//}

/* getAcc() //{ */

Eigen::Vector3d QuadrotorModel::getAcc() const {
  return acc_;
}

//}

}  // namespace mrs_multirotor_simulator
