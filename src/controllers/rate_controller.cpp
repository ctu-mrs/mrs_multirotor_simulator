#include <controllers/rate_controller.h>
#include <math.h>
#include <iostream>

namespace mrs_multirotor_simulator
{

// constructor
RateController::RateController() {
}

void RateController::setParams(const Params& params) {

  params_ = params;

  pid_x_.reset();
  pid_y_.reset();
  pid_z_.reset();

  pid_x_.setParams(params.kp, params.kd, params.ki);
  pid_y_.setParams(params.kp, params.kd, params.ki);
  pid_z_.setParams(params.kp, params.kd, params.ki);
}

Eigen::VectorXd RateController::getControlSignal(const QuadrotorModel::State& state, const RateController::Reference& reference, const double& dt) {

  Eigen::Vector3d wr = reference.angular_rate - state.omega;

  Eigen::Vector4d action;

  action(0) = pid_x_.update(wr(0), dt);
  action(1) = pid_y_.update(wr(1), dt);
  action(2) = pid_z_.update(wr(2), dt);
  action(3) = reference.throttle * params_.n_motors * params_.force_coef * pow((params_.max_rpm - params_.min_rpm) + params_.min_rpm, 2.0);

  Eigen::MatrixXd mixer = params_.allocation_matrix.inverse();

  Eigen::VectorXd motors = mixer * action;

  for (int i = 0; i < params_.n_motors; i++) {

    double arg = motors(i) / params_.force_coef;

    if (arg > 0) {
      motors(i) = (sqrt(arg) - params_.min_rpm) / (params_.max_rpm - params_.min_rpm);
    } else {
      motors(i) = 0;
    }
  }

  return motors;
}

}  // namespace mrs_multirotor_simulator
