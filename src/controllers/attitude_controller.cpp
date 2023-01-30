#include <controllers/attitude_controller.h>

namespace mrs_multirotor_simulator
{

// constructor
AttitudeController::AttitudeController() {
}

void AttitudeController::setParams(const Params& params) {

  params_ = params;

  pid_x_.reset();
  pid_y_.reset();
  pid_z_.reset();

  pid_x_.setParams(params.kp * params.mass, params.kd * params.mass, params.ki * params.mass, params.max_rate_roll_pitch);
  pid_y_.setParams(params.kp * params.mass, params.kd * params.mass, params.ki * params.mass, params.max_rate_roll_pitch);
  pid_z_.setParams(params.kp * params.mass, params.kd * params.mass, params.ki * params.mass, params.max_rate_yaw);
}

reference::AttitudeRate AttitudeController::getControlSignal(const QuadrotorModel::State& state, const reference::Attitude& reference, const double& dt) {

  // orientation error
  Eigen::Matrix3d E = 0.5 * (reference.orientation.transpose() * state.R - state.R.transpose() * reference.orientation);

  // vectorize the orientation error
  Eigen::Vector3d Eq;
  Eq << (E(1, 2) - E(2, 1)) / 2.0, (E(2, 0) - E(0, 2)) / 2.0, (E(0, 1) - E(1, 0)) / 2.0;

  reference::AttitudeRate output;

  output.rate_x   = pid_x_.update(Eq(0), dt);
  output.rate_y   = pid_y_.update(Eq(1), dt);
  output.rate_z   = pid_z_.update(Eq(2), dt);
  output.throttle = reference.throttle;

  return output;
}

}  // namespace mrs_multirotor_simulator
