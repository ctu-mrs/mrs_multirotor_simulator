#include <mrs_multirotor_simulator/controllers/velocity_controller.h>

namespace mrs_multirotor_simulator
{

// constructor
VelocityController::VelocityController() {
}

void VelocityController::setParams(const Params& params) {

  params_ = params;

  pid_x_.reset();
  pid_y_.reset();
  pid_z_.reset();

  pid_x_.setParams(params.kp, params.kd, params.ki, params.max_acceleration);
  pid_y_.setParams(params.kp, params.kd, params.ki, params.max_acceleration);
  pid_z_.setParams(params.kp, params.kd, params.ki, params.max_acceleration);
}

reference::Acceleration VelocityController::getControlSignal(const MultirotorModel::State& state, const reference::Velocity& reference, const double& dt) {

  Eigen::Vector3d vel_error = reference.velocity - state.v;

  reference::Acceleration output;

  output.acceleration[0] = pid_x_.update(vel_error[0], dt);
  output.acceleration[1] = pid_y_.update(vel_error[1], dt);
  output.acceleration[2] = pid_z_.update(vel_error[2], dt);

  output.heading = reference.heading;

  return output;
}

}  // namespace mrs_multirotor_simulator
