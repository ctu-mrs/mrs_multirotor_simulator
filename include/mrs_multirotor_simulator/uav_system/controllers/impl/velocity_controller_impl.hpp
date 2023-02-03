#ifndef VELOCITY_CONTROLLER_IMPL_H
#define VELOCITY_CONTROLLER_IMPL_H

namespace mrs_multirotor_simulator
{

// constructor
VelocityController::VelocityController() {
  initializePIDs();
}

VelocityController::VelocityController(const ModelParams& model_params) {
  model_params_ = model_params;

  initializePIDs();
}

void VelocityController::setParams(const Params& params) {

  params_ = params;

  initializePIDs();
}

void VelocityController::initializePIDs(void) {

  pid_x_.reset();
  pid_y_.reset();
  pid_z_.reset();

  pid_x_.setParams(params_.kp, params_.kd, params_.ki, params_.max_acceleration);
  pid_y_.setParams(params_.kp, params_.kd, params_.ki, params_.max_acceleration);
  pid_z_.setParams(params_.kp, params_.kd, params_.ki, params_.max_acceleration);
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

#endif // VELOCITY_CONTROLLER_IMPL_H
