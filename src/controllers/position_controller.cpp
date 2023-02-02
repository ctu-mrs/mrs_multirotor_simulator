#include <mrs_multirotor_simulator/controllers/position_controller.h>

namespace mrs_multirotor_simulator
{

// constructor
PositionController::PositionController() {

  initializePIDs();
}

PositionController::PositionController(const ModelParams& model_params) {

  model_params_ = model_params;

  initializePIDs();
}

void PositionController::setParams(const Params& params) {

  params_ = params;

  initializePIDs();
}

void PositionController::initializePIDs(void) {

  pid_x_.reset();
  pid_y_.reset();
  pid_z_.reset();

  pid_x_.setParams(params_.kp, params_.kd, params_.ki, params_.max_velocity);
  pid_y_.setParams(params_.kp, params_.kd, params_.ki, params_.max_velocity);
  pid_z_.setParams(params_.kp, params_.kd, params_.ki, params_.max_velocity);
}

reference::Velocity PositionController::getControlSignal(const MultirotorModel::State& state, const reference::Position& reference, const double& dt) {

  Eigen::Vector3d pos_error = reference.position - state.x;

  reference::Velocity output;

  output.velocity[0] = pid_x_.update(pos_error[0], dt);
  output.velocity[1] = pid_y_.update(pos_error[1], dt);
  output.velocity[2] = pid_z_.update(pos_error[2], dt);

  output.heading = reference.heading;

  return output;
}

}  // namespace mrs_multirotor_simulator
