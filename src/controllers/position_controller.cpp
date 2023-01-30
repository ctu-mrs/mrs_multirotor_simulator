#include <controllers/position_controller.h>

namespace mrs_multirotor_simulator
{

// constructor
PositionController::PositionController() {
}

void PositionController::setParams(const Params& params) {

  params_ = params;

  pid_x_.reset();
  pid_y_.reset();
  pid_z_.reset();

  pid_x_.setParams(params.kp, params.kd, params.ki, params.max_velocity);
  pid_y_.setParams(params.kp, params.kd, params.ki, params.max_velocity);
  pid_z_.setParams(params.kp, params.kd, params.ki, params.max_velocity);
}

reference::Velocity PositionController::getControlSignal(const QuadrotorModel::State& state, const reference::Position& reference, const double& dt) {

  Eigen::Vector3d pos_error = reference.position - state.x;

  reference::Velocity output;

  output.velocity[0] = pid_x_.update(pos_error[0], dt);
  output.velocity[1] = pid_y_.update(pos_error[1], dt);
  output.velocity[2] = pid_z_.update(pos_error[2], dt);

  output.heading = reference.heading;

  return output;
}

}  // namespace mrs_multirotor_simulator
