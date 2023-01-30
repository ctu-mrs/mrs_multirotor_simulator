#include <controllers/velocity_controller.h>

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

  pid_x_.setParams(params.kp, params.kd, params.ki);
  pid_y_.setParams(params.kp, params.kd, params.ki);
  pid_z_.setParams(params.kp, params.kd, params.ki);
}

reference::Acceleration VelocityController::getControlSignal(const QuadrotorModel::State& state, const reference::Velocity& reference, const double& dt) {

  // TODO

  reference::Acceleration output;

  return output;
}

}  // namespace mrs_multirotor_simulator
