#include <mrs_multirotor_simulator/controllers/rate_controller.h>

namespace mrs_multirotor_simulator
{

// constructor
RateController::RateController() {
  initializePIDs();
}

RateController::RateController(const ModelParams& model_params) {
  model_params_ = model_params;
  initializePIDs();
}

void RateController::setParams(const Params& params) {

  params_ = params;

  initializePIDs();
}

void RateController::initializePIDs(void) {

  pid_x_.reset();
  pid_y_.reset();
  pid_z_.reset();

  pid_x_.setParams(params_.kp * model_params_.J(0, 0), params_.kd * model_params_.J(0, 0), params_.ki * model_params_.J(0, 0), 1.0);
  pid_y_.setParams(params_.kp * model_params_.J(1, 1), params_.kd * model_params_.J(1, 1), params_.ki * model_params_.J(1, 1), 1.0);
  pid_z_.setParams(params_.kp * model_params_.J(2, 2), params_.kd * model_params_.J(2, 2), params_.ki * model_params_.J(2, 2), 1.0);
}

reference::ControlGroup RateController::getControlSignal(const MultirotorModel::State& state, const reference::AttitudeRate& reference, const double& dt) {

  Eigen::Vector3d ang_rate_ref = Eigen::Vector3d(reference.rate_x, reference.rate_y, reference.rate_z);

  Eigen::Vector3d ang_rate_error = ang_rate_ref - state.omega;

  reference::ControlGroup output;

  output.roll     = pid_x_.update(ang_rate_error(0), dt);
  output.pitch    = pid_y_.update(ang_rate_error(1), dt);
  output.yaw      = pid_z_.update(ang_rate_error(2), dt);
  output.throttle = reference.throttle;

  return output;
}

}  // namespace mrs_multirotor_simulator
