#include <mrs_multirotor_simulator/controllers/attitude_controller.h>

namespace mrs_multirotor_simulator
{

// constructor
AttitudeController::AttitudeController() {
  initializePIDS();
}

AttitudeController::AttitudeController(const ModelParams& model_params) {
  model_params_ = model_params;
  initializePIDS();
}

void AttitudeController::setParams(const Params& params) {

  params_ = params;

  initializePIDS();
}

void AttitudeController::initializePIDS(void) {

  pid_x_.reset();
  pid_y_.reset();
  pid_z_.reset();

  pid_x_.setParams(params_.kp * model_params_.mass, params_.kd * model_params_.mass, params_.ki * model_params_.mass, params_.max_rate_roll_pitch);
  pid_y_.setParams(params_.kp * model_params_.mass, params_.kd * model_params_.mass, params_.ki * model_params_.mass, params_.max_rate_roll_pitch);
  pid_z_.setParams(params_.kp * model_params_.mass, params_.kd * model_params_.mass, params_.ki * model_params_.mass, params_.max_rate_yaw);
}

reference::AttitudeRate AttitudeController::getControlSignal(const MultirotorModel::State& state, const reference::Attitude& reference, const double& dt) {

  // orientation error
  Eigen::Matrix3d R_error = 0.5 * (reference.orientation.transpose() * state.R - state.R.transpose() * reference.orientation);

  // vectorize the orientation error
  // clang-format off
  Eigen::Vector3d R_error_vec;
  R_error_vec << (R_error(1, 2) - R_error(2, 1)) / 2.0,
                 (R_error(2, 0) - R_error(0, 2)) / 2.0,
                 (R_error(0, 1) - R_error(1, 0)) / 2.0;
  // clang-format on

  reference::AttitudeRate output;

  output.rate_x   = pid_x_.update(R_error_vec(0), dt);
  output.rate_y   = pid_y_.update(R_error_vec(1), dt);
  output.rate_z   = pid_z_.update(R_error_vec(2), dt);
  output.throttle = reference.throttle;

  return output;
}

}  // namespace mrs_multirotor_simulator
