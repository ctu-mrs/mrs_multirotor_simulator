#include <mrs_multirotor_simulator/uav_system.h>

namespace mrs_multirotor_simulator
{

// constructor
UavSystem::UavSystem(void) {
}

// constructor
UavSystem::UavSystem(const ModelParams_t& model_params) {

  quadrotor_model_ = std::make_unique<MultirotorModel>(model_params, Eigen::Vector3d(0, 0, 0));

  initialized_ = true;
}

UavSystem::UavSystem(const ModelParams_t& model_params, const Eigen::Vector3d initial_position) {

  quadrotor_model_ = std::make_unique<MultirotorModel>(model_params, initial_position);

  initialized_ = true;
}

void UavSystem::setInput(const reference::Actuators& cmd) {

  actuators_cmd_ = cmd;

  active_input_ = ACTUATOR_CMD;
}

void UavSystem::setInput(const reference::ControlGroup& cmd) {

  control_group_cmd_ = cmd;

  active_input_ = CONTROL_GROUP_CMD;
}

void UavSystem::setInput(const reference::AttitudeRate& cmd) {

  attitude_rate_cmd_ = cmd;

  active_input_ = ATTITUDE_RATE_CMD;
}

void UavSystem::setInput(const reference::Attitude& cmd) {

  attitude_cmd_ = cmd;

  active_input_ = ATTITUDE_CMD;
}

void UavSystem::setInput(const reference::Acceleration& cmd) {

  acceleration_cmd_ = cmd;

  active_input_ = ACCELERATION_CMD;
}

void UavSystem::setInput(const reference::Velocity& cmd) {

  velocity_cmd_ = cmd;

  active_input_ = VELOCITY_CMD;
}

void UavSystem::setInput(const reference::Position& cmd) {

  position_cmd_ = cmd;

  active_input_ = POSITION_CMD;
}

void UavSystem::makeStep(const double dt) {

  quadrotor_model_->step(dt);
}

MultirotorModel::State UavSystem::getState(void) {

  return quadrotor_model_->getState();
}

Eigen::Vector3d UavSystem::getImuAcceleration(void) {

  return quadrotor_model_->getImuAcceleration();
}

}  // namespace mrs_multirotor_simulator

