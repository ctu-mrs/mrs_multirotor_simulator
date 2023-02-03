#ifndef UAV_SYSTEM_IMPL_H
#define UAV_SYSTEM_IMPL_H

namespace mrs_multirotor_simulator
{

/* UavSystem() //{ */

// constructor
UavSystem::UavSystem(void) {

  // the model params are supplied by the defaults from the header

  initializeControllers();
}

// constructor
UavSystem::UavSystem(const ModelParams& model_params) {

  multirotor_model_.setParams(model_params);

  initializeControllers();
}

UavSystem::UavSystem(const ModelParams& model_params, const Eigen::Vector3d spawn_pos, const double spawn_heading) {

  multirotor_model_.setParams(model_params);
  multirotor_model_.setStatePos(spawn_pos, spawn_heading);

  initializeControllers();
}

//}

/* initializeControllers() //{ */

void UavSystem::initializeControllers(void) {

  ModelParams model_params = multirotor_model_.getParams();

  mixer_                   = Mixer(model_params);
  rate_controller_         = RateController(model_params);
  attitude_controller_     = AttitudeController(model_params);
  acceleration_controller_ = AccelerationController(model_params);
  velocity_controller_     = VelocityController(model_params);
  position_controller_     = PositionController(model_params);
}

//}

/* setInput() //{ */

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

void UavSystem::setInput(void) {

  active_input_ = INPUT_UNKNOWN;
}

//}

/* crash() //{ */

void UavSystem::crash(void) {
  crashed_ = true;
}

//}

/* applyForce() //{ */

void UavSystem::applyForce(const Eigen::Vector3d& force) {

  multirotor_model_.applyForce(force);
}

//}

/* makeStep() //{ */

void UavSystem::makeStep(const double dt) {

  if (active_input_ >= UavSystem::POSITION_CMD) {
    velocity_cmd_ = position_controller_.getControlSignal(multirotor_model_.getState(), position_cmd_, dt);
  }

  if (active_input_ >= UavSystem::VELOCITY_CMD) {
    acceleration_cmd_ = velocity_controller_.getControlSignal(multirotor_model_.getState(), velocity_cmd_, dt);
  }

  if (active_input_ >= UavSystem::ACCELERATION_CMD) {
    attitude_cmd_ = acceleration_controller_.getControlSignal(multirotor_model_.getState(), acceleration_cmd_, dt);
  }

  if (active_input_ >= UavSystem::ATTITUDE_CMD) {
    attitude_rate_cmd_ = attitude_controller_.getControlSignal(multirotor_model_.getState(), attitude_cmd_, dt);
  }

  if (active_input_ >= UavSystem::ATTITUDE_RATE_CMD) {
    control_group_cmd_ = rate_controller_.getControlSignal(multirotor_model_.getState(), attitude_rate_cmd_, dt);
  }

  if (active_input_ >= UavSystem::CONTROL_GROUP_CMD) {
    actuators_cmd_ = mixer_.getControlSignal(control_group_cmd_);
  }

  if (crashed_ || active_input_ == UavSystem::INPUT_UNKNOWN) {
    actuators_cmd_.motors = Eigen::VectorXd::Zero(multirotor_model_.getParams().n_motors);
  }

  // set the motor input for the model
  multirotor_model_.setInput(actuators_cmd_);

  multirotor_model_.step(dt);
}

//}

/* getState() //{ */

MultirotorModel::State UavSystem::getState(void) {

  return multirotor_model_.getState();
}

//}

/* getMixerAllocation() //{ */

Eigen::MatrixXd UavSystem::getMixerAllocation(void) {

  return mixer_.getAllocationMatrix();
}

//}

/* getImuAcceleration() //{ */

Eigen::Vector3d UavSystem::getImuAcceleration(void) {

  return multirotor_model_.getImuAcceleration();
}

//}

/* setters for controllers' params //{ */

void UavSystem::setMixerParams(const Mixer::Params& params) {
  mixer_.setParams(params);
}

void UavSystem::setRateControllerParams(const RateController::Params& params) {
  rate_controller_.setParams(params);
}

void UavSystem::setAttitudeControllerParams(const AttitudeController::Params& params) {
  attitude_controller_.setParams(params);
}

void UavSystem::setVelocityControllerParams(const VelocityController::Params& params) {
  velocity_controller_.setParams(params);
}

void UavSystem::setPositionControllerParams(const PositionController::Params& params) {
  position_controller_.setParams(params);
}

//}


}  // namespace mrs_multirotor_simulator

#endif  // UAV_SYSTEM_IMPL_H
