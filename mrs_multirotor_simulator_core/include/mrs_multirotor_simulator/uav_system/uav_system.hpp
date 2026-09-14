#ifndef UAV_SYSTEM_H
#define UAV_SYSTEM_H

#include <cmath>
#include <optional>

#include "fixedwing_model.hpp"

// namespace mrs_fixedwing_simulator
namespace mrs_multirotor_simulator
{

class UavSystem {
public:
  UavSystem(void);
  UavSystem(const FixedwingModel::ModelParams &model_params);
  UavSystem(const FixedwingModel::ModelParams &model_params, const Eigen::Vector3d &spawn_pos, const Eigen::Vector3d &spawn_rot, const Eigen::Vector3d &spawn_vel);

  void makeStep(const double dt);
  void crash(void);
  bool hasCrashed(void);

  // void applyForce(const Eigen::Vector3d &force);

  void setInput(const double pitch, const double roll, const double speed);
  // void setInput(const reference::ControlGroup &control_group);
  // void setInput(const reference::AttitudeRate &attitude_rate);
  // void setInput(const reference::Attitude &attitude);
  // void setInput(const reference::TiltHdgRate &tilt);
  // void setInput(const reference::AccelerationHdgRate &acceleration);
  // void setInput(const reference::AccelerationHdg &acceleration);
  // void setInput(const reference::VelocityHdgRate &velocity);
  // void setInput(const reference::VelocityHdg &velocity);
  // void setInput(const reference::Position &position);
  // void setInput(void);

  // void setFeedforward(const reference::AccelerationHdgRate &cmd);
  // void setFeedforward(const reference::AccelerationHdg &cmd);
  // void setFeedforward(const reference::VelocityHdg &cmd);
  // void setFeedforward(const reference::VelocityHdgRate &cmd);

  FixedwingModel::State getState(void) const;
  FixedwingModel::ModelParams getParams(void);
  void setParams(const FixedwingModel::ModelParams &params);
  void setState(const FixedwingModel::State &state);
  void setStatePos(const Eigen::Vector3d &pos, const Eigen::Vector3d &spawn_rot);

  // void setMixerParams(const Mixer::Params &params);
  // void setRateControllerParams(const RateController::Params &params);
  // void setAttitudeControllerParams(const AttitudeController::Params &params);
  // void setVelocityControllerParams(const VelocityController::Params &params);
  // void setPositionControllerParams(const PositionController::Params &params);

  // Eigen::MatrixXd getMixerAllocation(void);

private:
  // | ------------------------- basics ------------------------- |
  bool crashed_ = false;
  double target_pitch_ = 0.0;
  double target_roll_ = 0.0;
  double target_speed_ = 30.0;

  // | --------------------- dynamics model --------------------- |
  FixedwingModel fixedwing_model_;

  // | ----------------------- controllers ---------------------- |
  // Mixer mixer_;
  // RateController rate_controller_;
  // AttitudeController attitude_controller_;
  // AccelerationController acceleration_controller_;
  // VelocityController velocity_controller_;
  // PositionController position_controller_;
  // INPUT_MODE active_input_ = INPUT_UNKNOWN;

  // | ------------- inputs for feedback controllers ------------ |
  // reference::Actuators actuators_cmd_;
  // reference::ControlGroup control_group_cmd_;
  // reference::AttitudeRate attitude_rate_cmd_;
  // reference::Attitude attitude_cmd_;
  // reference::TiltHdgRate tilt_hdg_rate_cmd_;
  // reference::AccelerationHdgRate acceleration_hdg_rate_cmd_;
  // reference::AccelerationHdg acceleration_hdg_cmd_;
  // reference::VelocityHdgRate velocity_hdg_rate_cmd_;
  // reference::VelocityHdg velocity_hdg_cmd_;
  // reference::Position position_cmd_;

  // | ------------------- feedforward inputs ------------------- |
  // std::optional<reference::VelocityHdgRate> velocity_hdg_rate_ff_;
  // std::optional<reference::VelocityHdg> velocity_hdg_ff_;
  // std::optional<reference::AccelerationHdgRate> acceleration_hdg_rate_ff_;
  // std::optional<reference::AccelerationHdg> acceleration_hdg_ff_;

  // void shiftPositionReference(const Eigen::Vector3d &translation, const double heading_delta);
  // static double extractHeading(const Eigen::Matrix3d &R);
  // static double wrapAngle(const double angle);
};

// --------------------------------------------------------------
// | implementation |
// --------------------------------------------------------------

/* UavSystem() //{ */

// constructor
inline UavSystem::UavSystem(void) {
  // the model params are supplied by the defaults from the header
}

// constructor
inline UavSystem::UavSystem(const FixedwingModel::ModelParams &model_params) {
  fixedwing_model_.setParams(model_params);
  fixedwing_model_.initializeState();
}

inline UavSystem::UavSystem(const FixedwingModel::ModelParams &model_params, const Eigen::Vector3d &spawn_pos, const Eigen::Vector3d &spawn_rot, const Eigen::Vector3d &spawn_vel) {
  fixedwing_model_.setParams(model_params);
  fixedwing_model_.initializeState();
  fixedwing_model_.setStatePos(spawn_pos, spawn_rot);
}

//}

inline void UavSystem::setInput(const double pitch, const double roll, const double speed) {
  target_pitch_ = pitch;
  target_roll_ = roll;
  target_speed_ = speed;
}

/* crash() //{ */

inline void UavSystem::crash(void) {
  crashed_ = true;
}

//}

/* hasCrashed() //{ */

inline bool UavSystem::hasCrashed(void) {
  return crashed_;
}

//}

/* makeStep() //{ */

inline void UavSystem::makeStep(const double dt) {
  fixedwing_model_.setInput(target_pitch_, target_roll_, target_speed_);
  fixedwing_model_.step(dt);
}

//}

/* getState() //{ */

inline FixedwingModel::State UavSystem::getState(void) const {
  return fixedwing_model_.getState();
}

//}

/* getParams() //{ */

inline FixedwingModel::ModelParams UavSystem::getParams(void) {
  return fixedwing_model_.getParams();
}

//}

/* setParams() //{ */

inline void UavSystem::setParams(const FixedwingModel::ModelParams &params) {
  fixedwing_model_.setParams(params);
}

//}

/* setState() //{ */

inline void UavSystem::setState(const FixedwingModel::State &state) {
  fixedwing_model_.setState(state);
}

//}

/* setStatePos() //{ */

inline void UavSystem::setStatePos(const Eigen::Vector3d &pos, const Eigen::Vector3d &spawn_rot) {
  fixedwing_model_.setStatePos(pos, spawn_rot);
}

//}


} // namespace mrs_fixedwing_simulator

#endif // UAV_SYSTEM_H