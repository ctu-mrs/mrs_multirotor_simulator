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

  void setInput(const double pitch, const double roll, const double speed);

  FixedwingModel::State getState(void) const;
  FixedwingModel::ModelParams getParams(void);
  void setParams(const FixedwingModel::ModelParams &params);
  void setState(const FixedwingModel::State &state);
  void setStatePos(const Eigen::Vector3d &pos, const Eigen::Vector3d &spawn_rot);

private:
  // | ------------------------- basics ------------------------- |
  bool crashed_ = false;
  double target_pitch_ = 0.0;
  double target_roll_ = 0.0;
  double target_speed_ = 30.0;

  // | --------------------- dynamics model --------------------- |
  FixedwingModel fixedwing_model_;

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