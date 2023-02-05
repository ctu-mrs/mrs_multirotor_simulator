#ifndef ACCELERATION_CONTROLLER_H
#define ACCELERATION_CONTROLLER_H

#include "references.h"

namespace mrs_multirotor_simulator
{

class AccelerationController {

public:
  AccelerationController();
  AccelerationController(const MultirotorModel::ModelParams& model_params);

  reference::Attitude getControlSignal(const MultirotorModel::State& state, const reference::Acceleration& reference, const double dt);

private:
  MultirotorModel::ModelParams model_params_;
};

}  // namespace mrs_multirotor_simulator

#include "impl/acceleration_controller_impl.hpp"

#endif  // ACCELERATION_CONTROLLER_H
