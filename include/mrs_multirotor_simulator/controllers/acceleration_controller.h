#ifndef ACCELERATION_CONTROLLER_H
#define ACCELERATION_CONTROLLER_H

#include <mrs_multirotor_simulator/multirotor_model.h>
#include <mrs_multirotor_simulator/controllers/references.h>

namespace mrs_multirotor_simulator
{

class AccelerationController {

public:
  AccelerationController();
  AccelerationController(const ModelParams& model_params);

  reference::Attitude getControlSignal(const MultirotorModel::State& state, const reference::Acceleration& reference, const double dt);

private:
  ModelParams model_params_;
};

}  // namespace mrs_multirotor_simulator

#endif  // ACCELERATION_CONTROLLER_H
