#ifndef ACCELERATION_CONTROLLER_H
#define ACCELERATION_CONTROLLER_H

#include <quadrotor_model.h>
#include <controllers/references.h>

namespace mrs_multirotor_simulator
{

class AccelerationController {

public:
  struct Params
  {
    double g;
    double mass;
    double kf;
    double max_rpm;
    double min_rpm;
    double n_motors;
  };

  AccelerationController();

  void setParams(const Params& params);

  reference::Attitude getControlSignal(const QuadrotorModel::State& state, const reference::Acceleration& reference, const double dt);

private:
  Params params_;
};

}  // namespace mrs_multirotor_simulator

#endif  // ACCELERATION_CONTROLLER_H
