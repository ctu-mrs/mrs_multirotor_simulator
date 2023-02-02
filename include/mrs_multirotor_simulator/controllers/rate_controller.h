#ifndef RATE_CONTROLLER_H
#define RATE_CONTROLLER_H

#include <eigen3/Eigen/Eigen>

#include <mrs_multirotor_simulator/multirotor_model.h>
#include <mrs_multirotor_simulator/controllers/pid.h>
#include <mrs_multirotor_simulator/controllers/references.h>

namespace mrs_multirotor_simulator
{

class RateController {

public:
  class Params {
  public:
    double kp = 4.0;
    double kd = 0.04;
    double ki = 0.0;
  };

  RateController();
  RateController(const ModelParams& model_params);

  void setParams(const Params& params);

  reference::ControlGroup getControlSignal(const MultirotorModel::State& state, const reference::AttitudeRate& reference, const double& dt);

private:
  ModelParams model_params_;
  Params      params_;

  void initializePIDs(void);

  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_z_;
};

}  // namespace mrs_multirotor_simulator

#endif  // RATE_CONTROLLER_H
