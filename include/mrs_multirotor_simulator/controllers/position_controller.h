#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <mrs_multirotor_simulator/multirotor_model.h>
#include <mrs_multirotor_simulator/controllers/pid.h>
#include <mrs_multirotor_simulator/controllers/references.h>

namespace mrs_multirotor_simulator
{

class PositionController {

public:
  class Params {
  public:
    double kp           = 1.0;
    double kd           = 0.5;
    double ki           = 0.01;
    double max_velocity = 4.0;  // m/s
  };

  PositionController();
  PositionController(const ModelParams& model_params);

  void setParams(const Params& params);

  reference::Velocity getControlSignal(const MultirotorModel::State& state, const reference::Position& reference, const double& dt);

private:
  ModelParams model_params_;
  Params      params_;

  void initializePIDs(void);

  PIDController pid_x_;
  PIDController pid_y_;
  PIDController pid_z_;
};

}  // namespace mrs_multirotor_simulator

#endif  // POSITION_CONTROLLER_H
