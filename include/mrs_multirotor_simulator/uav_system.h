#ifndef UAV_SYSTEM_H
#define UAV_SYSTEM_H

#include <mrs_multirotor_simulator/multirotor_model.h>

#include <mrs_multirotor_simulator/controllers/mixer.h>
#include <mrs_multirotor_simulator/controllers/rate_controller.h>
#include <mrs_multirotor_simulator/controllers/attitude_controller.h>
#include <mrs_multirotor_simulator/controllers/acceleration_controller.h>
#include <mrs_multirotor_simulator/controllers/velocity_controller.h>
#include <mrs_multirotor_simulator/controllers/position_controller.h>

namespace mrs_multirotor_simulator
{

class UavSystem {

public:
  enum INPUT_MODE
  {
    INPUT_UNKNOWN     = 0,
    ACTUATOR_CMD      = 1,
    CONTROL_GROUP_CMD = 2,
    ATTITUDE_RATE_CMD = 3,
    ATTITUDE_CMD      = 4,
    ACCELERATION_CMD  = 5,
    VELOCITY_CMD      = 6,
    POSITION_CMD      = 7,
  };

  UavSystem(void);
  UavSystem(const ModelParams& model_params);
  UavSystem(const ModelParams& model_params, const Eigen::Vector4d spawn);

  void makeStep(const double dt);

  void setInput(const reference::Actuators& actuators);
  void setInput(const reference::ControlGroup& control_group);
  void setInput(const reference::AttitudeRate& attitude_rate);
  void setInput(const reference::Attitude& attitude);
  void setInput(const reference::Acceleration& acceleration);
  void setInput(const reference::Velocity& velocity);
  void setInput(const reference::Position& position);
  void setInput(void);

  MultirotorModel::State getState(void);

  Eigen::Vector3d getImuAcceleration(void);

  void setMixerParams(const Mixer::Params& params);
  void setRateControllerParams(const RateController::Params& params);
  void setAttitudeControllerParams(const AttitudeController::Params& params);
  void setVelocityControllerParams(const VelocityController::Params& params);
  void setPositionControllerParams(const PositionController::Params& params);

  Eigen::MatrixXd getMixerAllocation(void);

private:
  // | ------------------------- basics ------------------------- |

  bool initialized_ = false;

  // | --------------------- dynamics model --------------------- |

  MultirotorModel multirotor_model_;

  // | ----------------------- controllers ---------------------- |

  Mixer                  mixer_;
  RateController         rate_controller_;
  AttitudeController     attitude_controller_;
  AccelerationController acceleration_controller_;
  VelocityController     velocity_controller_;
  PositionController     position_controller_;

  INPUT_MODE active_input_ = INPUT_UNKNOWN;

  reference::Actuators    actuators_cmd_;
  reference::ControlGroup control_group_cmd_;
  reference::AttitudeRate attitude_rate_cmd_;
  reference::Attitude     attitude_cmd_;
  reference::Acceleration acceleration_cmd_;
  reference::Velocity     velocity_cmd_;
  reference::Position     position_cmd_;

  void initializeControllers(void);
};

}  // namespace mrs_multirotor_simulator

#endif  // UAV_SYSTEM_H