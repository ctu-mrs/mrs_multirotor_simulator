/**
 * @brief Dynamic simulation of a multirotor helicopter.
 *
 * Acknowledgement:
 * * https://github.com/HKUST-Aerial-Robotics/Fast-Planner
 */
#include <cmath>
#include <Eigen/Dense>

extern "C" {
#include "simulink_planner/FlightMechanicsModelForMRS.h"
}
#ifndef FIXEDWING_MODEL_H
#define FIXEDWING_MODEL_H

// namespace mrs_fixedwing_simulator 
namespace mrs_multirotor_simulator 
{

class FixedwingModel {
public:
  class ModelParams {

    public:
      ModelParams() {
        AoA_max = 15;     
        AoA_min = -10;      
        wing_area = 0.32;
        T2W_ratio = 0.6;
        cla = 5.5;
        g = 9.81;  
        m = 2.5;    
        pitch_tau = 0.05; 
        roll_tau = 0.05; 
        speed_tau = 0.3; 
        rho = 1.225;    
        v_max = 30;  
        v_stall = 10; 
      };

    real_T AoA_max;   
    real_T AoA_min;  
    real_T wing_area; 
    real_T T2W_ratio;
    real_T cla;  
    real_T g;  
    real_T m;
    real_T pitch_tau; 
    real_T roll_tau; 
    real_T speed_tau;
    real_T rho;      
    real_T v_max;   
    real_T v_stall; 

  };

  struct State
  {
    Eigen::Vector3d x;
    Eigen::Vector3d v;
    Eigen::Matrix3d R;
  };

  FixedwingModel();
  FixedwingModel(const ModelParams &params, const Eigen::Vector3d &spawn_pos, const Eigen::Vector3d &spawn_rot, const Eigen::Vector3d &spawn_vel);

  void setState(const FixedwingModel::State &state);
  void setStatePos(const Eigen::Vector3d &pos, const Eigen::Vector3d &spawn_rot); // teleport
  void setInput(const double &Pitch, const double &Roll, const double &Speed);
  void setParams(const ModelParams &params);

  const State &getState(void) const;
  ModelParams getParams(void);

  void step(const double &dt);

  void initializeState(void);

private:
  State state_;
  ModelParams params_;

  double time_accum_ = 0.0;
  const double simulink_dt_ = 0.001;
};

// --------------------------------------------------------------
// |                       implementation                       |
// --------------------------------------------------------------

/* constructor FixedwingModel //{ */

inline FixedwingModel::FixedwingModel(void) {
  initializeState();
}

inline FixedwingModel::FixedwingModel(const FixedwingModel::ModelParams &params, const Eigen::Vector3d &spawn_pos, const Eigen::Vector3d &spawn_rot, const Eigen::Vector3d &spawn_vel) {
  initializeState();
  params_ = params;

  state_.x = spawn_pos;
  state_.v = spawn_vel;
  state_.R = Eigen::AngleAxisd(spawn_rot(2), Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(spawn_rot(1), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(spawn_rot(0), Eigen::Vector3d::UnitX());

  FlightMechanicsModelForMRS_DW.XPositionState_DSTATE = spawn_pos(0);
  FlightMechanicsModelForMRS_DW.YPositionState_DSTATE = spawn_pos(1);
  FlightMechanicsModelForMRS_DW.AltitudeState_DSTATE  = -spawn_pos(2);

  FlightMechanicsModelForMRS_DW.RollAngleState_DSTATE   = spawn_rot(0);
  FlightMechanicsModelForMRS_DW.PitchAngleState_DSTATE  = spawn_rot(1);
  FlightMechanicsModelForMRS_DW.HeadingState_DSTATE     = spawn_rot(2);
}

//}

/* initializeState() //{ */

inline void FixedwingModel::initializeState(void) {

  state_.x      = Eigen::Vector3d::Zero();
  state_.v      = Eigen::Vector3d::Zero();
  state_.R      = Eigen::Matrix3d::Identity();

  setParams(params_);
  FlightMechanicsModelForMRS_initialize();

}

//}


/* step() //{ */

inline void FixedwingModel::step(const double &dt) {
  int num_steps = (dt + time_accum_) / simulink_dt_;
  for (int i = 0; i < num_steps; ++i) {
    FlightMechanicsModelForMRS_step(); 
  }
  time_accum_ = (dt + time_accum_) - (num_steps * simulink_dt_);

  state_.x = Eigen::Vector3d(
    FlightMechanicsModelForMRS_Y.States_X,
    FlightMechanicsModelForMRS_Y.States_Y,
    -FlightMechanicsModelForMRS_Y.States_Z 
  );

  state_.v = Eigen::Vector3d(
    FlightMechanicsModelForMRS_Y.States_VX,
    FlightMechanicsModelForMRS_Y.States_VY,
    -FlightMechanicsModelForMRS_Y.States_VZ
  );

  state_.R = Eigen::AngleAxisd(FlightMechanicsModelForMRS_Y.States_Heading * (M_PI / 180.0), Eigen::Vector3d::UnitZ()) 
           * Eigen::AngleAxisd(-FlightMechanicsModelForMRS_Y.States_Pitch * (M_PI / 180.0), Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(FlightMechanicsModelForMRS_Y.States_Roll * (M_PI / 180.0), Eigen::Vector3d::UnitX());
}

//}

// | ------------------- setters and getters ------------------ |

/* setParams() //{ */

inline void FixedwingModel::setParams(const FixedwingModel::ModelParams &params) {
  params_ = params;
  AircraftParameters.AoA_max = params_.AoA_max;
  AircraftParameters.AoA_min = params_.AoA_min;
  AircraftParameters.wing_area = params_.wing_area;
  AircraftParameters.T2W_ratio = params_.T2W_ratio;
  AircraftParameters.cla = params_.cla;
  AircraftParameters.g = params_.g;
  AircraftParameters.m = params_.m;
  AircraftParameters.pitch_tau = params_.pitch_tau;
  AircraftParameters.rho = params_.rho;
  AircraftParameters.roll_tau = params_.roll_tau;
  AircraftParameters.speed_tau = params_.speed_tau;
  AircraftParameters.v_max = params_.v_max; 
  AircraftParameters.v_stall = params_.v_stall;

}

//}

/* getParams() //{ */

inline FixedwingModel::ModelParams FixedwingModel::getParams(void) {

  return params_;
}

//}

/* setInput() //{ */

inline void FixedwingModel::setInput(const double &Pitch, const double &Roll, const double &Speed) { 

  FlightMechanicsModelForMRS_U.Commands_PitchRef = -Pitch* (180.0 / M_PI);
  FlightMechanicsModelForMRS_U.Commands_RollRef = Roll* (180.0 / M_PI);
  FlightMechanicsModelForMRS_U.Commands_VinfRef = Speed;

}

//}

/* getState() //{ */

inline const FixedwingModel::State &FixedwingModel::getState(void) const {
  return state_;
}

//}

/* setState() //{ */

inline void FixedwingModel::setState(const FixedwingModel::State &state) {

  FlightMechanicsModelForMRS_DW.XPositionState_DSTATE = state.x(0);
  FlightMechanicsModelForMRS_DW.YPositionState_DSTATE = state.x(1);
  FlightMechanicsModelForMRS_DW.AltitudeState_DSTATE = -state.x(2);

  FlightMechanicsModelForMRS_DW.SpeedState_DSTATE = state.v.norm();

  double heading = std::atan2(state.R(1, 0), state.R(0, 0));
  double pitch = std::atan2(-state.R(2, 0), std::sqrt(state.R(0, 0) * state.R(0, 0) + state.R(1, 0) * state.R(1, 0)));
  double roll = std::atan2(state.R(2, 1), state.R(2, 2));

  FlightMechanicsModelForMRS_DW.HeadingState_DSTATE = heading;
  FlightMechanicsModelForMRS_DW.PitchAngleState_DSTATE = pitch;
  FlightMechanicsModelForMRS_DW.RollAngleState_DSTATE = roll;

  state_.x = state.x;
  state_.v = state.v;
  state_.R = state.R;
}

//}

/* setStatePos() //{ */

inline void FixedwingModel::setStatePos(const Eigen::Vector3d &pos, const Eigen::Vector3d &spawn_rot) {

  FlightMechanicsModelForMRS_DW.XPositionState_DSTATE = pos(0);
  FlightMechanicsModelForMRS_DW.YPositionState_DSTATE = pos(1);
  FlightMechanicsModelForMRS_DW.AltitudeState_DSTATE  = -pos(2);

  FlightMechanicsModelForMRS_DW.RollAngleState_DSTATE   = spawn_rot(0);
  FlightMechanicsModelForMRS_DW.PitchAngleState_DSTATE  = spawn_rot(1);
  FlightMechanicsModelForMRS_DW.HeadingState_DSTATE     = spawn_rot(2);
  FlightMechanicsModelForMRS_DW.FlypathAngleState_DSTATE = 0.0; 
  FlightMechanicsModelForMRS_DW.SpeedState_DSTATE = 30.0;

  state_.x = pos;
  state_.R = Eigen::AngleAxisd(spawn_rot(2), Eigen::Vector3d::UnitZ())
           * Eigen::AngleAxisd(spawn_rot(1), Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(spawn_rot(0), Eigen::Vector3d::UnitX());
}

//}

} // namespace mrs_fixedwing_simulator

#endif
