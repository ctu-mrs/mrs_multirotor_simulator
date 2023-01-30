/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <quadrotor_model.h>
#include <controllers/mixer.h>
#include <controllers/rate_controller.h>
#include <controllers/attitude_controller.h>
#include <controllers/acceleration_controller.h>
#include <controllers/velocity_controller.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <std_srvs/SetBool.h>

#include <mrs_msgs/HwApiAttitudeRateCmd.h>
#include <mrs_msgs/HwApiAttitudeCmd.h>
#include <mrs_msgs/HwApiAccelerationCmd.h>
#include <mrs_msgs/HwApiVelocityCmd.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/subscribe_handler.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_multirotor_simulator/multirotor_simulatorConfig.h>

#include <mrs_multirotor_simulator/Diagnostics.h>

//}

namespace mrs_multirotor_simulator
{

/* class MultirotorSimulator //{ */

class MultirotorSimulator : public nodelet::Nodelet {

public:
  virtual void onInit();

  enum INPUT_MODE
  {
    INPUT_TIMEOUT     = 0,
    ACTUATOR_CMD      = 1,
    CONTROL_GROUP_CMD = 2,
    ATTITUDE_RATE_CMD = 3,
    ATTITUDE_CMD      = 4,
    ACCELERATION_CMD  = 5,
    VELOCITY_CMD      = 6,
    POSITION_CMD      = 7,
  };

private:
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_ = false;

  // | ------------------------- params ------------------------- |

  ModelParams_t model_params_;

  Eigen::MatrixXd mixing_matrix;

  double _simulation_rate_;

  ros::Time sim_time_;

  bool _iterate_without_input_;

  double _input_timeout_;

  double _spawn_x_;
  double _spawn_y_;
  double _spawn_z_;

  // | --------------------- dynamics model --------------------- |

  std::unique_ptr<QuadrotorModel> quadrotor_model_;

  // | ----------------------- controllers ---------------------- |

  Mixer                  mixer_;
  RateController         rate_controller_;
  AttitudeController     attitude_controller_;
  AccelerationController acceleration_controller_;
  VelocityController     velocity_controller_;

  // | ------------------------- timers ------------------------- |

  ros::WallTimer timer_main_;
  void           timerMain(const ros::WallTimerEvent& event);

  ros::WallTimer timer_diagnostics_;
  void           timerDiagnostics(const ros::WallTimerEvent& event);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<sensor_msgs::Imu>                      ph_imu_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry>                    ph_odom_;
  mrs_lib::PublisherHandler<rosgraph_msgs::Clock>                  ph_clock_;
  mrs_lib::PublisherHandler<mrs_multirotor_simulator::Diagnostics> ph_diagnostics_;

  // | ----------------------- subscribers ---------------------- |

  void callbackAttitudeRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp);
  void callbackAttitudeCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>& wrp);
  void callbackAccelerationCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationCmd>& wrp);
  void callbackVelocityCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityCmd>& wrp);

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd> sh_attitude_rate_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>     sh_attitude_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationCmd> sh_acceleration_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityCmd>     sh_velocity_cmd_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                       mutex_drs_;
  typedef mrs_multirotor_simulator::multirotor_simulatorConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>             Drs_t;
  boost::shared_ptr<Drs_t>                                     drs_;
  void                                                         callbackDrs(mrs_multirotor_simulator::multirotor_simulatorConfig& config, uint32_t level);
  DrsConfig_t                                                  drs_params_;
  std::mutex                                                   mutex_drs_params_;

  // | ------------------------- methods ------------------------ |

  MultirotorSimulator::INPUT_MODE getInputMode(void);

  reference::Actuators    getLastActuatorCmd(void);
  reference::ControlGroup getLastControlGroupCmd(void);
  reference::AttitudeRate getLastAttitudeRateCmd(void);
  reference::Attitude     getLastAttitudeCmd(void);
  reference::Acceleration getLastAccelerationCmd(void);
  reference::Velocity     getLastVelocityCmd(void);
};

//}

/* onInit() //{ */

void MultirotorSimulator::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  if (!(nh_.hasParam("/use_sim_time"))) {
    nh_.setParam("/use_sim_time", true);
  }

  sim_time_ = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh_, "MultirotorSimulator");

  param_loader.loadParam("simulation_rate", _simulation_rate_);
  param_loader.loadParam("realtime_factor", drs_params_.realtime_factor);
  param_loader.loadParam("iterate_without_input", _iterate_without_input_);
  param_loader.loadParam("input_timeout", _input_timeout_);

  param_loader.loadParam("spawn_location/x", _spawn_x_);
  param_loader.loadParam("spawn_location/y", _spawn_y_);
  param_loader.loadParam("spawn_location/z", _spawn_z_);

  param_loader.loadParam("n_motors", model_params_.n_motors);
  param_loader.loadParam("mass", model_params_.mass);
  param_loader.loadParam("arm_length", model_params_.arm_length);
  param_loader.loadParam("body_height", model_params_.body_height);
  param_loader.loadParam("motor_time_constant", model_params_.motor_time_constant);
  param_loader.loadParam("propulsion/prop_radius", model_params_.prop_radius);
  param_loader.loadParam("propulsion/force_constant", model_params_.kf);
  param_loader.loadParam("g", model_params_.g);
  param_loader.loadParam("rpm/min", model_params_.min_rpm);
  param_loader.loadParam("rpm/max", model_params_.max_rpm);
  param_loader.loadParam("air_resistance_coeff", model_params_.air_resistance_coeff);

  param_loader.loadParam("ground/enabled", model_params_.ground_enabled);
  param_loader.loadParam("ground/z", model_params_.ground_z);

  param_loader.loadParam("takeoff_patch/enabled", model_params_.takeoff_patch_enabled);

  // create the inertia matrix
  model_params_.J = Eigen::Matrix3d::Zero();
  model_params_.J(0, 0) =
      model_params_.mass * (3.0 * model_params_.arm_length * model_params_.arm_length + model_params_.body_height * model_params_.body_height) / 12.0;
  model_params_.J(1, 1) =
      model_params_.mass * (3.0 * model_params_.arm_length * model_params_.arm_length + model_params_.body_height * model_params_.body_height) / 12.0;
  model_params_.J(2, 2) = (model_params_.mass * model_params_.arm_length * model_params_.arm_length) / 2.0;

  model_params_.allocation_matrix = param_loader.loadMatrixDynamic2("propulsion/allocation_matrix", 4, -1);

  model_params_.mixing_matrix = model_params_.allocation_matrix;

  double moment_constant;
  param_loader.loadParam("propulsion/moment_constant", moment_constant);

  model_params_.mixing_matrix.row(0) = model_params_.mixing_matrix.row(0) * model_params_.arm_length;
  model_params_.mixing_matrix.row(1) = model_params_.mixing_matrix.row(1) * model_params_.arm_length;

  const double km = moment_constant * (3.0 * model_params_.prop_radius);

  model_params_.mixing_matrix.row(2) = model_params_.mixing_matrix.row(2) * km;

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ControlManager]: could not load all parameters!");
    ros::shutdown();
  }

  quadrotor_model_ = std::make_unique<QuadrotorModel>(model_params_, Eigen::Vector3d(_spawn_x_, _spawn_y_, _spawn_z_));

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&MultirotorSimulator::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | -------------------------- mixer ------------------------- |

  Mixer::Params mixer_params;

  mixer_params.n_motors = model_params_.n_motors;
  param_loader.loadParam("mixer/desaturation", mixer_params.desaturation);

  mixer_params.allocation_matrix = model_params_.allocation_matrix;

  mixer_.setParams(mixer_params);

  // | --------------------- rate controller -------------------- |

  RateController::Params rate_controller_params;

  rate_controller_params.mass = model_params_.mass;

  param_loader.loadParam("rate_controller/kp", rate_controller_params.kp);
  param_loader.loadParam("rate_controller/kd", rate_controller_params.kd);
  param_loader.loadParam("rate_controller/ki", rate_controller_params.ki);

  rate_controller_.setParams(rate_controller_params);

  // | --------------------- attitude controller -------------------- |

  AttitudeController::Params attitude_controller_params;

  attitude_controller_params.mass = model_params_.mass;

  param_loader.loadParam("attitude_controller/kp", attitude_controller_params.kp);
  param_loader.loadParam("attitude_controller/kd", attitude_controller_params.kd);
  param_loader.loadParam("attitude_controller/ki", attitude_controller_params.ki);

  attitude_controller_.setParams(attitude_controller_params);

  // | ----------------- acceleration controller ---------------- |

  AccelerationController::Params acceleration_controller_params;

  acceleration_controller_params.mass     = model_params_.mass;
  acceleration_controller_params.g        = model_params_.g;
  acceleration_controller_params.kf       = model_params_.kf;
  acceleration_controller_params.max_rpm  = model_params_.max_rpm;
  acceleration_controller_params.min_rpm  = model_params_.min_rpm;
  acceleration_controller_params.n_motors = model_params_.n_motors;

  acceleration_controller_.setParams(acceleration_controller_params);

  // | ------------------- velocity controller ------------------ |

  VelocityController::Params velocity_controller_params;

  param_loader.loadParam("velocity_controller/kp", velocity_controller_params.kp);
  param_loader.loadParam("velocity_controller/kd", velocity_controller_params.kd);
  param_loader.loadParam("velocity_controller/ki", velocity_controller_params.ki);

  velocity_controller_.setParams(velocity_controller_params);

  // | ----------------------- publishers ----------------------- |

  ph_imu_         = mrs_lib::PublisherHandler<sensor_msgs::Imu>(nh_, "imu_out", 1, false);
  ph_odom_        = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, "odom_out", 1, false);
  ph_clock_       = mrs_lib::PublisherHandler<rosgraph_msgs::Clock>(nh_, "clock_out", 10, false);
  ph_diagnostics_ = mrs_lib::PublisherHandler<mrs_multirotor_simulator::Diagnostics>(nh_, "diagnostics_out", 10, true);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "MultirotorSimulator";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_attitude_rate_cmd_ =
      mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>(shopts, "attitude_rate_cmd_in", &MultirotorSimulator::callbackAttitudeRateCmd, this);

  sh_attitude_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>(shopts, "attitude_cmd_in", &MultirotorSimulator::callbackAttitudeCmd, this);

  sh_acceleration_cmd_ =
      mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationCmd>(shopts, "acceleration_cmd_in", &MultirotorSimulator::callbackAccelerationCmd, this);

  sh_velocity_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityCmd>(shopts, "velocity_cmd_in", &MultirotorSimulator::callbackVelocityCmd, this);

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createWallTimer(ros::WallDuration(1.0 / (_simulation_rate_ * drs_params_.realtime_factor)), &MultirotorSimulator::timerMain, this);

  timer_diagnostics_ = nh_.createWallTimer(ros::WallDuration(1.0 / 10.0), &MultirotorSimulator::timerDiagnostics, this);

  // | ------------------ first model iteration ----------------- |

  // * we need to iterate the model first to initialize its state
  // * this needs to happen in order to publish the correct state
  //   when using iterate_without_input == false

  reference::Actuators actuators_cmd;

  actuators_cmd.motors = Eigen::VectorXd::Zero(model_params_.n_motors);

  // set the motor input for the model
  quadrotor_model_->setInput(actuators_cmd);

  // iterate the model twise to initialize all the states
  quadrotor_model_->step(1.0 / _simulation_rate_);
  quadrotor_model_->step(1.0 / _simulation_rate_);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[MultirotorSimulator]: initialized");
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void MultirotorSimulator::timerMain([[maybe_unused]] const ros::WallTimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MultirotorSimulator]: main timer spinning");

  double simulation_step_size = 1.0 / _simulation_rate_;

  // | -------------- prepare the control reference ------------- |

  MultirotorSimulator::INPUT_MODE input_mode = getInputMode();

  reference::Velocity     velocity_cmd      = getLastVelocityCmd();
  reference::Acceleration acceleration_cmd  = getLastAccelerationCmd();
  reference::Attitude     attitude_cmd      = getLastAttitudeCmd();
  reference::AttitudeRate attitude_rate_cmd = getLastAttitudeRateCmd();
  reference::ControlGroup control_group_cmd = getLastControlGroupCmd();
  reference::Actuators    actuators_cmd     = getLastActuatorCmd();

  if (input_mode >= MultirotorSimulator::VELOCITY_CMD) {
    acceleration_cmd = velocity_controller_.getControlSignal(quadrotor_model_->getState(), velocity_cmd, simulation_step_size);
  }

  if (input_mode >= MultirotorSimulator::ACCELERATION_CMD) {
    attitude_cmd = acceleration_controller_.getControlSignal(quadrotor_model_->getState(), acceleration_cmd, simulation_step_size);
  }

  if (input_mode >= MultirotorSimulator::ATTITUDE_CMD) {
    attitude_rate_cmd = attitude_controller_.getControlSignal(quadrotor_model_->getState(), attitude_cmd, simulation_step_size);
  }

  if (input_mode >= MultirotorSimulator::ATTITUDE_RATE_CMD) {
    control_group_cmd = rate_controller_.getControlSignal(quadrotor_model_->getState(), attitude_rate_cmd, simulation_step_size);
  }

  if (input_mode >= MultirotorSimulator::CONTROL_GROUP_CMD) {
    actuators_cmd = mixer_.getControlSignal(control_group_cmd);
  }

  // | --------------------- model iteration -------------------- |

  if (_iterate_without_input_ || input_mode != MultirotorSimulator::INPUT_TIMEOUT) {

    // set the motor input for the model
    quadrotor_model_->setInput(actuators_cmd);

    // iterate the model
    quadrotor_model_->step(simulation_step_size);
  }

  // extract the current state
  auto state = quadrotor_model_->getState();

  // step the time
  sim_time_ = sim_time_ + ros::Duration(simulation_step_size);

  // | ---------------------- publish time ---------------------- |

  rosgraph_msgs::Clock ros_time;

  ros_time.clock.fromSec(sim_time_.toSec());

  ph_clock_.publish(ros_time);

  // | -------------------- publish odometry -------------------- |

  nav_msgs::Odometry odom;

  odom.header.stamp    = ros::Time::now();
  odom.header.frame_id = "sim_world";
  odom.child_frame_id  = "body";

  odom.pose.pose.orientation = mrs_lib::AttitudeConverter(state.R);

  odom.pose.pose.position.x = state.x[0];
  odom.pose.pose.position.y = state.x[1];
  odom.pose.pose.position.z = state.x[2];

  Eigen::Vector3d vel_body = state.R.transpose() * state.v;

  odom.twist.twist.linear.x = vel_body[0];
  odom.twist.twist.linear.y = vel_body[1];
  odom.twist.twist.linear.z = vel_body[2];

  odom.twist.twist.angular.x = state.omega[0];
  odom.twist.twist.angular.y = state.omega[1];
  odom.twist.twist.angular.z = state.omega[2];

  ph_odom_.publish(odom);

  // | ----------------------- publish IMU ---------------------- |

  sensor_msgs::Imu imu;

  imu.header.stamp    = ros::Time::now();
  imu.header.frame_id = "body";

  imu.angular_velocity.x = state.omega[0];
  imu.angular_velocity.y = state.omega[1];
  imu.angular_velocity.z = state.omega[2];

  auto acc = quadrotor_model_->getAcc();

  imu.linear_acceleration.x = acc[0];
  imu.linear_acceleration.y = acc[1];
  imu.linear_acceleration.z = acc[2];

  ph_imu_.publish(imu);
}

//}

/* timerDiagnostics() //{ */

void MultirotorSimulator::timerDiagnostics([[maybe_unused]] const ros::WallTimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MultirotorSimulator]: diagnostics timer spinning");

  mrs_multirotor_simulator::Diagnostics msg;

  ph_diagnostics_.publish(msg);
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackVelocityCmd() //{ */

void MultirotorSimulator::callbackVelocityCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MultirotorSimulator]: getting velocity command");
}

//}

/* callbackAccelerationCmd() //{ */

void MultirotorSimulator::callbackAccelerationCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MultirotorSimulator]: getting acceleration command");
}

//}

/* callbackAttitudeCmd() //{ */

void MultirotorSimulator::callbackAttitudeCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MultirotorSimulator]: getting attitude command");
}

//}

/* callbackAttitudeRateCmd() //{ */

void MultirotorSimulator::callbackAttitudeRateCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MultirotorSimulator]: getting attitude rate command");
}

//}

/* callbackDrs() //{ */

void MultirotorSimulator::callbackDrs(mrs_multirotor_simulator::multirotor_simulatorConfig& config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_drs_params_);

    drs_params_ = config;
  }

  timer_main_.setPeriod(ros::WallDuration(1.0 / (_simulation_rate_ * config.realtime_factor)), true);

  ROS_INFO("[MultirotorSimulator]: DRS updated gains");
}

//}

// | ------------------------- methods ------------------------ |

/* getLastActuatorCmd() //{ */

reference::Actuators MultirotorSimulator::getLastActuatorCmd(void) {

  reference::Actuators cmd;

  cmd.motors = Eigen::VectorXd::Zero(model_params_.n_motors);

  return cmd;
}

//}

/* getLastControlGroupCmd() //{ */

reference::ControlGroup MultirotorSimulator::getLastControlGroupCmd(void) {

  reference::ControlGroup cmd;

  // default values
  cmd.roll     = 0;
  cmd.pitch    = 0;
  cmd.yaw      = 0;
  cmd.throttle = 0;

  return cmd;
}

//}

/* getLastAttitudeRateCmd() //{ */

reference::AttitudeRate MultirotorSimulator::getLastAttitudeRateCmd(void) {

  reference::AttitudeRate cmd;

  // default values
  cmd.throttle = 0.0;
  cmd.rate_x   = 0;
  cmd.rate_x   = 0;
  cmd.rate_x   = 0;

  if (!sh_attitude_rate_cmd_.hasMsg()) {
    return cmd;
  }

  if ((sh_attitude_rate_cmd_.lastMsgTime() - ros::Time::now()).toSec() > 0.25) {

    ROS_ERROR_THROTTLE(1.0, "[MultirotorSimulator]: attitude cmd timeouted");

    return cmd;
  }

  mrs_msgs::HwApiAttitudeRateCmd::ConstPtr msg = sh_attitude_rate_cmd_.getMsg();

  cmd.throttle = msg->throttle;
  cmd.rate_x   = msg->body_rate.x;
  cmd.rate_y   = msg->body_rate.y;
  cmd.rate_z   = msg->body_rate.z;

  return cmd;
}

//}

/* getLastAttitudeCmd() //{ */

reference::Attitude MultirotorSimulator::getLastAttitudeCmd(void) {

  reference::Attitude cmd;

  // default values
  cmd.throttle    = 0.0;
  cmd.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

  if (!sh_attitude_cmd_.hasMsg()) {
    return cmd;
  }

  if ((sh_attitude_cmd_.lastMsgTime() - ros::Time::now()).toSec() > 0.25) {

    ROS_ERROR_THROTTLE(1.0, "[MultirotorSimulator]: attitude cmd timeouted");

    return cmd;
  }

  mrs_msgs::HwApiAttitudeCmd::ConstPtr msg = sh_attitude_cmd_.getMsg();

  cmd.throttle    = msg->throttle;
  cmd.orientation = mrs_lib::AttitudeConverter(msg->orientation);

  return cmd;
}

//}

/* getLastAccelerationCmd() //{ */

reference::Acceleration MultirotorSimulator::getLastAccelerationCmd(void) {

  reference::Acceleration cmd;

  // default values
  cmd.heading      = 0.0;
  cmd.acceleration = Eigen::Vector3d::Zero();

  if (!sh_acceleration_cmd_.hasMsg()) {
    return cmd;
  }

  if ((sh_acceleration_cmd_.lastMsgTime() - ros::Time::now()).toSec() > 0.25) {

    ROS_ERROR_THROTTLE(1.0, "[MultirotorSimulator]: acceleration cmd timeouted");

    return cmd;
  }

  mrs_msgs::HwApiAccelerationCmd::ConstPtr msg = sh_acceleration_cmd_.getMsg();

  cmd.heading = msg->heading;

  cmd.acceleration[0] = msg->acceleration.x;
  cmd.acceleration[1] = msg->acceleration.y;
  cmd.acceleration[2] = msg->acceleration.z;

  return cmd;
}

//}

/* getLastVelocityCmd() //{ */

reference::Velocity MultirotorSimulator::getLastVelocityCmd(void) {

  reference::Velocity cmd;

  // default values
  cmd.heading  = 0.0;
  cmd.velocity = Eigen::Vector3d::Zero();

  if (!sh_velocity_cmd_.hasMsg()) {
    return cmd;
  }

  if ((sh_velocity_cmd_.lastMsgTime() - ros::Time::now()).toSec() > 0.25) {

    ROS_ERROR_THROTTLE(1.0, "[MultirotorSimulator]: velocity cmd timeouted");

    return cmd;
  }

  mrs_msgs::HwApiVelocityCmd::ConstPtr msg = sh_velocity_cmd_.getMsg();

  cmd.heading = msg->heading;

  cmd.velocity[0] = msg->velocity.x;
  cmd.velocity[1] = msg->velocity.y;
  cmd.velocity[2] = msg->velocity.z;

  return cmd;
}

//}

/* getInputType() //{ */

MultirotorSimulator::INPUT_MODE MultirotorSimulator::getInputMode(void) {

  const ros::Time now = ros::Time::now();

  const bool getting_attitude_rate = sh_attitude_rate_cmd_.hasMsg() && (now - sh_attitude_rate_cmd_.lastMsgTime()).toSec() <= _input_timeout_;
  const bool getting_attitude      = sh_attitude_cmd_.hasMsg() && (now - sh_attitude_cmd_.lastMsgTime()).toSec() <= _input_timeout_;
  const bool getting_acceleration  = sh_acceleration_cmd_.hasMsg() && (now - sh_acceleration_cmd_.lastMsgTime()).toSec() <= _input_timeout_;
  const bool getting_velocity      = sh_velocity_cmd_.hasMsg() && (now - sh_velocity_cmd_.lastMsgTime()).toSec() <= _input_timeout_;

  if (getting_attitude_rate) {

    return MultirotorSimulator::ATTITUDE_RATE_CMD;

  } else if (getting_attitude) {

    return MultirotorSimulator::ATTITUDE_CMD;

  } else if (getting_acceleration) {

    return MultirotorSimulator::ACCELERATION_CMD;

  } else if (getting_velocity) {

    return MultirotorSimulator::VELOCITY_CMD;

  } else {

    return MultirotorSimulator::INPUT_TIMEOUT;
  }
}

//}

}  // namespace mrs_multirotor_simulator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_multirotor_simulator::MultirotorSimulator, nodelet::Nodelet)
