/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_multirotor_simulator/uav_system.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <std_srvs/SetBool.h>

#include <mrs_msgs/HwApiAttitudeRateCmd.h>
#include <mrs_msgs/HwApiAttitudeCmd.h>
#include <mrs_msgs/HwApiAccelerationCmd.h>
#include <mrs_msgs/HwApiVelocityCmd.h>
#include <mrs_msgs/HwApiPositionCmd.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_multirotor_simulator/multirotor_simulatorConfig.h>

//}

namespace mrs_multirotor_simulator
{

/* class MultirotorSimulator //{ */

class MultirotorSimulator : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_;

  // | ------------------------ UavSystem ----------------------- |

  UavSystem  uav_system_;
  std::mutex mutex_uav_system_;

  ros::Time  time_last_input_;
  std::mutex mutex_time_last_input_;

  // | ------------------------- params ------------------------- |

  ModelParams model_params_;

  Eigen::MatrixXd mixing_matrix;

  double _simulation_rate_;

  ros::Time sim_time_;

  bool _iterate_without_input_;

  double _input_timeout_;

  double _spawn_x_;
  double _spawn_y_;
  double _spawn_z_;

  // | ------------------------- timers ------------------------- |

  ros::WallTimer timer_main_;
  void           timerMain(const ros::WallTimerEvent& event);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<sensor_msgs::Imu>     ph_imu_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry>   ph_odom_;
  mrs_lib::PublisherHandler<rosgraph_msgs::Clock> ph_clock_;

  // | ----------------------- subscribers ---------------------- |

  void callbackAttitudeRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp);
  void callbackAttitudeCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>& wrp);
  void callbackAccelerationCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationCmd>& wrp);
  void callbackVelocityCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityCmd>& wrp);
  void callbackPositionCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd>& wrp);

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd> sh_attitude_rate_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>     sh_attitude_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationCmd> sh_acceleration_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityCmd>     sh_velocity_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd>     sh_position_cmd_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                       mutex_drs_;
  typedef mrs_multirotor_simulator::multirotor_simulatorConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>             Drs_t;
  boost::shared_ptr<Drs_t>                                     drs_;
  void                                                         callbackDrs(mrs_multirotor_simulator::multirotor_simulatorConfig& config, uint32_t level);
  DrsConfig_t                                                  drs_params_;
  std::mutex                                                   mutex_drs_params_;
};

//}

/* onInit() //{ */

void MultirotorSimulator::onInit() {

  is_initialized_ = false;

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  if (!(nh_.hasParam("/use_sim_time"))) {
    nh_.setParam("/use_sim_time", true);
  }

  sim_time_        = ros::Time(0);
  time_last_input_ = ros::Time(0);

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
  param_loader.loadParam("propulsion/moment_constant", model_params_.km);
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

  model_params_.allocation_matrix.row(0) *= model_params_.arm_length * model_params_.kf;
  model_params_.allocation_matrix.row(1) *= model_params_.arm_length * model_params_.kf;
  model_params_.allocation_matrix.row(2) *= model_params_.km * (3.0 * model_params_.prop_radius) * model_params_.kf;
  model_params_.allocation_matrix.row(3) *= model_params_.kf;

  /* uav_system_ = UavSystem(model_params_, Eigen::Vector3d(_spawn_x_, _spawn_y_, _spawn_z_)); */
  uav_system_ = UavSystem();

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&MultirotorSimulator::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | -------------------------- mixer ------------------------- |

  Mixer::Params mixer_params;

  param_loader.loadParam("mixer/desaturation", mixer_params.desaturation);

  /* mixer_.setParams(mixer_params); */

  // | --------------------- rate controller -------------------- |

  RateController::Params rate_controller_params;

  param_loader.loadParam("rate_controller/kp", rate_controller_params.kp);
  param_loader.loadParam("rate_controller/kd", rate_controller_params.kd);
  param_loader.loadParam("rate_controller/ki", rate_controller_params.ki);

  /* rate_controller_.setParams(rate_controller_params); */

  // | --------------------- attitude controller -------------------- |

  AttitudeController::Params attitude_controller_params;

  param_loader.loadParam("attitude_controller/kp", attitude_controller_params.kp);
  param_loader.loadParam("attitude_controller/kd", attitude_controller_params.kd);
  param_loader.loadParam("attitude_controller/ki", attitude_controller_params.ki);
  param_loader.loadParam("attitude_controller/max_rate_roll_pitch", attitude_controller_params.max_rate_roll_pitch);
  param_loader.loadParam("attitude_controller/max_rate_yaw", attitude_controller_params.max_rate_yaw);

  /* attitude_controller_.setParams(attitude_controller_params); */

  // | ----------------- acceleration controller ---------------- |

  /* acceleration_controller_.setParams(acceleration_controller_params); */

  // | ------------------- velocity controller ------------------ |

  VelocityController::Params velocity_controller_params;

  param_loader.loadParam("velocity_controller/kp", velocity_controller_params.kp);
  param_loader.loadParam("velocity_controller/kd", velocity_controller_params.kd);
  param_loader.loadParam("velocity_controller/ki", velocity_controller_params.ki);
  param_loader.loadParam("velocity_controller/max_acceleration", velocity_controller_params.max_acceleration);

  /* velocity_controller_.setParams(velocity_controller_params); */

  // | ------------------- position controller ------------------ |

  PositionController::Params position_controller_params;

  param_loader.loadParam("position_controller/kp", position_controller_params.kp);
  param_loader.loadParam("position_controller/kd", position_controller_params.kd);
  param_loader.loadParam("position_controller/ki", position_controller_params.ki);
  param_loader.loadParam("position_controller/max_velocity", position_controller_params.max_velocity);

  /* position_controller_.setParams(position_controller_params); */

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MultirotorSimulator]: could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- publishers ----------------------- |

  ph_imu_   = mrs_lib::PublisherHandler<sensor_msgs::Imu>(nh_, "imu_out", 1, false);
  ph_odom_  = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, "odom_out", 1, false);
  ph_clock_ = mrs_lib::PublisherHandler<rosgraph_msgs::Clock>(nh_, "clock_out", 10, false);

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

  sh_position_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd>(shopts, "position_cmd_in", &MultirotorSimulator::callbackPositionCmd, this);

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createWallTimer(ros::WallDuration(1.0 / (_simulation_rate_ * drs_params_.realtime_factor)), &MultirotorSimulator::timerMain, this);

  // | ------------------ first model iteration ----------------- |

  // * we need to iterate the model first to initialize its state
  // * this needs to happen in order to publish the correct state
  //   when using iterate_without_input == false

  reference::Actuators actuators_cmd;

  actuators_cmd.motors = Eigen::VectorXd::Zero(model_params_.n_motors);

  // set the motor input for the model
  uav_system_.setInput(actuators_cmd);

  // iterate the model twise to initialize all the states
  uav_system_.makeStep(1.0 / _simulation_rate_);
  uav_system_.makeStep(1.0 / _simulation_rate_);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[MultirotorSimulator]: initialized");
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

  // | ---------------- check timeout of an input --------------- |

  auto time_last_input = mrs_lib::get_mutexed(mutex_time_last_input_, time_last_input_);

  if (time_last_input > ros::Time(0)) {
    if ((ros::Time::now() - time_last_input).toSec() > _input_timeout_) {
      ROS_WARN_THROTTLE(1.0, "[MultirotorSimulator]: input timeouted");

      {
        std::scoped_lock lock(mutex_uav_system_);
        uav_system_.setInput();
      }

      {
        std::scoped_lock lock(mutex_time_last_input_);
        time_last_input_ = ros::Time(0);
      }
    }
  }

  // | --------------------- model iteration -------------------- |

  if (_iterate_without_input_ || time_last_input_ > ros::Time(0)) {

    std::scoped_lock lock(mutex_uav_system_);

    // iterate the model
    uav_system_.makeStep(simulation_step_size);
  }

  // extract the current state
  MultirotorModel::State state = uav_system_.getState();

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

  try {
    odom.pose.pose.orientation = mrs_lib::AttitudeConverter(state.R);
  }
  catch (...) {
    ROS_ERROR("[AttitudeConverter]: internal orientation is invalid");
    ROS_INFO_STREAM("[MultirotorSimulator]: " << state.R);
  }

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

  auto acc = uav_system_.getImuAcceleration();

  imu.linear_acceleration.x = acc[0];
  imu.linear_acceleration.y = acc[1];
  imu.linear_acceleration.z = acc[2];

  ph_imu_.publish(imu);
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackAttitudeRateCmd() //{ */

void MultirotorSimulator::callbackAttitudeRateCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MultirotorSimulator]: getting attitude rate command");

  mrs_msgs::HwApiAttitudeRateCmd::ConstPtr msg = wrp.getMsg();

  reference::AttitudeRate cmd;

  cmd.throttle = msg->throttle;
  cmd.rate_x   = msg->body_rate.x;
  cmd.rate_y   = msg->body_rate.y;
  cmd.rate_z   = msg->body_rate.z;

  {
    std::scoped_lock lock(mutex_uav_system_);

    uav_system_.setInput(cmd);
  }

  {
    std::scoped_lock lock(mutex_time_last_input_);

    time_last_input_ = ros::Time::now();
  }
}

//}

/* callbackAttitudeCmd() //{ */

void MultirotorSimulator::callbackAttitudeCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MultirotorSimulator]: getting attitude command");

  mrs_msgs::HwApiAttitudeCmd::ConstPtr msg = wrp.getMsg();

  reference::Attitude cmd;

  cmd.throttle = msg->throttle;

  try {
    cmd.orientation = mrs_lib::AttitudeConverter(msg->orientation);
  }
  catch (...) {
    ROS_ERROR("[AttitudeConverter]: exception caught in callbackAttitude()");
    ROS_INFO_STREAM("[MultirotorSimulator]: " << msg->orientation);
  }

  {
    std::scoped_lock lock(mutex_uav_system_);

    uav_system_.setInput(cmd);
  }

  {
    std::scoped_lock lock(mutex_time_last_input_);

    time_last_input_ = ros::Time::now();
  }
}

//}

/* callbackAccelerationCmd() //{ */

void MultirotorSimulator::callbackAccelerationCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MultirotorSimulator]: getting acceleration command");

  mrs_msgs::HwApiAccelerationCmd::ConstPtr msg = wrp.getMsg();

  reference::Acceleration cmd;

  cmd.heading = msg->heading;

  cmd.acceleration[0] = msg->acceleration.x;
  cmd.acceleration[1] = msg->acceleration.y;
  cmd.acceleration[2] = msg->acceleration.z;

  {
    std::scoped_lock lock(mutex_uav_system_);

    uav_system_.setInput(cmd);
  }

  {
    std::scoped_lock lock(mutex_time_last_input_);

    time_last_input_ = ros::Time::now();
  }
}

//}

/* callbackVelocityCmd() //{ */

void MultirotorSimulator::callbackVelocityCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MultirotorSimulator]: getting velocity command");

  mrs_msgs::HwApiVelocityCmd::ConstPtr msg = wrp.getMsg();

  reference::Velocity cmd;

  cmd.heading = msg->heading;

  cmd.velocity[0] = msg->velocity.x;
  cmd.velocity[1] = msg->velocity.y;
  cmd.velocity[2] = msg->velocity.z;

  {
    std::scoped_lock lock(mutex_uav_system_);

    uav_system_.setInput(cmd);
  }

  {
    std::scoped_lock lock(mutex_time_last_input_);

    time_last_input_ = ros::Time::now();
  }
}

//}

/* callbackPositionCmd() //{ */

void MultirotorSimulator::callbackPositionCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MultirotorSimulator]: getting position command");

  mrs_msgs::HwApiPositionCmd::ConstPtr msg = wrp.getMsg();

  reference::Position cmd;

  cmd.heading = msg->heading;

  cmd.position[0] = msg->position.x;
  cmd.position[1] = msg->position.y;
  cmd.position[2] = msg->position.z;

  {
    std::scoped_lock lock(mutex_uav_system_);

    uav_system_.setInput(cmd);
  }

  {
    std::scoped_lock lock(mutex_time_last_input_);

    time_last_input_ = ros::Time::now();
  }
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

}  // namespace mrs_multirotor_simulator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_multirotor_simulator::MultirotorSimulator, nodelet::Nodelet)
