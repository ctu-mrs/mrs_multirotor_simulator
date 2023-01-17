/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <quadrotor_model.h>
#include <controllers/rate_controller.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>

#include <mrs_msgs/HwApiAttitudeRateCmd.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/subscribe_handler.h>

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
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

  // | ------------------------- params ------------------------- |

  ModelParams_t model_params_;

  Eigen::MatrixXd mixing_matrix;

  double _simulation_rate_;

  // | --------------------- dynamics model --------------------- |

  std::unique_ptr<QuadrotorModel> quadrotor_model_;
  RateController                  rate_controller_;

  ros::Time sim_time_;

  // | ------------------------- timers ------------------------- |

  ros::WallTimer timer_main_;
  void           timerMain(const ros::WallTimerEvent& event);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<sensor_msgs::Imu>     ph_imu_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry>   ph_odom_;
  mrs_lib::PublisherHandler<rosgraph_msgs::Clock> ph_clock_;

  // | ----------------------- subscribers ---------------------- |

  void callbackRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd> sh_rate_cmd_;

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

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  if (!(nh_.hasParam("/use_sim_time"))) {
    nh_.setParam("/use_sim_time", true);
  }

  sim_time_ = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh_, "MultirotorSimulator");

  param_loader.loadParam("simulation_rate", _simulation_rate_);
  param_loader.loadParam("realtime_factor", drs_params_.realtime_factor);

  param_loader.loadParam("n_motors", model_params_.n_motors);
  param_loader.loadParam("mass", model_params_.mass);
  param_loader.loadParam("arm_length", model_params_.arm_length);
  param_loader.loadParam("motor_time_constant", model_params_.motor_time_constant);
  param_loader.loadParam("propulsion/prop_radius", model_params_.prop_radius);
  param_loader.loadParam("propulsion/force_constant", model_params_.kf);
  param_loader.loadParam("g", model_params_.g);
  param_loader.loadParam("rpm/min", model_params_.min_rpm);
  param_loader.loadParam("rpm/max", model_params_.max_rpm);

  model_params_.J       = param_loader.loadMatrixStatic2<3, 3>("J");
  Eigen::MatrixXd mixer = param_loader.loadMatrixDynamic2("propulsion/mixing_matrix", 4, -1);

  model_params_.mixing_matrix = mixer;

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

  quadrotor_model_ = std::make_unique<QuadrotorModel>(model_params_);

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&MultirotorSimulator::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | --------------------- rate controller -------------------- |

  RateController::Params rate_controller_params;

  rate_controller_params.n_motors   = model_params_.n_motors;
  rate_controller_params.force_coef = model_params_.kf;
  rate_controller_params.max_rpm    = model_params_.max_rpm;
  rate_controller_params.min_rpm    = model_params_.min_rpm;

  param_loader.loadParam("rate_controller/kp", rate_controller_params.kp);
  param_loader.loadParam("rate_controller/kd", rate_controller_params.kd);
  param_loader.loadParam("rate_controller/ki", rate_controller_params.ki);

  rate_controller_params.allocation_matrix = model_params_.mixing_matrix;

  rate_controller_.setParams(rate_controller_params);

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

  sh_rate_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>(shopts, "rate_cmd_in", &MultirotorSimulator::callbackRateCmd, this);

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createWallTimer(ros::WallDuration(1.0 / (_simulation_rate_ * drs_params_.realtime_factor)), &MultirotorSimulator::timerMain, this);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[MultirotorSimulator]: initialized");
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void MultirotorSimulator::timerMain(const ros::WallTimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MultirotorSimulator]: main timer spinning");

  RateController::RateController::Reference reference;

  if (sh_rate_cmd_.hasMsg() && (ros::Time::now() - sh_rate_cmd_.lastMsgTime()).toSec() < 0.25) {

    auto cmd = sh_rate_cmd_.getMsg();

    reference.throttle        = cmd->throttle;
    reference.angular_rate(0) = cmd->body_rate.x;
    reference.angular_rate(1) = cmd->body_rate.y;
    reference.angular_rate(2) = cmd->body_rate.z;

  } else {

    ROS_WARN_THROTTLE(1.0, "[MultirotorSimulator]: not getting cmd");

    reference.throttle = 0;
    reference.angular_rate << 0, 0, 0;
  }

  Eigen::VectorXd input = rate_controller_.getControlSignal(quadrotor_model_->getState(), reference, 0.01);

  quadrotor_model_->setInput(input);

  const double step = 1.0 / _simulation_rate_;

  quadrotor_model_->step(step);

  auto state = quadrotor_model_->getState();

  sim_time_ = sim_time_ + ros::Duration(step);

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

// | ------------------------ callbacks ----------------------- |

/* callbackRateCmd() //{ */

void MultirotorSimulator::callbackRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp) {

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

}  // namespace mrs_multirotor_simulator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_multirotor_simulator::MultirotorSimulator, nodelet::Nodelet)
