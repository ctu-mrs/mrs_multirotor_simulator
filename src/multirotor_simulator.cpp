/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <quadrotor_model.h>
#include <controllers/rate_controller.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>

#include <mrs_msgs/HwApiAttitudeRateCmd.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/subscribe_handler.h>

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

  // | --------------------- dynamics model --------------------- |

  std::unique_ptr<QuadrotorModel> quadrotor_model_;
  RateController                  rate_controller_;

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent& event);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<sensor_msgs::Imu>   ph_imu_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry> ph_odom_;

  // | ----------------------- subscribers ---------------------- |

  void callbackRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd> sh_rate_cmd_;
};

//}

/* onInit() //{ */

void MultirotorSimulator::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "MultirotorSimulator");

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

  ph_imu_  = mrs_lib::PublisherHandler<sensor_msgs::Imu>(nh_, "imu_out", 1, false);
  ph_odom_ = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, "odom_out", 1, false);

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

  timer_main_ = nh_.createTimer(ros::Rate(100.0), &MultirotorSimulator::timerMain, this);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[MultirotorSimulator]: initialized");
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void MultirotorSimulator::timerMain(const ros::TimerEvent& event) {

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

  quadrotor_model_->step(0.01);

  auto state = quadrotor_model_->getState();

  ROS_INFO_STREAM("[MultirotorSimulator]: pos " << state.x.transpose());
  ROS_INFO_STREAM("[MultirotorSimulator]: v " << state.v.transpose());
  ROS_INFO_STREAM("[MultirotorSimulator]: omega " << state.omega.transpose());
  ROS_INFO("[MultirotorSimulator]: ");

  // | ---------------------- publish pose ---------------------- |

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

/* callbackRateCmd //{ */

void MultirotorSimulator::callbackRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MultirotorSimulator]: getting attitude rate command");
}

//}

}  // namespace mrs_multirotor_simulator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_multirotor_simulator::MultirotorSimulator, nodelet::Nodelet)
