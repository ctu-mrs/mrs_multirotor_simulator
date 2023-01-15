/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <quadrotor_model.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>

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

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent& event);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<sensor_msgs::Imu>    ph_imu_;
  mrs_lib::PublisherHandler<sensor_msgs::Range>  ph_range_;
  mrs_lib::PublisherHandler<geometry_msgs::Pose> ph_pose_;
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

  model_params_.J             = param_loader.loadMatrixStatic2<3, 3>("J");
  model_params_.mixing_matrix = param_loader.loadMatrixDynamic2("propulsion/mixing_matrix", 3, -1);

  double moment_constant;
  param_loader.loadParam("propulsion/moment_constant", moment_constant);

  model_params_.mixing_matrix.row(0) = model_params_.mixing_matrix.row(0) * model_params_.kf * model_params_.arm_length;
  model_params_.mixing_matrix.row(1) = model_params_.mixing_matrix.row(1) * model_params_.kf * model_params_.arm_length;

  const double km = moment_constant * (3.0 * model_params_.prop_radius) * model_params_.kf;

  model_params_.mixing_matrix.row(2) = model_params_.mixing_matrix.row(2) * km * model_params_.arm_length;

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ControlManager]: could not load all parameters!");
    ros::shutdown();
  }

  quadrotor_model_ = std::make_unique<QuadrotorModel>(model_params_);

  // | ----------------------- publishers ----------------------- |

  ph_imu_   = mrs_lib::PublisherHandler<sensor_msgs::Imu>(nh_, "imu_out", 1, false);
  ph_range_ = mrs_lib::PublisherHandler<sensor_msgs::Range>(nh_, "range_out", 1, false);
  ph_pose_  = mrs_lib::PublisherHandler<geometry_msgs::Pose>(nh_, "pose_out", 1, false);

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

  Eigen::VectorXd input = Eigen::VectorXd::Zero(4);

  input << 0.6, 0.6, 0.8, 0.6;

  quadrotor_model_->setInput(input);

  quadrotor_model_->step(0.01);

  auto state = quadrotor_model_->getState();

  ROS_INFO_STREAM("[MultirotorSimulator]: pos " << state.x.transpose());
  ROS_INFO_STREAM("[MultirotorSimulator]: v " << state.v.transpose());
  ROS_INFO_STREAM("[MultirotorSimulator]: omega " << state.omega.transpose());
  ROS_INFO("[MultirotorSimulator]: ");

  // | ---------------------- publish pose ---------------------- |

  geometry_msgs::Pose pose;

  pose.orientation = mrs_lib::AttitudeConverter(state.R);

  pose.position.x = state.x[0];
  pose.position.y = state.x[1];
  pose.position.z = state.x[2];

  ph_pose_.publish(pose);

  // | ----------------------- publish IMU ---------------------- |

  sensor_msgs::Imu imu;

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

}  // namespace mrs_multirotor_simulator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_multirotor_simulator::MultirotorSimulator, nodelet::Nodelet)
