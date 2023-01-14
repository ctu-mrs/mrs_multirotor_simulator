/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <quadrotor_model.h>

#include <mrs_lib/param_loader.h>

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
  param_loader.loadParam("prop_radius", model_params_.prop_radius);
  param_loader.loadParam("g", model_params_.g);
  param_loader.loadParam("rpm/min", model_params_.min_rpm);
  param_loader.loadParam("rpm/max", model_params_.max_rpm);

  model_params_.J             = param_loader.loadMatrixStatic2<3, 3>("J");
  model_params_.mixing_matrix = param_loader.loadMatrixDynamic2("mixing_matrix", 3, -1);

  model_params_.mixing_matrix.row(0) = model_params_.mixing_matrix.row(0) * model_params_.kf * model_params_.arm_length;
  model_params_.mixing_matrix.row(1) = model_params_.mixing_matrix.row(1) * model_params_.kf * model_params_.arm_length;

  double km_ = 0.07 * (3.0 * model_params_.prop_radius) * model_params_.kf;

  model_params_.mixing_matrix.row(2) = model_params_.mixing_matrix.row(2) * model_params_.km * model_params_.arm_length;

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ControlManager]: could not load all parameters!");
    ros::shutdown();
  }

  quadrotor_model_ = std::make_unique<QuadrotorModel>(model_params_);

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
}

//}

}  // namespace mrs_multirotor_simulator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_multirotor_simulator::MultirotorSimulator, nodelet::Nodelet)
