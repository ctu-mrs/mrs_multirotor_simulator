/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <uav_system_ros.h>

#include <rosgraph_msgs/Clock.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
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
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_;

  // | ------------------------- params ------------------------- |

  double    _simulation_rate_;
  ros::Time sim_time_;

  // | ------------------------- timers ------------------------- |

  ros::WallTimer timer_main_;
  void           timerMain(const ros::WallTimerEvent& event);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<rosgraph_msgs::Clock> ph_clock_;

  // | ------------------------- system ------------------------- |

  std::vector<std::unique_ptr<UavSystemRos>> uavs_;

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

  sim_time_ = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh_, "MultirotorSimulator");

  param_loader.loadParam("simulation_rate", _simulation_rate_);
  param_loader.loadParam("realtime_factor", drs_params_.realtime_factor);

  std::vector<std::string> uav_names;

  param_loader.loadParam("uav_names", uav_names);

  for (size_t i = 0; i < uav_names.size(); i++) {

    std::string uav_name = uav_names[i];

    ROS_INFO("[MultirotorSimulator]: initializing '%s'", uav_name.c_str());

    uavs_.push_back(std::make_unique<UavSystemRos>(nh_, uav_name));
  }

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&MultirotorSimulator::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MultirotorSimulator]: could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- publishers ----------------------- |

  ph_clock_ = mrs_lib::PublisherHandler<rosgraph_msgs::Clock>(nh_, "clock_out", 10, false);

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createWallTimer(ros::WallDuration(1.0 / (_simulation_rate_ * drs_params_.realtime_factor)), &MultirotorSimulator::timerMain, this);

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

  // step the time
  sim_time_ = sim_time_ + ros::Duration(simulation_step_size);

  for (size_t i = 0; i < uavs_.size(); i++) {
    uavs_[i]->makeStep(simulation_step_size);
  }

  // | ---------------------- publish time ---------------------- |

  rosgraph_msgs::Clock ros_time;

  ros_time.clock.fromSec(sim_time_.toSec());

  ph_clock_.publish(ros_time);
}

//}

/* callbackDrs() //{ */

void MultirotorSimulator::callbackDrs(mrs_multirotor_simulator::multirotor_simulatorConfig& config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_drs_params_);

    drs_params_ = config;
  }

  timer_main_.setPeriod(ros::WallDuration(1.0 / (_simulation_rate_ * config.realtime_factor)), true);

  ROS_INFO("[MultirotorSimulator]: DRS updated params");
}

//}

}  // namespace mrs_multirotor_simulator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_multirotor_simulator::MultirotorSimulator, nodelet::Nodelet)
