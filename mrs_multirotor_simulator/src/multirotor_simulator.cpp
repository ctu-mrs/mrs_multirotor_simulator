/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/timer_handler.h>
#include <mrs_lib/dynparam_mgr.h>
#include <mrs_lib/node.h>
#include <mrs_lib/scope_timer.h>

#include <KDTreeVectorOfVectorsAdaptor.h>
#include <Eigen/Dense>
#include <vector>

#include <mrs_multirotor_simulator/uav_system_ros.h>
#include <mrs_multirotor_simulator/rate_counter.h>

using namespace std::chrono_literals;

//}

namespace mrs_multirotor_simulator
{

/* class MultirotorSimulator //{ */

class MultirotorSimulator : public mrs_lib::Node {

public:
  MultirotorSimulator(rclcpp::NodeOptions options);

private:
  rclcpp::CallbackGroup::SharedPtr cbgrp_main_;
  rclcpp::CallbackGroup::SharedPtr cbgrp_status_;

  void initialize();

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;
  std::atomic<bool>        is_initialized_ = false;

  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;

  // | ------------------------- params ------------------------- |

  double _simulation_rate_;
  double _clock_rate_;

  rclcpp::Time sim_time_;
  rclcpp::Time last_step_time_;
  std::mutex   mutex_sim_time_;

  std::string _world_frame_name_;

  // | ------------------------- timers ------------------------- |

  rclcpp::TimerBase::SharedPtr timer_main_;
  void                         timerMain();

  rclcpp::TimerBase::SharedPtr timer_status_;
  void                         timerStatus();

  // | ------------------------ rtf check ----------------------- |

  double       actual_rtf_ = 1.0;
  rclcpp::Time last_sim_time_status_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<rosgraph_msgs::msg::Clock>     ph_clock_;
  mrs_lib::PublisherHandler<geometry_msgs::msg::PoseArray> ph_poses_;

  // | ------------------------- system ------------------------- |

  std::vector<std::unique_ptr<UavSystemRos>> uavs_;

  // | ------------------------- methods ------------------------ |

  void handleCollisions(void);

  void publishPoses(void);

  std::shared_ptr<mrs_lib::TransformBroadcaster> tf_broadcaster_;

  // | --------------------- dynamic params --------------------- |

  std::shared_ptr<mrs_lib::DynparamMgr> dynparam_mgr_;

  struct drs_params
  {
    double realtime_factor     = 1.0;
    bool   paused              = false;
    bool   collisions_enabled  = false;
    bool   collisions_crash    = false;
    double collisions_rebounce = 1;
  };

  void callbackRealtimeFactor(const double &param_value);
  void callbackPause(const bool &param_value);

  drs_params drs_params_;
  std::mutex mutex_drs_params_;
};

//}

/* MultirotorSimulator::MultirotorSimulator() //{ */

MultirotorSimulator::MultirotorSimulator(rclcpp::NodeOptions options) : mrs_lib::Node("multirotor_simulator", options) {

  this->initialize();
}

//}

// | ------------------------- timers ------------------------- |

/* initialize() //{ */

void MultirotorSimulator::initialize() {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  srand(time(NULL));

  RCLCPP_INFO(node_->get_logger(), "initializing");

  cbgrp_main_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbgrp_status_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // | ---------------- initialize param wrappers --------------- |

  mrs_lib::ParamLoader param_loader(node_, node_->get_name());

  dynparam_mgr_ = std::make_shared<mrs_lib::DynparamMgr>(node_, mutex_drs_params_);

  // | ----------------------- load files ----------------------- |

  // load custom config

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    RCLCPP_INFO(node_->get_logger(), "loading custom config '%s", custom_config_path.c_str());

    param_loader.addYamlFile(custom_config_path);
  }

  // load other configs

  std::vector<std::string> config_files;
  param_loader.loadParam("simulator_configs", config_files);

  for (auto config_file : config_files) {
    RCLCPP_INFO(node_->get_logger(), "loading config file '%s'", config_file.c_str());

    param_loader.addYamlFile(config_file);
  }

  dynparam_mgr_->get_param_provider().copyYamls(param_loader.getParamProvider());

  // | ----------------------- load params ---------------------- |

  param_loader.loadParam("simulation_rate", _simulation_rate_);
  param_loader.loadParam("clock_rate", _clock_rate_);

  dynparam_mgr_->register_param("dynamic/realtime_factor", &drs_params_.realtime_factor, mrs_lib::DynparamMgr::range_t<double>(0.01, 10),
                                (std::function<void(const double &)>)std::bind(&MultirotorSimulator::callbackRealtimeFactor, this, std::placeholders::_1));

  dynparam_mgr_->register_param("dynamic/collisions/enabled", &drs_params_.collisions_enabled);

  dynparam_mgr_->register_param("dynamic/collisions/crash", &drs_params_.collisions_crash);

  dynparam_mgr_->register_param("dynamic/collisions/rebounce", &drs_params_.collisions_rebounce, mrs_lib::DynparamMgr::range_t<double>(0.1, 1000));

  dynparam_mgr_->register_param("dynamic/paused", &drs_params_.paused, false,
                                (std::function<void(const bool &)>)std::bind(&MultirotorSimulator::callbackPause, this, std::placeholders::_1));

  param_loader.loadParam("frames/world/name", _world_frame_name_);

  bool sim_time_from_wall_time;
  param_loader.loadParam("sim_time_from_wall_time", sim_time_from_wall_time);

  if (sim_time_from_wall_time) {
    sim_time_       = clock_->now();
    last_step_time_ = clock_->now();
  } else {
    sim_time_       = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_step_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  last_sim_time_status_ = sim_time_;

  drs_params_.paused = false;

  tf_broadcaster_ = std::make_shared<mrs_lib::TransformBroadcaster>(node_);

  std::vector<std::string> uav_names;

  param_loader.loadParam("uav_names", uav_names);

  for (size_t i = 0; i < uav_names.size(); i++) {

    std::string uav_name = uav_names.at(i);

    RCLCPP_INFO(node_->get_logger(), "initializing '%s'", uav_name.c_str());

    UavSystemRos_CommonHandlers_t common_handlers;

    common_handlers.node                  = node_;
    common_handlers.uav_name              = uav_name;
    common_handlers.transform_broadcaster = tf_broadcaster_;

    uavs_.push_back(std::make_unique<UavSystemRos>(common_handlers));
  }

  RCLCPP_INFO(node_->get_logger(), "all uavs initialized");

  if (!param_loader.loadedSuccessfully() || !dynparam_mgr_->loaded_successfully()) {
    RCLCPP_ERROR(node_->get_logger(), "could not load all parameters!");
    rclcpp::shutdown();
  }

  if (_clock_rate_ < _simulation_rate_) {
    RCLCPP_ERROR(node_->get_logger(), "clock_rate (%.2f Hz) should be higher than simulation rate (%.2f Hz)!", _clock_rate_, _simulation_rate_);
    rclcpp::shutdown();
    exit(1);
  }

  // | ----------------------- publishers ----------------------- |

  ph_clock_ = mrs_lib::PublisherHandler<rosgraph_msgs::msg::Clock>(node_, "~/clock_out");

  ph_poses_ = mrs_lib::PublisherHandler<geometry_msgs::msg::PoseArray>(node_, "~/uav_poses_out");

  // | ------------------------- timers ------------------------- |

  timer_main_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / (_clock_rate_ * drs_params_.realtime_factor)),
                                         std::bind(&MultirotorSimulator::timerMain, this), cbgrp_main_);

  timer_status_ = node_->create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&MultirotorSimulator::timerStatus, this), cbgrp_status_);

  // | ----------------------- scope timer ---------------------- |

  scope_timer_logger_ = std::make_shared<mrs_lib::ScopeTimerLogger>(node_, "", false);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

/* timerMain() //{ */

void MultirotorSimulator::timerMain() {

  if (!is_initialized_) {
    return;
  }

  double simulation_step_size = 1.0 / _simulation_rate_;
  double clock_step_size      = 1.0 / _clock_rate_;

  auto sim_time = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);

  sim_time = sim_time + rclcpp::Duration(std::chrono::duration<double>(clock_step_size));

  mrs_lib::set_mutexed(mutex_sim_time_, sim_time, sim_time_);

  const double dt_since_last_step = (sim_time - last_step_time_).seconds();

  if (dt_since_last_step >= simulation_step_size) {

    for (size_t i = 0; i < uavs_.size(); i++) {

      uavs_.at(i)->makeStep(dt_since_last_step, sim_time_.seconds());
    }

    publishPoses();

    handleCollisions();

    last_step_time_ = sim_time;
  }

  // | ---------------------- publish time ---------------------- |

  rosgraph_msgs::msg::Clock ros_time;

  ros_time.clock = sim_time;

  ph_clock_.publish(ros_time);
}

//}

/* timeStatus() //{ */

void MultirotorSimulator::timerStatus() {

  if (!is_initialized_) {
    return;
  }

  auto sim_time   = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);
  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  rclcpp::Duration last_sec_sim_dt = sim_time - last_sim_time_status_;

  last_sim_time_status_ = sim_time;

  double last_sec_rtf = last_sec_sim_dt.seconds() / 1.0;

  actual_rtf_ = 0.9 * actual_rtf_ + 0.1 * last_sec_rtf;

  RCLCPP_INFO(node_->get_logger(), "%s, desired RTF = %.2f, actual RTF = %.2f", drs_params.paused ? "paused" : "running", drs_params.realtime_factor,
              actual_rtf_);
}

//}

/* dynamic parameter callbacks //{ */

/* callbackRealtimeFactor() //{ */

void MultirotorSimulator::callbackRealtimeFactor(const double &param_value) {

  timer_main_->cancel();

  timer_main_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / (_clock_rate_ * param_value)), std::bind(&MultirotorSimulator::timerMain, this),
                                         cbgrp_main_);

  RCLCPP_INFO(node_->get_logger(), "desired realtime factor updated to %.3f", param_value);
}

//}

/* callbackPause() //{ */

void MultirotorSimulator::callbackPause(const bool &param_value) {

  RCLCPP_INFO(node_->get_logger(), "callbackPause()");

  if (param_value) {

    timer_main_->cancel();
    timer_status_->cancel();

    RCLCPP_INFO(node_->get_logger(), "paused");

  } else {

    timer_main_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / (_clock_rate_ * drs_params_.realtime_factor)),
                                           std::bind(&MultirotorSimulator::timerMain, this), cbgrp_main_);

    timer_status_ = node_->create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&MultirotorSimulator::timerStatus, this), cbgrp_status_);

    RCLCPP_INFO(node_->get_logger(), "unpaused");
  }
}

//}

//}

/* handleCollisions() //{ */

void MultirotorSimulator::handleCollisions(void) {

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  if (!(drs_params.collisions_crash || drs_params.collisions_enabled)) {
    return;
  }

  std::vector<Eigen::VectorXd> poses;

  for (size_t i = 0; i < uavs_.size(); i++) {
    poses.push_back(uavs_.at(i)->getPose());
  }

  typedef std::vector<Eigen::VectorXd> my_vector_of_vectors_t;

  typedef KDTreeVectorOfVectorsAdaptor<my_vector_of_vectors_t, double> my_kd_tree_t;

  my_kd_tree_t mat_index(3, poses, 10);

  std::vector<nanoflann::ResultItem<int, double>> indices_dists;

  std::vector<Eigen::Vector3d> forces;

  for (size_t i = 0; i < uavs_.size(); i++) {
    forces.push_back(Eigen::Vector3d::Zero());
  }

  for (size_t i = 0; i < uavs_.size(); i++) {

    MultirotorModel::State       state_1  = uavs_.at(i)->getState();
    MultirotorModel::ModelParams params_1 = uavs_.at(i)->getParams();

    nanoflann::RadiusResultSet<double, int> resultSet(3.0, indices_dists);

    mat_index.index->findNeighbors(resultSet, &state_1.x(0));

    for (size_t j = 0; j < resultSet.m_indices_dists.size(); j++) {

      const size_t idx  = resultSet.m_indices_dists.at(j).first;
      const double dist = resultSet.m_indices_dists.at(j).second;

      if (idx == i) {
        continue;
      }

      MultirotorModel::State       state_2  = uavs_.at(idx)->getState();
      MultirotorModel::ModelParams params_2 = uavs_.at(idx)->getParams();

      const double crit_dist = params_1.arm_length + params_1.prop_radius + params_2.arm_length + params_2.prop_radius;

      const Eigen::Vector3d rel_pos = state_1.x - state_2.x;

      if (dist < crit_dist) {
        if (drs_params.collisions_crash && !uavs_.at(idx)->hasCrashed()) {

          RCLCPP_WARN(node_->get_logger(), "uav%u crashed", int(idx + 1));

          uavs_.at(idx)->crash();

        } else {
          forces.at(i) += drs_params.collisions_rebounce * rel_pos.normalized() * params_1.mass * (params_2.mass / (params_1.mass + params_2.mass));
        }
      }
    }
  }

  for (size_t i = 0; i < uavs_.size(); i++) {
    uavs_.at(i)->applyForce(forces.at(i));
  }
}

//}

/* publishPoses() //{ */

void MultirotorSimulator::publishPoses(void) {

  auto sim_time = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);

  geometry_msgs::msg::PoseArray pose_array;

  pose_array.header.stamp    = sim_time;
  pose_array.header.frame_id = _world_frame_name_;

  for (size_t i = 0; i < uavs_.size(); i++) {

    auto state = uavs_.at(i)->getState();

    geometry_msgs::msg::Pose pose;

    pose.position.x  = state.x(0);
    pose.position.y  = state.x(1);
    pose.position.z  = state.x(2);
    pose.orientation = mrs_lib::AttitudeConverter(state.R);

    pose_array.poses.push_back(pose);
  }

  ph_poses_.publish(pose_array);
}

//}

} // namespace mrs_multirotor_simulator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_multirotor_simulator::MultirotorSimulator)
