/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <uav_system_ros.h>

#include <rosgraph_msgs/Clock.h>

#include <geometry_msgs/PoseArray.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_multirotor_simulator/multirotor_simulatorConfig.h>

#include <KDTreeVectorOfVectorsAdaptor.h>
#include <Eigen/Dense>

//}

namespace mrs_multirotor_simulator
{

typedef std::vector<Eigen::VectorXd> my_vector_of_vectors_t;

/* class MultirotorSimulator //{ */

class MultirotorSimulator : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_;

  // | ------------------------- params ------------------------- |

  double _simulation_rate_;

  ros::Time  sim_time_;
  std::mutex mutex_sim_time_;

  double _clock_min_dt_;

  std::string _world_frame_name_;

  // | ------------------------- timers ------------------------- |

  ros::WallTimer timer_main_;
  void           timerMain(const ros::WallTimerEvent& event);

  ros::WallTimer timer_status_;
  void           timerStatus(const ros::WallTimerEvent& event);

  // | ------------------------ rtf check ----------------------- |

  double    actual_rtf_ = 1.0;
  ros::Time last_sim_time_status_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<rosgraph_msgs::Clock>     ph_clock_;
  mrs_lib::PublisherHandler<geometry_msgs::PoseArray> ph_poses_;

  // | ------------------------- system ------------------------- |

  std::vector<std::unique_ptr<UavSystemRos>> uavs_;

  // | -------------------------- time -------------------------- |

  ros::Time last_published_time_;

  // | ------------------------- methods ------------------------ |

  void handleCollisions(void);

  void publishPoses(void);

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

  srand(time(NULL));

  sim_time_            = ros::Time(0);
  last_published_time_ = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh_, "MultirotorSimulator");

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");
  param_loader.addYamlFileFromParam("config_uavs");

  param_loader.loadParam("simulation_rate", _simulation_rate_);
  param_loader.loadParam("realtime_factor", drs_params_.realtime_factor);
  param_loader.loadParam("collisions/enabled", drs_params_.collisions_enabled);
  param_loader.loadParam("collisions/crash", drs_params_.collisions_crash);
  param_loader.loadParam("collisions/rebounce", drs_params_.collisions_rebounce);
  param_loader.loadParam("frames/world/name", _world_frame_name_);

  double clock_rate;
  param_loader.loadParam("clock_rate", clock_rate);

  drs_params_.paused = false;

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

  _clock_min_dt_ = 1.0 / clock_rate;

  // | ----------------------- publishers ----------------------- |

  ph_clock_ = mrs_lib::PublisherHandler<rosgraph_msgs::Clock>(nh_, "clock_out", 10, false);

  ph_poses_ = mrs_lib::PublisherHandler<geometry_msgs::PoseArray>(nh_, "uav_poses_out", 10, false);

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createWallTimer(ros::WallDuration(1.0 / (_simulation_rate_ * drs_params_.realtime_factor)), &MultirotorSimulator::timerMain, this);

  timer_status_ = nh_.createWallTimer(ros::WallDuration(1.0), &MultirotorSimulator::timerStatus, this);

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

  publishPoses();

  handleCollisions();

  // | ---------------------- publish time ---------------------- |

  if ((sim_time_ - last_published_time_).toSec() >= _clock_min_dt_) {

    rosgraph_msgs::Clock ros_time;

    ros_time.clock.fromSec(sim_time_.toSec());

    ph_clock_.publish(ros_time);

    last_published_time_ = sim_time_;
  }
}

//}

/* timeStatus() //{ */

void MultirotorSimulator::timerStatus([[maybe_unused]] const ros::WallTimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  auto sim_time   = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);
  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  ros::Duration last_sec_sim_dt = sim_time - last_sim_time_status_;

  last_sim_time_status_ = sim_time;

  double last_sec_rtf = last_sec_sim_dt.toSec() / 1.0;

  actual_rtf_ = 0.9 * actual_rtf_ + 0.1 * last_sec_rtf;

  ROS_INFO_THROTTLE(0.1, "[MultirotorSimulator]: %s, desired RTF = %.2f, actual RTF = %.2f", drs_params.paused ? "paused" : "running",
                    drs_params.realtime_factor, actual_rtf_);
}

//}

/* callbackDrs() //{ */

void MultirotorSimulator::callbackDrs(mrs_multirotor_simulator::multirotor_simulatorConfig& config, [[maybe_unused]] uint32_t level) {

  {
    // | ----------------- pausing the simulation ----------------- |

    auto old_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

    if (!old_params.paused && config.paused) {
      timer_main_.stop();
    } else if (old_params.paused && !config.paused) {
      timer_main_.start();
    }
  }

  // | --------------------- save the params -------------------- |

  {
    std::scoped_lock lock(mutex_drs_params_);

    drs_params_ = config;
  }

  // | ----------------- set the realtime factor ---------------- |

  timer_main_.setPeriod(ros::WallDuration(1.0 / (_simulation_rate_ * config.realtime_factor)), true);

  ROS_INFO("[MultirotorSimulator]: DRS updated params");
}

//}

/* handleCollisions() //{ */

void MultirotorSimulator::handleCollisions(void) {

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  std::vector<Eigen::VectorXd> poses;

  for (size_t i = 0; i < uavs_.size(); i++) {
    poses.push_back(uavs_[i]->getPose());
  }

  typedef KDTreeVectorOfVectorsAdaptor<my_vector_of_vectors_t, double> my_kd_tree_t;

  my_kd_tree_t mat_index(3, poses, 10);

  std::vector<nanoflann::ResultItem<int, double>> indices_dists;

  std::vector<Eigen::Vector3d> forces;

  for (size_t i = 0; i < uavs_.size(); i++) {
    forces.push_back(Eigen::Vector3d::Zero());
  }

  for (size_t i = 0; i < uavs_.size(); i++) {

    MultirotorModel::State       state_1  = uavs_[i]->getState();
    MultirotorModel::ModelParams params_1 = uavs_[i]->getParams();

    nanoflann::RadiusResultSet<double, int> resultSet(3.0, indices_dists);

    mat_index.index->findNeighbors(resultSet, &state_1.x[0]);

    for (size_t j = 0; j < resultSet.m_indices_dists.size(); j++) {

      const size_t idx  = resultSet.m_indices_dists[j].first;
      const double dist = resultSet.m_indices_dists[j].second;

      if (idx == i) {
        continue;
      }

      MultirotorModel::State       state_2  = uavs_[idx]->getState();
      MultirotorModel::ModelParams params_2 = uavs_[idx]->getParams();

      const double crit_dist = params_1.arm_length + params_1.prop_radius + params_2.arm_length + params_2.prop_radius;

      const Eigen::Vector3d rel_pos = state_1.x - state_2.x;

      if (dist < crit_dist) {
        if (drs_params.collisions_crash) {
          uavs_[idx]->crash();
        } else {
          forces[i] += drs_params.collisions_rebounce * rel_pos.normalized() * params_1.mass * (params_2.mass / (params_1.mass + params_2.mass));
        }
      }
    }
  }

  for (size_t i = 0; i < uavs_.size(); i++) {
    uavs_[i]->applyForce(forces[i]);
  }
}

//}

/* publishPoses() //{ */

void MultirotorSimulator::publishPoses(void) {

  auto sim_time = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);

  geometry_msgs::PoseArray pose_array;

  pose_array.header.stamp    = sim_time;
  pose_array.header.frame_id = _world_frame_name_;

  for (size_t i = 0; i < uavs_.size(); i++) {

    auto state = uavs_[i]->getState();

    geometry_msgs::Pose pose;

    pose.position.x  = state.x[0];
    pose.position.y  = state.x[1];
    pose.position.z  = state.x[2];
    pose.orientation = mrs_lib::AttitudeConverter(state.R);

    pose_array.poses.push_back(pose);
  }

  ph_poses_.publish(pose_array);
}

//}

}  // namespace mrs_multirotor_simulator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_multirotor_simulator::MultirotorSimulator, nodelet::Nodelet)
