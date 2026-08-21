/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/dynparam_mgr.h>
#include <mrs_lib/mutex.h>

#include <mrs_multirotor_simulator/plugins/world_plugin.h>

//}

namespace mrs_multirotor_simulator_plugins
{

/* class RandomizePositionWorldPlugin //{ */

class RandomizePositionWorldPlugin : public mrs_multirotor_simulator::WorldPlugin {

public:
  bool initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_multirotor_simulator::WorldPluginCommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_multirotor_simulator::WorldPluginPrivateHandlers_t> private_handlers) override;

  void update(const double dt, const rclcpp::Time &sim_time) override;

private:
  rclcpp::Node::SharedPtr                                                node_;
  std::shared_ptr<mrs_multirotor_simulator::WorldPluginCommonHandlers_t> common_handlers_;

  double randd(const double from, const double to);

  // | --------------------- dynamic params --------------------- |

  std::shared_ptr<mrs_lib::DynparamMgr> dynparam_mgr_;
  std::mutex                            mutex_drs_params_;

  struct drs_params_t
  {
    double bounds_x = 50.0;
    double bounds_y = 50.0;
    double bounds_z = 50.0;
    bool   trigger  = false;
  };

  drs_params_t drs_params_;

  std::atomic<bool> trigger_requested_ = false;

  void callbackTrigger(const bool &value);
};

//}

/* initialize() //{ */

bool RandomizePositionWorldPlugin::initialize(const rclcpp::Node::SharedPtr                                          &node,
                                              std::shared_ptr<mrs_multirotor_simulator::WorldPluginCommonHandlers_t>  common_handlers,
                                              std::shared_ptr<mrs_multirotor_simulator::WorldPluginPrivateHandlers_t> private_handlers) {

  node_            = node;
  common_handlers_ = common_handlers;

  private_handlers->param_loader->loadParam("bounds/x", drs_params_.bounds_x, drs_params_.bounds_x);
  private_handlers->param_loader->loadParam("bounds/y", drs_params_.bounds_y, drs_params_.bounds_y);
  private_handlers->param_loader->loadParam("bounds/z", drs_params_.bounds_z, drs_params_.bounds_z);

  dynparam_mgr_ = std::make_shared<mrs_lib::DynparamMgr>(node_, mutex_drs_params_);

  dynparam_mgr_->register_param("dynamic/trigger", &drs_params_.trigger, false,
                                (std::function<void(const bool &)>)std::bind(&RandomizePositionWorldPlugin::callbackTrigger, this, std::placeholders::_1));

  if (!private_handlers->param_loader->loadedSuccessfully() || !dynparam_mgr_->loaded_successfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[RandomizePositionWorldPlugin]: could not load all parameters!");
    return false;
  }

  srand(time(NULL));

  RCLCPP_INFO(node_->get_logger(), "[RandomizePositionWorldPlugin]: initialized, bounds = [%.1f, %.1f, %.1f] m", drs_params_.bounds_x, drs_params_.bounds_y,
              drs_params_.bounds_z);

  return true;
}

//}

/* update() //{ */

void RandomizePositionWorldPlugin::update([[maybe_unused]] const double dt, [[maybe_unused]] const rclcpp::Time &sim_time) {

  if (!trigger_requested_.exchange(false)) {
    return;
  }

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  for (const auto &[uav_name, uav_system] : common_handlers_->uavs) {

    const Eigen::Vector3d pos(randd(-drs_params.bounds_x, drs_params.bounds_x), randd(-drs_params.bounds_y, drs_params.bounds_y),
                              randd(0, drs_params.bounds_z));

    const double heading = randd(-M_PI, M_PI);

    uav_system->setStatePos(pos, heading);

    RCLCPP_INFO(node_->get_logger(), "[RandomizePositionWorldPlugin]: '%s' randomized to [%.1f, %.1f, %.1f]", uav_name.c_str(), pos.x(), pos.y(), pos.z());
  }
}

//}

/* callbackTrigger() //{ */

void RandomizePositionWorldPlugin::callbackTrigger(const bool &value) {

  if (value) {
    trigger_requested_ = true;
  }
}

//}

/* randd() //{ */

double RandomizePositionWorldPlugin::randd(const double from, const double to) {

  return from + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (to - from)));
}

//}

} // namespace mrs_multirotor_simulator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_multirotor_simulator_plugins::RandomizePositionWorldPlugin, mrs_multirotor_simulator::WorldPlugin)
