/* includes //{ */

#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <mrs_multirotor_simulator/plugins/uav_plugin.h>

//}

namespace mrs_multirotor_simulator_plugins
{

/* class NeighborCountUavPlugin //{ */

/**
 * @brief A minimal, purely observational UAV plugin: it never touches the uav's controls,
 *        it just logs how many other uavs are currently within the shared neighbor radius.
 *        Meant to demonstrate attaching more than one UavPlugin to the same uav at once
 *        (e.g. alongside a controlling plugin such as BoidsUavPlugin).
 */
class NeighborCountUavPlugin : public mrs_multirotor_simulator::UavPlugin {

public:
  bool initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_multirotor_simulator::UavPluginCommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_multirotor_simulator::UavPluginPrivateHandlers_t> private_handlers) override;

  void update(const double dt, const rclcpp::Time &sim_time, const std::vector<mrs_multirotor_simulator::UavPluginNeighborState_t> &neighbors) override;

private:
  rclcpp::Node::SharedPtr node_;
  std::string             uav_name_;

  // NOTE: RCLCPP_INFO_THROTTLE keys its "last logged" state to the call site (a static
  // variable at that source line), which every instance of this plugin shares since they
  // all execute the same compiled line -- that throttles across all uavs combined, not
  // per uav. Tracking it ourselves, per instance, using the simulation time already handed
  // to update() gives each uav its own independent 1 Hz cadence.
  std::optional<rclcpp::Time> last_log_time_;
  static constexpr double     LOG_PERIOD = 1.0; // [s]
};

//}

/* initialize() //{ */

bool NeighborCountUavPlugin::initialize(const rclcpp::Node::SharedPtr                                                        &node,
                                        [[maybe_unused]] std::shared_ptr<mrs_multirotor_simulator::UavPluginCommonHandlers_t> common_handlers,
                                        std::shared_ptr<mrs_multirotor_simulator::UavPluginPrivateHandlers_t>                 private_handlers) {

  node_     = node;
  uav_name_ = private_handlers->uav_name;

  RCLCPP_INFO(node_->get_logger(), "[NeighborCountUavPlugin]: '%s' initialized", uav_name_.c_str());

  return true;
}

//}

/* update() //{ */

void NeighborCountUavPlugin::update([[maybe_unused]] const double dt, const rclcpp::Time &sim_time,
                                    const std::vector<mrs_multirotor_simulator::UavPluginNeighborState_t> &neighbors) {

  if (last_log_time_ && (sim_time - *last_log_time_).seconds() < LOG_PERIOD) {
    return;
  }

  last_log_time_ = sim_time;

  RCLCPP_INFO(node_->get_logger(), "[NeighborCountUavPlugin]: '%s' has %zu uav(s) in its neighborhood", uav_name_.c_str(), neighbors.size());
}

//}

} // namespace mrs_multirotor_simulator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_multirotor_simulator_plugins::NeighborCountUavPlugin, mrs_multirotor_simulator::UavPlugin)
