#ifndef MRS_MULTIROTOR_SIMULATOR_UAV_PLUGIN_H
#define MRS_MULTIROTOR_SIMULATOR_UAV_PLUGIN_H

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/param_loader.h>

#include <mrs_multirotor_simulator/uav_system/uav_system.hpp>

//}

namespace mrs_multirotor_simulator
{

/* struct UavPluginNeighborState_t //{ */

/**
 * @brief Read-only snapshot of a neighboring UAV's state. There is no control handle attached,
 *        i.e., a UAV plugin cannot control any UAV other than its own through this struct.
 */
struct UavPluginNeighborState_t
{
  std::string            uav_name;
  MultirotorModel::State state;
};

//}

/* struct UavPluginCommonHandlers_t //{ */

/**
 * @brief Handlers shared by all loaded UAV plugins. Constructed once by the MultirotorSimulator node.
 */
struct UavPluginCommonHandlers_t
{
  rclcpp::Node::SharedPtr node;

  /**
   * @brief returns the neighbor search radius, shared and dynamically-reconfigurable for all UAVs
   */
  std::function<double(void)> getNeighborRadius;
};

//}

/* struct UavPluginPrivateHandlers_t //{ */

/**
 * @brief Handlers provided individually to each UAV plugin instance.
 */
struct UavPluginPrivateHandlers_t
{
  std::unique_ptr<mrs_lib::ParamLoader> param_loader;        ///< this plugin's own scoped param loader
  std::shared_ptr<mrs_lib::ParamLoader> parent_param_loader; ///< this UAV's param loader, for copyYamls()
  std::string                           uav_name;

  /**
   * @brief full access to this UAV's API: setInput(), getState(), crash(), ...
   */
  std::shared_ptr<UavSystem> uav_system;
};

//}

/* class UavPlugin //{ */

/**
 * @brief Abstract interface for a "UAV" plugin. A UAV plugin runs as one instance per UAV
 *        (opt-in, one plugin per UAV at most) and is given full API access to its own UavSystem,
 *        plus read-only states of neighboring UAVs within a configurable radius.
 */
class UavPlugin {

public:
  virtual ~UavPlugin() = default;

  /**
   * @brief Initializes the plugin. Called once after the UAV is spawned.
   *
   * @param node the sub-node given to this plugin instance
   * @param common_handlers handlers shared between all UAV plugins
   * @param private_handlers handlers provided individually to this plugin
   *
   * @return true if success
   */
  virtual bool initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<UavPluginCommonHandlers_t> common_handlers,
                          std::shared_ptr<UavPluginPrivateHandlers_t> private_handlers) = 0;

  /**
   * @brief Called once per simulation step. The plugin is expected to drive its UAV by calling
   *        setInput() on private_handlers->uav_system, if desired, during this call.
   *
   * @param dt the simulation step size [s]
   * @param sim_time the current simulation time
   * @param neighbors states of other UAVs within the current neighbor radius
   */
  virtual void update(const double dt, const rclcpp::Time &sim_time, const std::vector<UavPluginNeighborState_t> &neighbors) = 0;
};

//}

} // namespace mrs_multirotor_simulator

#endif // MRS_MULTIROTOR_SIMULATOR_UAV_PLUGIN_H
