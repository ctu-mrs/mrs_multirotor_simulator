#ifndef MRS_MULTIROTOR_SIMULATOR_WORLD_PLUGIN_H
#define MRS_MULTIROTOR_SIMULATOR_WORLD_PLUGIN_H

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/param_loader.h>

#include <mrs_multirotor_simulator/uav_system/uav_system.hpp>

//}

namespace mrs_multirotor_simulator
{

/* struct WorldPluginCommonHandlers_t //{ */

/**
 * @brief Handlers shared by all loaded world plugins. Constructed once by the MultirotorSimulator node.
 */
struct WorldPluginCommonHandlers_t
{
  rclcpp::Node::SharedPtr node; ///< the parent simulator node

  /**
   * @brief full API access to every UAV in the simulation, stable for the lifetime of the node
   */
  std::vector<std::pair<std::string, std::shared_ptr<UavSystem>>> uavs;
};

//}

/* struct WorldPluginPrivateHandlers_t //{ */

/**
 * @brief Handlers provided individually to each world plugin instance.
 */
struct WorldPluginPrivateHandlers_t
{
  std::unique_ptr<mrs_lib::ParamLoader> param_loader;        ///< this plugin's own scoped param loader
  std::shared_ptr<mrs_lib::ParamLoader> parent_param_loader; ///< the simulator's param loader, for copyYamls()
  std::string                           runtime_name;        ///< the name under which this plugin was loaded
};

//}

/* class WorldPlugin //{ */

/**
 * @brief Abstract interface for a "world" plugin. A world plugin runs as a single instance
 *        per simulation and is given access to every UAV in the simulation, allowing it to
 *        manipulate the whole world (e.g., randomize UAV positions, spawn obstacles, ...).
 */
class WorldPlugin {

public:
  virtual ~WorldPlugin() = default;

  /**
   * @brief Initializes the plugin. Called once after all UAVs are spawned.
   *
   * @param node the sub-node given to this plugin instance
   * @param common_handlers handlers shared between all world plugins
   * @param private_handlers handlers provided individually to this plugin
   *
   * @return true if success
   */
  virtual bool initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<WorldPluginCommonHandlers_t> common_handlers,
                          std::shared_ptr<WorldPluginPrivateHandlers_t> private_handlers) = 0;

  /**
   * @brief Called once per simulation step.
   *
   * @param dt the simulation step size [s]
   * @param sim_time the current simulation time
   */
  virtual void update(const double dt, const rclcpp::Time &sim_time) = 0;
};

//}

} // namespace mrs_multirotor_simulator

#endif // MRS_MULTIROTOR_SIMULATOR_WORLD_PLUGIN_H
