/* includes //{ */

#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>

#include <mrs_multirotor_simulator/plugins/uav_plugin.h>

//}

namespace mrs_multirotor_simulator_plugins
{

/* class BoidsUavPlugin //{ */

/**
 * @brief A classic 3D boids flocking model (separation / alignment / cohesion), driving the
 *        uav through velocity-heading commands based on the states of nearby uavs.
 */
class BoidsUavPlugin : public mrs_multirotor_simulator::UavPlugin {

public:
  bool initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_multirotor_simulator::UavPluginCommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_multirotor_simulator::UavPluginPrivateHandlers_t> private_handlers) override;

  void update(const double dt, const rclcpp::Time &sim_time, const std::vector<mrs_multirotor_simulator::UavPluginNeighborState_t> &neighbors) override;

private:
  rclcpp::Node::SharedPtr                              node_;
  std::string                                          uav_name_;
  std::shared_ptr<mrs_multirotor_simulator::UavSystem> uav_system_;

  // | ------------------------- gains --------------------------- |

  double _separation_weight_;
  double _alignment_weight_;
  double _cohesion_weight_;
  double _max_speed_;
  double _perception_radius_;
  double _separation_radius_;

  // shared attractor pulling the flock's mean altitude back towards a common level, since
  // separation/alignment/cohesion alone have no restoring force against it drifting away
  double _cruise_altitude_;
  double _altitude_bias_weight_;
};

//}

/* initialize() //{ */

bool BoidsUavPlugin::initialize(const rclcpp::Node::SharedPtr                                                        &node,
                                [[maybe_unused]] std::shared_ptr<mrs_multirotor_simulator::UavPluginCommonHandlers_t> common_handlers,
                                std::shared_ptr<mrs_multirotor_simulator::UavPluginPrivateHandlers_t>                 private_handlers) {

  node_       = node;
  uav_name_   = private_handlers->uav_name;
  uav_system_ = private_handlers->uav_system;

  private_handlers->param_loader->loadParam("separation_weight", _separation_weight_, 1.5);
  private_handlers->param_loader->loadParam("alignment_weight", _alignment_weight_, 1.0);
  private_handlers->param_loader->loadParam("cohesion_weight", _cohesion_weight_, 1.0);
  private_handlers->param_loader->loadParam("max_speed", _max_speed_, 3.0);
  private_handlers->param_loader->loadParam("perception_radius", _perception_radius_, 10.0);
  private_handlers->param_loader->loadParam("separation_radius", _separation_radius_, 3.0);
  private_handlers->param_loader->loadParam("cruise_altitude", _cruise_altitude_, 5.0);
  private_handlers->param_loader->loadParam("altitude_bias_weight", _altitude_bias_weight_, 2.0);

  if (!private_handlers->param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[BoidsUavPlugin]: '%s' could not load all parameters!", uav_name_.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "[BoidsUavPlugin]: '%s' initialized", uav_name_.c_str());

  return true;
}

//}

/* update() //{ */

void BoidsUavPlugin::update([[maybe_unused]] const double dt, [[maybe_unused]] const rclcpp::Time &sim_time,
                            const std::vector<mrs_multirotor_simulator::UavPluginNeighborState_t> &neighbors) {

  const auto self_state = uav_system_->getState();

  Eigen::Vector3d separation = Eigen::Vector3d::Zero();
  Eigen::Vector3d alignment  = Eigen::Vector3d::Zero();
  Eigen::Vector3d cohesion   = Eigen::Vector3d::Zero();

  int n_flockmates = 0;

  for (const auto &neighbor : neighbors) {

    const Eigen::Vector3d rel_pos = self_state.x - neighbor.state.x;
    const double          dist    = rel_pos.norm();

    if (dist > _perception_radius_ || dist < 1e-3) {
      continue;
    }

    n_flockmates++;

    if (dist < _separation_radius_) {
      separation += rel_pos / (dist * dist);
    }

    alignment += neighbor.state.v;
    cohesion += neighbor.state.x;
  }

  // recomputed fresh each tick rather than accumulated onto the previous velocity, which
  // has no restoring term and drifts unboundedly under any small asymmetry
  Eigen::Vector3d desired_velocity = Eigen::Vector3d::Zero();

  if (n_flockmates > 0) {

    alignment /= n_flockmates;
    cohesion = (cohesion / n_flockmates) - self_state.x;

    // alignment steers towards the neighbors' average velocity, not adds it outright --
    // otherwise a fast-moving flock keeps re-commanding its own already-large velocity
    desired_velocity = _separation_weight_ * separation + _alignment_weight_ * (alignment - self_state.v) + _cohesion_weight_ * cohesion;
  }

  desired_velocity.z() += _altitude_bias_weight_ * (_cruise_altitude_ - self_state.x.z());

  // altitude gets first claim on the max_speed budget, horizontal gets whatever remains,
  // so total speed stays <= max_speed without the altitude correction being starved out
  double z_component = desired_velocity.z();

  if (std::abs(z_component) > _max_speed_) {
    z_component = std::copysign(_max_speed_, z_component);
  }

  const double    remaining_speed = std::sqrt(std::max(0.0, _max_speed_ * _max_speed_ - z_component * z_component));
  Eigen::Vector2d velocity_xy     = desired_velocity.head<2>();
  const double    speed_xy        = velocity_xy.norm();

  if (speed_xy > remaining_speed) {
    velocity_xy = velocity_xy * (remaining_speed / speed_xy);
  }

  mrs_multirotor_simulator::reference::VelocityHdg cmd;

  cmd.velocity.head<2>() = velocity_xy;
  cmd.velocity.z()       = z_component;

  if (velocity_xy.norm() > 0.1) {
    cmd.heading = atan2(velocity_xy.y(), velocity_xy.x());
  } else {
    cmd.heading = mrs_lib::AttitudeConverter(self_state.R).getHeading();
  }

  uav_system_->setInput(cmd);
}

//}

} // namespace mrs_multirotor_simulator_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_multirotor_simulator_plugins::BoidsUavPlugin, mrs_multirotor_simulator::UavPlugin)
