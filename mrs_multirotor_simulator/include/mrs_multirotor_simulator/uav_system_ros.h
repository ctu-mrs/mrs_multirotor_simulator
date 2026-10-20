#ifndef UAV_SYSTEM_ROS_H
#define UAV_SYSTEM_ROS_H

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>

#include <mrs_multirotor_simulator/uav_system/uav_system.hpp>

// #include <sensor_msgs/msg/imu.hpp>
// #include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mrs_msgs/msg/float64.hpp>
#include <mrs_msgs/srv/float64_srv.hpp>

// #include <mrs_msgs/msg/hw_api_actuator_cmd.hpp>
#include <mrs_msgs/msg/hw_api_control_group_cmd.hpp>
// #include <mrs_msgs/msg/hw_api_attitude_rate_cmd.hpp>
// #include <mrs_msgs/msg/hw_api_attitude_cmd.hpp>
// #include <mrs_msgs/msg/hw_api_acceleration_hdg_rate_cmd.hpp>
// #include <mrs_msgs/msg/hw_api_acceleration_hdg_cmd.hpp>
// #include <mrs_msgs/msg/hw_api_velocity_hdg_rate_cmd.hpp>
// #include <mrs_msgs/msg/hw_api_velocity_hdg_cmd.hpp>
// #include <mrs_msgs/msg/hw_api_position_cmd.hpp>
// #include <mrs_msgs/msg/tracker_command.hpp>

namespace mrs_multirotor_simulator
{

struct UavSystemRos_CommonHandlers_t
{

  rclcpp::Node::SharedPtr                                       node;
  std::string                                                   uav_name;
  std::optional<std::shared_ptr<mrs_lib::TransformBroadcaster>> transform_broadcaster;

};

class UavSystemRos {

public:
  UavSystemRos(const UavSystemRos_CommonHandlers_t common_handlers);

  void makeStep(const double dt, const double time_stamp);

  void crash(void);

  bool hasCrashed(void);

  // void applyForce(const Eigen::Vector3d &force);

  Eigen::Vector3d getPose(void);

  FixedwingModel::ModelParams getParams();
  FixedwingModel::State       getState();

  std::shared_ptr<UavSystem> getUavSystem(void) const;

  std::string getUavName(void) const;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::CallbackGroup::SharedPtr cbgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbgrp_ss_;

  rclcpp::Time time_stamp_;
  std::mutex   mutex_time_stamp_;

  std::atomic<bool> is_initialized_ = false;
  std::string       _uav_name_;

  std::shared_ptr<mrs_lib::ParamLoader> param_loader_;

  // double randd(double from, double to);

  // bool   _randomization_enabled_;
  // double _randomization_bounds_x_;
  // double _randomization_bounds_y_;
  // double _randomization_bounds_z_;

  // | ------------------------ UavSystem ----------------------- |

  std::shared_ptr<UavSystem> uav_system_;
  std::mutex                 mutex_uav_system_;

  rclcpp::Time time_last_input_;
  std::mutex   mutex_time_last_input_;

  FixedwingModel::ModelParams model_params_;

  // | ------------------------ uav plugins ------------------------ |


  bool   _iterate_without_input_;
  double _input_timeout_;

  std::string _frame_world_;
  std::string _frame_fcu_;
  // std::string _frame_rangefinder_;
  // bool        _publish_rangefinder_tf_;
  bool        _publish_fcu_tf_;

  // | ----------------------- publishers ----------------------- |

  // std::shared_ptr<mrs_lib::PublisherHandler<sensor_msgs::msg::Imu>>   ph_imu_;
  std::shared_ptr<mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>> ph_odom_;
  // std::shared_ptr<mrs_lib::PublisherHandler<sensor_msgs::msg::Range>> ph_rangefinder_;

  void publishOdometry(const FixedwingModel::State &state);
  void publishFCUTF(const FixedwingModel::State &state);
  // void publishIMU(const FixedwingModel::State &state);
  // void publishRangefinder(const FixedwingModel::State &state);

  void timeoutInput(void);

  // | --------------------------- tf --------------------------- |

  std::shared_ptr<mrs_lib::TransformBroadcaster>       tf_broadcaster_;

  // | ----------------------- subscribers ---------------------- |

  void callbackControlGroupCmd(const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg);
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiControlGroupCmd>        sh_control_group_cmd_;

  void callbackTeleportCmd(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> sh_teleport_cmd_;

  // | --------------------- service servers -------------------- |

  mrs_lib::ServiceServerHandler<mrs_msgs::srv::Float64Srv> ss_set_mass_;
  // mrs_lib::ServiceServerHandler<mrs_msgs::srv::Float64Srv> ss_set_ground_z_;

  bool callbackSetMass(const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request> request, const std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> response);

  // bool callbackSetGroundZ(const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request>  request,
  //                         const std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> response);

  // | ------------------------ routines ------------------------ |

  // void calculateInertia(FixedwingModel::ModelParams &params);
};

} // namespace mrs_multirotor_simulator

#endif // UAV_SYSTEM_ROS_H
