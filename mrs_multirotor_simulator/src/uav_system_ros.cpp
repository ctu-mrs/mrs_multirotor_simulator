#include <mrs_multirotor_simulator/uav_system_ros.h>

namespace mrs_multirotor_simulator
{

/* UavSystemRos() //{ */

UavSystemRos::UavSystemRos(const UavSystemRos_CommonHandlers_t common_handlers) {

  node_ = common_handlers.node;

  cbgrp_subs_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbgrp_ss_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  _uav_name_ = common_handlers.uav_name;

  param_loader_ = std::make_shared<mrs_lib::ParamLoader>(node_, node_->get_name() + std::string("_") + _uav_name_);

  // load custom config

  std::string custom_config_path;
  param_loader_->loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    RCLCPP_INFO(node_->get_logger(), "[%s] loading custom config '%s", _uav_name_.c_str(), custom_config_path.c_str());
    param_loader_->addYamlFile(custom_config_path);
  }

  // load other configs

  std::vector<std::string> config_files;
  param_loader_->loadParam("uav_configs", config_files);

  for (auto config_file : config_files) {
    RCLCPP_INFO(node_->get_logger(), "[%s] loading config file '%s'", _uav_name_.c_str(), config_file.c_str());
    param_loader_->addYamlFile(config_file);
  }

  std::string type;
  param_loader_->loadParam(_uav_name_ + "/type", type);

  // | --------------------- general params --------------------- |

  param_loader_->loadParam("frames/world/name", _frame_world_);
  bool prefix_world_name;
  param_loader_->loadParam("frames/world/prefix_with_uav_name", prefix_world_name);

  if (prefix_world_name) {
    _frame_world_ = _uav_name_ + "/" + _frame_world_;
  }

  param_loader_->loadParam("frames/fcu/publish_tf", _publish_fcu_tf_);

  param_loader_->loadParam("frames/fcu/name", _frame_fcu_);

  _frame_fcu_ = _uav_name_ + "/" + _frame_fcu_;

  param_loader_->loadParam("g", model_params_.g);
  param_loader_->loadParam("iterate_without_input", _iterate_without_input_);
  param_loader_->loadParam("input_timeout", _input_timeout_);

  // | ------------------ model-specific params ----------------- |

  param_loader_->loadParam(type + "/mass", model_params_.m);
 
  // | --------------------- spawn location --------------------- |

  double spawn_x;
  double spawn_y;
  double spawn_z;
  double spawn_heading;

  param_loader_->loadParam(_uav_name_ + "/spawn/x", spawn_x);
  param_loader_->loadParam(_uav_name_ + "/spawn/y", spawn_y);
  param_loader_->loadParam(_uav_name_ + "/spawn/z", spawn_z);
  param_loader_->loadParam(_uav_name_ + "/spawn/heading", spawn_heading);

  Eigen::Vector3d spawn_pos(spawn_x, spawn_y, spawn_z);
  Eigen::Vector3d spawn_rot(0, 0, spawn_heading);
  Eigen::Vector3d spawn_vel(0, 0, 0); 
  uav_system_ = std::make_shared<UavSystem>(model_params_, spawn_pos, spawn_rot, spawn_vel);

  if (!param_loader_->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load all parameters");
    rclcpp::shutdown();
  }

  // | ----------------------- publishers ----------------------- |

  bool pub_odom_enabled;

  param_loader_->loadParam("publishers/odometry/enabled", pub_odom_enabled);

  if (pub_odom_enabled) {
    ph_odom_ = std::make_shared<mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>>(node_, "~/" + _uav_name_ + "/odom");
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.node_name                           = _uav_name_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbgrp_subs_;
  shopts.qos                                 = rclcpp::SystemDefaultsQoS();

  // 
  bool sub_control_group_cmd_enabled;
  
  param_loader_->loadParam("subscribers/control_group_cmd/enabled", sub_control_group_cmd_enabled);

  if (sub_control_group_cmd_enabled) {
    sh_control_group_cmd_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiControlGroupCmd>(shopts, "~/" + _uav_name_ + "/control_group_cmd",
                                                                                            &UavSystemRos::callbackControlGroupCmd, this);
  }

  // | --------------------- tf broadcaster --------------------- |

  if (common_handlers.transform_broadcaster) {
    tf_broadcaster_ = common_handlers.transform_broadcaster.value();
  } else {
    tf_broadcaster_ = std::make_shared<mrs_lib::TransformBroadcaster>(node_);
  }


  // | --------------------- service servers -------------------- |

  ss_set_mass_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::Float64Srv>(
      node_, "~/" + _uav_name_ + "/set_mass", std::bind(&UavSystemRos::callbackSetMass, this, std::placeholders::_1, std::placeholders::_2), cbgrp_ss_);

  // | ------------------ first model iteration ----------------- |

  // * we need to iterate the model first to initialize its state
  // * this needs to happen in order to publish the correct state
  //   when using iterate_without_input == false

  // iterate the model twise to initialize all the states
  uav_system_->makeStep(0.01);
  uav_system_->makeStep(0.01);

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

/* makeStep() //{ */

void UavSystemRos::makeStep(const double dt, const double time_stamp) {

  mrs_lib::set_mutexed(mutex_time_stamp_, rclcpp::Time(static_cast<int64_t>(time_stamp * 1e9), RCL_ROS_TIME), time_stamp_);

    // | ---------------- check timeout of an input --------------- |

    auto time_last_input = mrs_lib::get_mutexed(mutex_time_last_input_, time_last_input_);

    if (time_last_input.seconds() > 0) {

      if (time_stamp - time_last_input.seconds() > _input_timeout_) {

        RCLCPP_WARN(node_->get_logger(), "input timeouted");

        time_last_input = rclcpp::Time(0.0);

        mrs_lib::set_mutexed(mutex_time_last_input_, time_last_input, time_last_input_);
      }
    }

    // | --------------------- model iteration -------------------- |

    if (_iterate_without_input_ || time_last_input.seconds() > 0) {

      std::scoped_lock lock(mutex_uav_system_);

      // iterate the model
      uav_system_->makeStep(dt);
    }


  // extract the current state
  FixedwingModel::State state = uav_system_->getState();

  // publish data

  publishFCUTF(state);

  publishOdometry(state);

}

//}

/* getPose() //{ */

Eigen::Vector3d UavSystemRos::getPose(void) {

  return uav_system_->getState().x;
}

//}

/* getParams() //{ */

FixedwingModel::ModelParams UavSystemRos::getParams() {

  return uav_system_->getParams();
}

//}

/* getUavSystem() //{ */

std::shared_ptr<UavSystem> UavSystemRos::getUavSystem(void) const {

  return uav_system_;
}

//}

/* getUavName() //{ */

std::string UavSystemRos::getUavName(void) const {

  return _uav_name_;
}

//}

/* getState() //{ */

FixedwingModel::State UavSystemRos::getState() {

  return uav_system_->getState();
}

//}

/* crash() //{ */

void UavSystemRos::crash(void) {
  uav_system_->crash();
}

//}

/* hasCrashed() //{ */

bool UavSystemRos::hasCrashed(void) {
  return uav_system_->hasCrashed();
}

//}


// | ----------------------- publishers ----------------------- |

/* publishOdometry() //{ */

void UavSystemRos::publishOdometry(const FixedwingModel::State &state) {

  if (!ph_odom_) {
    return;
  }

  nav_msgs::msg::Odometry odom;

  odom.header.stamp    = time_stamp_;
  odom.header.frame_id = _frame_world_;
  odom.child_frame_id  = _frame_fcu_;

  odom.pose.pose.orientation = mrs_lib::AttitudeConverter(state.R);

  odom.pose.pose.position.x = state.x(0);
  odom.pose.pose.position.y = state.x(1);
  odom.pose.pose.position.z = state.x(2);

  Eigen::Vector3d vel_body = state.R.transpose() * state.v;

  odom.twist.twist.linear.x = vel_body(0);
  odom.twist.twist.linear.y = vel_body(1);
  odom.twist.twist.linear.z = vel_body(2);

  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = 0;

  ph_odom_->publish(odom);
}

//}

/* publishFCUTF() //{ */

void UavSystemRos::publishFCUTF(const FixedwingModel::State &state) {

  if (_publish_fcu_tf_) {

    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp    = time_stamp_;
    tf.header.frame_id = _frame_world_;
    tf.child_frame_id  = _frame_fcu_;

    tf.transform.translation.x = state.x(0);
    tf.transform.translation.y = state.x(1);
    tf.transform.translation.z = state.x(2);

    tf.transform.rotation = mrs_lib::AttitudeConverter(state.R);

    tf_broadcaster_->sendTransform(tf);
  }
}

//}


// | ------------------------ callbacks ----------------------- |

/* callbackControlGroupCmd() //{ */

void UavSystemRos::callbackControlGroupCmd(const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting control group command");

  {
    std::scoped_lock lock(mutex_uav_system_);

    uav_system_->setInput(msg->pitch, msg->roll, msg->throttle);
  }

  {
    std::scoped_lock lock(mutex_time_last_input_);

    time_last_input_ = time_stamp_;
  }
}

//}

/* callbackSetMass() //{ */

bool UavSystemRos::callbackSetMass(const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request>  request,
                                   const std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  {
    std::scoped_lock lock(mutex_uav_system_);

    model_params_ = uav_system_->getParams();

    model_params_.m = request->value;

    uav_system_->setParams(model_params_);
  }

  response->success = true;
  response->message = "mass set";

  return true;
}

//}


} // namespace mrs_multirotor_simulator