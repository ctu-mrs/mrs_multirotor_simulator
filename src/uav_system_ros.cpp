#include <uav_system_ros.h>

namespace mrs_multirotor_simulator
{

/* UavSystemRos() //{ */

UavSystemRos::UavSystemRos(const UavSystemRos_CommonHandlers_t common_handlers) {

  node_ = common_handlers.node;

  cbgrp_subs_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbgrp_ss_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  _uav_name_ = common_handlers.uav_name;

  mrs_lib::ParamLoader param_loader(node_, node_->get_name() + std::string("_") + _uav_name_);

  // load custom config

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    RCLCPP_INFO(node_->get_logger(), "[%s] loading custom config '%s", _uav_name_.c_str(), custom_config_path.c_str());
    param_loader.addYamlFile(custom_config_path);
  }

  // load other configs

  std::vector<std::string> config_files;
  param_loader.loadParam("uav_configs", config_files);

  for (auto config_file : config_files) {
    RCLCPP_INFO(node_->get_logger(), "[%s] loading config file '%s'", _uav_name_.c_str(), config_file.c_str());
    param_loader.addYamlFile(config_file);
  }

  std::string type;
  param_loader.loadParam(_uav_name_ + "/type", type);

  // | --------------------- general params --------------------- |

  param_loader.loadParam("frames/world/name", _frame_world_);
  bool prefix_world_name;
  param_loader.loadParam("frames/world/prefix_with_uav_name", prefix_world_name);

  if (prefix_world_name) {
    _frame_world_ = _uav_name_ + "/" + _frame_world_;
  }

  param_loader.loadParam("frames/rangefinder/name", _frame_rangefinder_);

  _frame_rangefinder_ = _uav_name_ + "/" + _frame_rangefinder_;

  param_loader.loadParam("frames/rangefinder/publish_tf", _publish_rangefinder_tf_);
  param_loader.loadParam("frames/fcu/publish_tf", _publish_fcu_tf_);

  param_loader.loadParam("frames/fcu/name", _frame_fcu_);

  _frame_fcu_ = _uav_name_ + "/" + _frame_fcu_;

  param_loader.loadParam("g", model_params_.g);
  param_loader.loadParam("iterate_without_input", _iterate_without_input_);
  param_loader.loadParam("input_timeout", _input_timeout_);
  param_loader.loadParam("ground/enabled", model_params_.ground_enabled);
  param_loader.loadParam("ground/z", model_params_.ground_z);
  param_loader.loadParam("individual_takeoff_platform/enabled", model_params_.takeoff_patch_enabled);

  // | ------------------ model-specific params ----------------- |

  param_loader.loadParam(type + "/n_motors", model_params_.n_motors);
  param_loader.loadParam(type + "/mass", model_params_.mass);
  param_loader.loadParam(type + "/arm_length", model_params_.arm_length);
  param_loader.loadParam(type + "/body_height", model_params_.body_height);
  param_loader.loadParam(type + "/air_resistance_coeff", model_params_.air_resistance_coeff);
  param_loader.loadParam(type + "/motor_time_constant", model_params_.motor_time_constant);
  param_loader.loadParam(type + "/propulsion/prop_radius", model_params_.prop_radius);
  param_loader.loadParam(type + "/propulsion/force_constant", model_params_.kf);
  param_loader.loadParam(type + "/propulsion/moment_constant", model_params_.km);
  param_loader.loadParam(type + "/propulsion/rpm/min", model_params_.min_rpm);
  param_loader.loadParam(type + "/propulsion/rpm/max", model_params_.max_rpm);

  // | --------------------- spawn location --------------------- |

  double spawn_x;
  double spawn_y;
  double spawn_z;
  double spawn_heading;

  param_loader.loadParam(_uav_name_ + "/spawn/x", spawn_x);
  param_loader.loadParam(_uav_name_ + "/spawn/y", spawn_y);
  param_loader.loadParam(_uav_name_ + "/spawn/z", spawn_z);
  param_loader.loadParam(_uav_name_ + "/spawn/heading", spawn_heading);

  param_loader.loadParam("randomization/enabled", _randomization_enabled_);
  param_loader.loadParam("randomization/bounds/x", _randomization_bounds_x_);
  param_loader.loadParam("randomization/bounds/y", _randomization_bounds_y_);
  param_loader.loadParam("randomization/bounds/z", _randomization_bounds_z_);

  if (_randomization_enabled_) {
    spawn_x += randd(-_randomization_bounds_x_, _randomization_bounds_x_);
    spawn_y += randd(-_randomization_bounds_y_, _randomization_bounds_y_);
    spawn_z += randd(-_randomization_bounds_z_, _randomization_bounds_z_);
    spawn_heading += randd(-3.14, 3.14);
  }

  calculateInertia(model_params_);

  model_params_.allocation_matrix = param_loader.loadMatrixDynamic2(type + "/propulsion/allocation_matrix", 4, -1);

  model_params_.allocation_matrix.row(0) *= model_params_.arm_length * model_params_.kf;
  model_params_.allocation_matrix.row(1) *= model_params_.arm_length * model_params_.kf;
  model_params_.allocation_matrix.row(2) *= model_params_.km * (3.0 * model_params_.prop_radius) * model_params_.kf;
  model_params_.allocation_matrix.row(3) *= model_params_.kf;

  uav_system_ = UavSystem(model_params_, Eigen::Vector3d(spawn_x, spawn_y, spawn_z), spawn_heading);

  // | -------------------------- mixer ------------------------- |

  Mixer::Params mixer_params;

  param_loader.loadParam("mixer/desaturation", mixer_params.desaturation);

  uav_system_.setMixerParams(mixer_params);

  // | --------------------- rate controller -------------------- |

  RateController::Params rate_controller_params;

  param_loader.loadParam("rate_controller/kp", rate_controller_params.kp);
  param_loader.loadParam("rate_controller/kd", rate_controller_params.kd);
  param_loader.loadParam("rate_controller/ki", rate_controller_params.ki);

  uav_system_.setRateControllerParams(rate_controller_params);

  // | --------------------- attitude controller -------------------- |

  AttitudeController::Params attitude_controller_params;

  param_loader.loadParam("attitude_controller/kp", attitude_controller_params.kp);
  param_loader.loadParam("attitude_controller/kd", attitude_controller_params.kd);
  param_loader.loadParam("attitude_controller/ki", attitude_controller_params.ki);
  param_loader.loadParam("attitude_controller/max_rate_roll_pitch", attitude_controller_params.max_rate_roll_pitch);
  param_loader.loadParam("attitude_controller/max_rate_yaw", attitude_controller_params.max_rate_yaw);

  uav_system_.setAttitudeControllerParams(attitude_controller_params);

  // | ------------------- velocity controller ------------------ |

  VelocityController::Params velocity_controller_params;

  param_loader.loadParam("velocity_controller/kp", velocity_controller_params.kp);
  param_loader.loadParam("velocity_controller/kd", velocity_controller_params.kd);
  param_loader.loadParam("velocity_controller/ki", velocity_controller_params.ki);
  param_loader.loadParam("velocity_controller/max_acceleration", velocity_controller_params.max_acceleration);

  uav_system_.setVelocityControllerParams(velocity_controller_params);

  // | ------------------- position controller ------------------ |

  PositionController::Params position_controller_params;

  param_loader.loadParam("position_controller/kp", position_controller_params.kp);
  param_loader.loadParam("position_controller/kd", position_controller_params.kd);
  param_loader.loadParam("position_controller/ki", position_controller_params.ki);
  param_loader.loadParam("position_controller/max_velocity", position_controller_params.max_velocity);

  uav_system_.setPositionControllerParams(position_controller_params);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load all parameters");
    rclcpp::shutdown();
  }

  // | ----------------------- publishers ----------------------- |

  bool pub_imu_enabled;
  bool pub_odom_enabled;
  bool pub_rangefinder_enabled;

  param_loader.loadParam("publishers/imu/enabled", pub_imu_enabled);
  param_loader.loadParam("publishers/rangefinder/enabled", pub_rangefinder_enabled);
  param_loader.loadParam("publishers/odometry/enabled", pub_odom_enabled);

  if (pub_imu_enabled) {
    ph_imu_ = std::make_shared<mrs_lib::PublisherHandler<sensor_msgs::msg::Imu>>(node_, "~/" + _uav_name_ + "/imu");
  }

  if (pub_odom_enabled) {
    ph_odom_ = std::make_shared<mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>>(node_, "~/" + _uav_name_ + "/odom");
  }

  if (pub_rangefinder_enabled) {
    ph_rangefinder_ = std::make_shared<mrs_lib::PublisherHandler<sensor_msgs::msg::Range>>(node_, "~/" + _uav_name_ + "/rangefinder");
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

  bool sub_tracker_cmd_enabled;
  bool sub_pos_cmd_enabled;
  bool sub_vel_hdg_cmd_enabled;
  bool sub_vel_hdg_rate_cmd_enabled;
  bool sub_acc_hdg_cmd_enabled;
  bool sub_acc_hdg_rate_cmd_enabled;
  bool sub_attitude_cmd_enabled;
  bool sub_attitude_rate_cmd_enabled;
  bool sub_control_group_cmd_enabled;
  bool sub_actuators_cmd_enabled;

  param_loader.loadParam("subscribers/tracker_cmd/enabled", sub_tracker_cmd_enabled);
  param_loader.loadParam("subscribers/position_cmd/enabled", sub_pos_cmd_enabled);
  param_loader.loadParam("subscribers/velocity_hdg_cmd/enabled", sub_vel_hdg_cmd_enabled);
  param_loader.loadParam("subscribers/velocity_hdg_rate_cmd/enabled", sub_vel_hdg_rate_cmd_enabled);
  param_loader.loadParam("subscribers/acceleration_hdg_cmd/enabled", sub_acc_hdg_cmd_enabled);
  param_loader.loadParam("subscribers/acceleration_hdg_rate_cmd/enabled", sub_acc_hdg_rate_cmd_enabled);
  param_loader.loadParam("subscribers/attitude_cmd/enabled", sub_attitude_cmd_enabled);
  param_loader.loadParam("subscribers/attitude_rate_cmd/enabled", sub_attitude_rate_cmd_enabled);
  param_loader.loadParam("subscribers/control_group_cmd/enabled", sub_control_group_cmd_enabled);
  param_loader.loadParam("subscribers/actuators_group_cmd/enabled", sub_actuators_cmd_enabled);

  if (sub_actuators_cmd_enabled) {
    sh_actuator_cmd_ =
        mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiActuatorCmd>(shopts, "~/" + _uav_name_ + "/actuators_cmd", &UavSystemRos::callbackActuatorCmd, this);
  }

  if (sub_control_group_cmd_enabled) {
    sh_control_group_cmd_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiControlGroupCmd>(shopts, "~/" + _uav_name_ + "/control_group_cmd",
                                                                                            &UavSystemRos::callbackControlGroupCmd, this);
  }

  if (sub_attitude_rate_cmd_enabled) {
    sh_attitude_rate_cmd_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiAttitudeRateCmd>(shopts, "~/" + _uav_name_ + "/attitude_rate_cmd",
                                                                                            &UavSystemRos::callbackAttitudeRateCmd, this);
  }

  if (sub_attitude_cmd_enabled) {
    sh_attitude_cmd_ =
        mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiAttitudeCmd>(shopts, "~/" + _uav_name_ + "/attitude_cmd", &UavSystemRos::callbackAttitudeCmd, this);
  }

  if (sub_acc_hdg_cmd_enabled) {
    sh_acceleration_hdg_cmd_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiAccelerationHdgCmd>(shopts, "~/" + _uav_name_ + "/acceleration_hdg_cmd",
                                                                                                  &UavSystemRos::callbackAccelerationHdgCmd, this);
  }

  if (sub_acc_hdg_rate_cmd_enabled) {
    sh_acceleration_hdg_rate_cmd_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiAccelerationHdgRateCmd>(
        shopts, "~/" + _uav_name_ + "/acceleration_hdg_rate_cmd", &UavSystemRos::callbackAccelerationHdgRateCmd, this);
  }

  if (sub_vel_hdg_cmd_enabled) {
    sh_velocity_hdg_cmd_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiVelocityHdgCmd>(shopts, "~/" + _uav_name_ + "/velocity_hdg_cmd",
                                                                                          &UavSystemRos::callbackVelocityHdgCmd, this);
  }

  if (sub_vel_hdg_rate_cmd_enabled) {
    sh_velocity_hdg_rate_cmd_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiVelocityHdgRateCmd>(shopts, "~/" + _uav_name_ + "/velocity_hdg_rate_cmd",
                                                                                                   &UavSystemRos::callbackVelocityHdgRateCmd, this);
  }

  if (sub_pos_cmd_enabled) {
    sh_position_cmd_ =
        mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiPositionCmd>(shopts, "~/" + _uav_name_ + "/position_cmd", &UavSystemRos::callbackPositionCmd, this);
  }

  if (sub_tracker_cmd_enabled) {
    sh_tracker_cmd_ =
        mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>(shopts, "~/" + _uav_name_ + "/tracker_cmd", &UavSystemRos::callbackTrackerCmd, this);
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

  ss_set_ground_z_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::Float64Srv>(
      node_, "~/" + _uav_name_ + "/set_ground_z", std::bind(&UavSystemRos::callbackSetGroundZ, this, std::placeholders::_1, std::placeholders::_2), cbgrp_ss_);

  // | ------------------ first model iteration ----------------- |

  // * we need to iterate the model first to initialize its state
  // * this needs to happen in order to publish the correct state
  //   when using iterate_without_input == false

  reference::Actuators actuators_cmd;

  actuators_cmd.motors = Eigen::VectorXd::Zero(model_params_.n_motors);

  // set the motor input for the model
  uav_system_.setInput(actuators_cmd);

  // iterate the model twise to initialize all the states
  uav_system_.makeStep(0.01);
  uav_system_.makeStep(0.01);

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

      timeoutInput();

      time_last_input = rclcpp::Time(0.0);

      mrs_lib::set_mutexed(mutex_time_last_input_, time_last_input, time_last_input_);
    }
  }

  // | --------------------- model iteration -------------------- |

  if (_iterate_without_input_ || time_last_input.seconds() > 0) {

    std::scoped_lock lock(mutex_uav_system_);

    // iterate the model
    uav_system_.makeStep(dt);
  }

  // extract the current state
  MultirotorModel::State state = uav_system_.getState();

  // publish data

  publishFCUTF(state);

  publishOdometry(state);

  publishIMU(state);

  publishRangefinder(state);
}

//}

/* getPose() //{ */

Eigen::Vector3d UavSystemRos::getPose(void) {

  return uav_system_.getState().x;
}

//}

/* getParams() //{ */

MultirotorModel::ModelParams UavSystemRos::getParams() {

  return uav_system_.getParams();
}

//}

/* getState() //{ */

MultirotorModel::State UavSystemRos::getState() {

  return uav_system_.getState();
}

//}

/* crash() //{ */

void UavSystemRos::crash(void) {
  uav_system_.crash();
}

//}

/* hasCrashed() //{ */

bool UavSystemRos::hasCrashed(void) {
  return uav_system_.hasCrashed();
}

//}

/* applyForce() //{ */

void UavSystemRos::applyForce(const Eigen::Vector3d &force) {
  uav_system_.applyForce(force);
}

//}

// | ----------------------- publishers ----------------------- |

/* publishOdometry() //{ */

void UavSystemRos::publishOdometry(const MultirotorModel::State &state) {

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

  odom.twist.twist.angular.x = state.omega(0);
  odom.twist.twist.angular.y = state.omega(1);
  odom.twist.twist.angular.z = state.omega(2);

  ph_odom_->publish(odom);
}

//}

/* publishFCUTF() //{ */

void UavSystemRos::publishFCUTF(const MultirotorModel::State &state) {

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

/* publishIMU() //{ */

void UavSystemRos::publishIMU(const MultirotorModel::State &state) {

  if (!ph_imu_) {
    return;
  }

  sensor_msgs::msg::Imu imu;

  imu.header.stamp    = time_stamp_;
  imu.header.frame_id = _frame_fcu_;

  imu.angular_velocity.x = state.omega(0);
  imu.angular_velocity.y = state.omega(1);
  imu.angular_velocity.z = state.omega(2);

  auto acc = uav_system_.getImuAcceleration();

  imu.linear_acceleration.x = acc(0);
  imu.linear_acceleration.y = acc(1);
  imu.linear_acceleration.z = acc(2);

  imu.orientation = mrs_lib::AttitudeConverter(state.R);

  ph_imu_->publish(imu);
}

//}

/* publishRangefinder() //{ */

void UavSystemRos::publishRangefinder(const MultirotorModel::State &state) {

  if (!ph_rangefinder_) {
    return;
  }

  // | ----------------------- publish tf ----------------------- |

  const Eigen::Vector3d body_z          = state.R.col(2);
  const Eigen::Vector3d rangefinder_dir = -body_z;

  // calculate the angle between the drone's z axis and the world's z axis
  double tilt = acos(rangefinder_dir.dot(Eigen::Vector3d(0, 0, -1)));

  double range_measurement;

  if (body_z(2) > 0) {
    range_measurement = (state.x(2) - model_params_.ground_z) / cos(tilt) + 0.01;
  } else {
    range_measurement = std::numeric_limits<double>::max();
  }

  if (range_measurement > 40.0) {
    range_measurement = 41.0;
  }

  sensor_msgs::msg::Range range;

  range.header.frame_id = _frame_rangefinder_;
  range.header.stamp    = time_stamp_;
  range.max_range       = 40.0;
  range.min_range       = 0.0;
  range.range           = range_measurement;
  range.radiation_type  = range.INFRARED;
  range.field_of_view   = 0.01;

  ph_rangefinder_->publish(range);

  if (_publish_rangefinder_tf_) {

    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp    = time_stamp_;
    tf.header.frame_id = _frame_fcu_;
    tf.child_frame_id  = _frame_rangefinder_;

    tf.transform.translation.x = 0;
    tf.transform.translation.y = 0;
    tf.transform.translation.z = -0.05;

    tf.transform.rotation = mrs_lib::AttitudeConverter(0, 1.57, 0);

    tf_broadcaster_->sendTransform(tf);
  }
}

//}

// | ------------------------ routines ------------------------ |

/* timeoutInput() //{ */

void UavSystemRos::timeoutInput(void) {

  auto last_input_mode = mrs_lib::get_mutexed(mutex_time_last_input_, last_input_mode_);

  MultirotorModel::State state = uav_system_.getState();

  switch (last_input_mode) {

  case UavSystem::POSITION_CMD: {

    reference::Position cmd;

    cmd.position = state.x;
    cmd.heading  = mrs_lib::AttitudeConverter(state.R).getHeading();

    {
      std::scoped_lock lock(mutex_uav_system_);
      uav_system_.setInput(cmd);
    }

    break;
  }

  case UavSystem::VELOCITY_HDG_CMD: {

    reference::VelocityHdg cmd;

    cmd.velocity = Eigen::Vector3d(0, 0, 0);
    cmd.heading  = mrs_lib::AttitudeConverter(state.R).getHeading();

    {
      std::scoped_lock lock(mutex_uav_system_);
      uav_system_.setInput(cmd);
    }

    break;
  }

  case UavSystem::VELOCITY_HDG_RATE_CMD: {

    reference::VelocityHdgRate cmd;

    cmd.velocity     = Eigen::Vector3d(0, 0, 0);
    cmd.heading_rate = 0;

    {
      std::scoped_lock lock(mutex_uav_system_);
      uav_system_.setInput(cmd);
    }

    break;
  }

  case UavSystem::ACCELERATION_HDG_CMD: {

    reference::AccelerationHdg cmd;

    cmd.acceleration = Eigen::Vector3d(0, 0, 0);
    cmd.heading      = mrs_lib::AttitudeConverter(state.R).getHeading();

    {
      std::scoped_lock lock(mutex_uav_system_);
      uav_system_.setInput(cmd);
    }

    break;
  }

  case UavSystem::ACCELERATION_HDG_RATE_CMD: {

    reference::AccelerationHdgRate cmd;

    cmd.acceleration = Eigen::Vector3d(0, 0, 0);
    cmd.heading_rate = 0;

    {
      std::scoped_lock lock(mutex_uav_system_);
      uav_system_.setInput(cmd);
    }

    break;
  }

  case UavSystem::ATTITUDE_CMD: {

    reference::Attitude cmd;

    double heading = mrs_lib::AttitudeConverter(state.R).getHeading();

    cmd.orientation = mrs_lib::AttitudeConverter(0, 0, heading);
    cmd.throttle    = 0.0;

    {
      std::scoped_lock lock(mutex_uav_system_);
      uav_system_.setInput(cmd);
    }

    break;
  }

  case UavSystem::TILT_HDG_RATE_CMD: {

    reference::TiltHdgRate cmd;

    cmd.tilt_vector = Eigen::Vector3d(0, 0, 1);
    cmd.throttle    = 0.0;

    {
      std::scoped_lock lock(mutex_uav_system_);
      uav_system_.setInput(cmd);
    }

    break;
  }

  case UavSystem::ATTITUDE_RATE_CMD: {

    reference::AttitudeRate cmd;

    cmd.rate_x   = 0;
    cmd.rate_y   = 0;
    cmd.rate_z   = 0;
    cmd.throttle = 0.0;

    {
      std::scoped_lock lock(mutex_uav_system_);
      uav_system_.setInput(cmd);
    }

    break;
  }

  case UavSystem::CONTROL_GROUP_CMD: {

    reference::ControlGroup cmd;

    cmd.roll     = 0;
    cmd.pitch    = 0;
    cmd.yaw      = 0;
    cmd.throttle = 0.0;

    {
      std::scoped_lock lock(mutex_uav_system_);
      uav_system_.setInput(cmd);
    }

    break;
  }

  case UavSystem::ACTUATOR_CMD: {

    reference::Actuators cmd;

    cmd.motors = Eigen::VectorXd::Zero(model_params_.n_motors);

    {
      std::scoped_lock lock(mutex_uav_system_);
      uav_system_.setInput(cmd);
    }

    break;
  }

  case UavSystem::INPUT_UNKNOWN: {

    {
      std::scoped_lock lock(mutex_uav_system_);
      uav_system_.setInput();
    }

    break;
  }
  }
}

//}

/* randd() //{ */

double UavSystemRos::randd(double from, double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return floor(to - from) * zero_to_one + from;
}

//}

/* calculateInertia() //{ */

void UavSystemRos::calculateInertia(MultirotorModel::ModelParams &params) {

  // create the inertia matrix
  params.J       = Eigen::Matrix3d::Zero();
  params.J(0, 0) = params.mass * (3.0 * params.arm_length * params.arm_length + params.body_height * params.body_height) / 12.0;
  params.J(1, 1) = params.mass * (3.0 * params.arm_length * params.arm_length + params.body_height * params.body_height) / 12.0;
  params.J(2, 2) = (params.mass * params.arm_length * params.arm_length) / 2.0;
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackActuatorCmd() //{ */

void UavSystemRos::callbackActuatorCmd(const mrs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting actuator command");

  if (model_params_.n_motors != int(msg->motors.size())) {
    RCLCPP_ERROR(node_->get_logger(), "the actuators message controls %d motors, but the model has %d motors", int(msg->motors.size()), model_params_.n_motors);
    return;
  }

  reference::Actuators cmd;

  cmd.motors = Eigen::VectorXd::Zero(model_params_.n_motors);

  for (int i = 0; i < model_params_.n_motors; i++) {
    cmd.motors(i) = msg->motors.at(i);
  }

  {
    std::scoped_lock lock(mutex_uav_system_);

    uav_system_.setInput(cmd);
  }

  {
    std::scoped_lock lock(mutex_time_last_input_, mutex_time_stamp_);

    time_last_input_ = rclcpp::Time(time_stamp_);
    last_input_mode_ = UavSystem::ACTUATOR_CMD;
  }
}

//}

/* callbackControlGroupCmd() //{ */

void UavSystemRos::callbackControlGroupCmd(const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting control group command");

  reference::ControlGroup cmd;

  cmd.throttle = msg->throttle;
  cmd.roll     = msg->roll;
  cmd.pitch    = msg->pitch;
  cmd.yaw      = msg->yaw;

  {
    std::scoped_lock lock(mutex_uav_system_);

    uav_system_.setInput(cmd);
  }

  {
    std::scoped_lock lock(mutex_time_last_input_);

    time_last_input_ = time_stamp_;
    last_input_mode_ = UavSystem::CONTROL_GROUP_CMD;
  }
}

//}

/* callbackAttitudeRateCmd() //{ */

void UavSystemRos::callbackAttitudeRateCmd(const mrs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting attitude rate command");

  reference::AttitudeRate cmd;

  cmd.throttle = msg->throttle;
  cmd.rate_x   = msg->body_rate.x;
  cmd.rate_y   = msg->body_rate.y;
  cmd.rate_z   = msg->body_rate.z;

  {
    std::scoped_lock lock(mutex_uav_system_);

    uav_system_.setInput(cmd);
  }

  {
    std::scoped_lock lock(mutex_time_last_input_);

    time_last_input_ = time_stamp_;
    last_input_mode_ = UavSystem::ATTITUDE_RATE_CMD;
  }
}

//}

/* callbackAttitudeCmd() //{ */

void UavSystemRos::callbackAttitudeCmd(const mrs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting attitude command");

  reference::Attitude cmd;

  cmd.throttle = msg->throttle;

  cmd.orientation = mrs_lib::AttitudeConverter(msg->orientation);

  {
    std::scoped_lock lock(mutex_uav_system_);

    uav_system_.setInput(cmd);
  }

  {
    std::scoped_lock lock(mutex_time_last_input_);

    time_last_input_ = time_stamp_;
    last_input_mode_ = UavSystem::ATTITUDE_CMD;
  }
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

void UavSystemRos::callbackAccelerationHdgRateCmd(const mrs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting acceleration+hdg rate command");

  reference::AccelerationHdgRate cmd;

  cmd.heading_rate = msg->heading_rate;

  cmd.acceleration(0) = msg->acceleration.x;
  cmd.acceleration(1) = msg->acceleration.y;
  cmd.acceleration(2) = msg->acceleration.z;

  {
    std::scoped_lock lock(mutex_uav_system_);

    uav_system_.setInput(cmd);
  }

  {
    std::scoped_lock lock(mutex_time_last_input_);

    time_last_input_ = time_stamp_;
    last_input_mode_ = UavSystem::ACCELERATION_HDG_RATE_CMD;
  }
}

//}

/* callbackAccelerationHdgCmd() //{ */

void UavSystemRos::callbackAccelerationHdgCmd(const mrs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting acceleration+hdg command");

  reference::AccelerationHdg cmd;

  cmd.heading = msg->heading;

  cmd.acceleration(0) = msg->acceleration.x;
  cmd.acceleration(1) = msg->acceleration.y;
  cmd.acceleration(2) = msg->acceleration.z;

  {
    std::scoped_lock lock(mutex_uav_system_);

    uav_system_.setInput(cmd);
  }

  {
    std::scoped_lock lock(mutex_time_last_input_);

    time_last_input_ = time_stamp_;
    last_input_mode_ = UavSystem::ACCELERATION_HDG_CMD;
  }
}

//}

/* callbackVelocityHdgRateCmd() //{ */

void UavSystemRos::callbackVelocityHdgRateCmd(const mrs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting velocity+hdg rate command");

  reference::VelocityHdgRate cmd;

  cmd.heading_rate = msg->heading_rate;

  cmd.velocity(0) = msg->velocity.x;
  cmd.velocity(1) = msg->velocity.y;
  cmd.velocity(2) = msg->velocity.z;

  {
    std::scoped_lock lock(mutex_uav_system_);

    uav_system_.setInput(cmd);
  }

  {
    std::scoped_lock lock(mutex_time_last_input_);

    time_last_input_ = time_stamp_;
    last_input_mode_ = UavSystem::VELOCITY_HDG_RATE_CMD;
  }
}

//}

/* callbackVelocityHdgCmd() //{ */

void UavSystemRos::callbackVelocityHdgCmd(const mrs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting velocity+hdg command");

  reference::VelocityHdg cmd;

  cmd.heading = msg->heading;

  cmd.velocity(0) = msg->velocity.x;
  cmd.velocity(1) = msg->velocity.y;
  cmd.velocity(2) = msg->velocity.z;

  {
    std::scoped_lock lock(mutex_uav_system_);

    uav_system_.setInput(cmd);
  }

  {
    std::scoped_lock lock(mutex_time_last_input_);

    time_last_input_ = time_stamp_;
    last_input_mode_ = UavSystem::VELOCITY_HDG_CMD;
  }
}

//}

/* callbackPositionCmd() //{ */

void UavSystemRos::callbackPositionCmd(const mrs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting position command");

  reference::Position cmd;

  cmd.heading = msg->heading;

  cmd.position(0) = msg->position.x;
  cmd.position(1) = msg->position.y;
  cmd.position(2) = msg->position.z;

  {
    std::scoped_lock lock(mutex_uav_system_);

    uav_system_.setInput(cmd);
  }

  {
    std::scoped_lock lock(mutex_time_last_input_);

    time_last_input_ = time_stamp_;
    last_input_mode_ = UavSystem::POSITION_CMD;
  }
}

//}

/* callbackTrackerCmd() //{ */

void UavSystemRos::callbackTrackerCmd(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting tracker command");

  Eigen::Vector3d velocity(0, 0, 0);
  Eigen::Vector3d acceleration(0, 0, 0);
  double          heading_rate = 0;

  if (msg->use_velocity_horizontal) {
    velocity(0) = msg->velocity.x;
    velocity(1) = msg->velocity.y;
  }

  if (msg->use_velocity_vertical) {
    velocity(2) = msg->velocity.z;
  }

  if (msg->use_heading_rate) {
    heading_rate = msg->heading_rate;
  }

  if (msg->use_acceleration) {
    acceleration(0) = msg->acceleration.x;
    acceleration(1) = msg->acceleration.y;
    acceleration(2) = msg->acceleration.z;
  }

  uav_system_.setFeedforward(reference::VelocityHdg(velocity, 0));
  uav_system_.setFeedforward(reference::VelocityHdgRate(velocity, heading_rate));
  uav_system_.setFeedforward(reference::AccelerationHdg(acceleration, 0));
  uav_system_.setFeedforward(reference::AccelerationHdgRate(acceleration, heading_rate));
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

    model_params_ = uav_system_.getParams();

    const double original_mass = model_params_.mass;

    model_params_.mass = request->value;

    model_params_.allocation_matrix.row(2) = model_params_.mass * (model_params_.allocation_matrix.row(2) / original_mass);

    calculateInertia(model_params_);

    uav_system_.setParams(model_params_);
  }

  response->success = true;
  response->message = "mass set";

  return true;
}

//}

/* callbackSetGroundZ() //{ */

bool UavSystemRos::callbackSetGroundZ(const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request>  request,
                                      const std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  {
    std::scoped_lock lock(mutex_uav_system_);

    model_params_ = uav_system_.getParams();

    model_params_.ground_z = request->value;

    uav_system_.setParams(model_params_);
  }

  response->success = true;
  response->message = "ground z set";

  return true;
}

//}

} // namespace mrs_multirotor_simulator
