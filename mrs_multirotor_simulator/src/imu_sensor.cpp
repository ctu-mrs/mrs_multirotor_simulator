#include <mrs_multirotor_simulator/imu_sensor.h>

#include <mrs_lib/attitude_converter.h>

namespace mrs_multirotor_simulator
{

/* ImuSensor() //{ */

ImuSensor::ImuSensor(const rclcpp::Node::SharedPtr &node, const rclcpp::CallbackGroup::SharedPtr &cbgrp_ss, const ImuSensorParams_t &params,
                     const std::shared_ptr<ImuShapingFilters> &shaping_filters) {

  node_ = node;

  _params_ = params;

  shaping_filters_ = shaping_filters;

  // | -------------------------- noise ------------------------- |

  noise_params_ = _params_.noise;

  updateNoiseShaperParams();

  // * the shaping filter is set only when the noise is enabled, otherwise it is not needed
  if (noise_params_.enabled && !setShapingFilter()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] could not set the noise shaping filter '%s'", _params_.frame_id.c_str(), noise_params_.shaping_filter_name.c_str());
    throw std::runtime_error("[" + _params_.frame_id + "] could not set the noise shaping filter '" + noise_params_.shaping_filter_name + "'");
  }

  // | ----------------------- publishers ----------------------- |

  ph_imu_ = std::make_shared<mrs_lib::PublisherHandler<sensor_msgs::msg::Imu>>(node_, _params_.topic);

  if (_params_.noiseless_topic != "") {
    ph_imu_noiseless_ = std::make_shared<mrs_lib::PublisherHandler<sensor_msgs::msg::Imu>>(node_, _params_.noiseless_topic);
  }

  // | --------------------- service servers -------------------- |

  ss_set_noise_ = mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>(
      node_, _params_.service_prefix + _params_.noise_toggle_service,
      std::bind(&ImuSensor::callbackSetNoise, this, std::placeholders::_1, std::placeholders::_2), cbgrp_ss);

  ss_set_gyro_bias_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::Float64Srv>(
      node_, _params_.service_prefix + "set_gyro_bias", std::bind(&ImuSensor::callbackSetGyroBias, this, std::placeholders::_1, std::placeholders::_2),
      cbgrp_ss);

  ss_set_gyro_stddev_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::Float64Srv>(
      node_, _params_.service_prefix + "set_gyro_stddev", std::bind(&ImuSensor::callbackSetGyroStddev, this, std::placeholders::_1, std::placeholders::_2),
      cbgrp_ss);

  ss_set_accel_bias_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::Float64Srv>(
      node_, _params_.service_prefix + "set_accel_bias", std::bind(&ImuSensor::callbackSetAccelBias, this, std::placeholders::_1, std::placeholders::_2),
      cbgrp_ss);

  ss_set_accel_stddev_ = mrs_lib::ServiceServerHandler<mrs_msgs::srv::Float64Srv>(
      node_, _params_.service_prefix + "set_accel_stddev", std::bind(&ImuSensor::callbackSetAccelStddev, this, std::placeholders::_1, std::placeholders::_2),
      cbgrp_ss);

  is_initialized_ = true;
}

//}

/* publish() //{ */

void ImuSensor::publish(const rclcpp::Time &stamp, const MultirotorModel::State &state, const Eigen::Vector3d &imu_acc, const Eigen::Vector3d &omega_dot) {

  if (!is_initialized_) {
    return;
  }

  const Eigen::Vector3d &r     = _params_.position;
  const Eigen::Matrix3d &R_b_s = _params_.orientation;
  const Eigen::Vector3d &omega = state.omega;

  // specific force at the IMU location (rigid body kinematics), in the FCU frame
  const Eigen::Vector3d acc_body = imu_acc + omega_dot.cross(r) + omega.cross(omega.cross(r));

  // rotate to the IMU frame
  Eigen::Vector3d gyro  = R_b_s.transpose() * omega;
  Eigen::Vector3d accel = R_b_s.transpose() * acc_body;

  sensor_msgs::msg::Imu imu;

  imu.header.stamp    = stamp;
  imu.header.frame_id = _params_.frame_id;

  imu.orientation = mrs_lib::AttitudeConverter(Eigen::Matrix3d(state.R * R_b_s));

  imu.angular_velocity.x = gyro(0);
  imu.angular_velocity.y = gyro(1);
  imu.angular_velocity.z = gyro(2);

  imu.linear_acceleration.x = accel(0);
  imu.linear_acceleration.y = accel(1);
  imu.linear_acceleration.z = accel(2);

  if (ph_imu_noiseless_) {
    ph_imu_noiseless_->publish(imu);
  }

  {
    std::scoped_lock lock(mutex_noise_);

    if (noise_params_.enabled) {
      noise_shaper_.addNoise(gyro, accel);
    }
  }

  imu.angular_velocity.x = gyro(0);
  imu.angular_velocity.y = gyro(1);
  imu.angular_velocity.z = gyro(2);

  imu.linear_acceleration.x = accel(0);
  imu.linear_acceleration.y = accel(1);
  imu.linear_acceleration.z = accel(2);

  ph_imu_->publish(imu);
}

//}

/* getStaticTF() //{ */

geometry_msgs::msg::TransformStamped ImuSensor::getStaticTF(const std::string &parent_frame, const rclcpp::Time &stamp) const {

  geometry_msgs::msg::TransformStamped tf;

  tf.header.stamp    = stamp;
  tf.header.frame_id = parent_frame;
  tf.child_frame_id  = _params_.frame_id;

  tf.transform.translation.x = _params_.position(0);
  tf.transform.translation.y = _params_.position(1);
  tf.transform.translation.z = _params_.position(2);

  tf.transform.rotation = mrs_lib::AttitudeConverter(_params_.orientation);

  return tf;
}

//}

/* getFrameId() //{ */

std::string ImuSensor::getFrameId(void) const {

  return _params_.frame_id;
}

//}

// | ------------------------ routines ------------------------ |

/* updateNoiseShaperParams() //{ */

// * expects mutex_noise_ to be locked by the caller (or not needed, as in the constructor)
void ImuSensor::updateNoiseShaperParams(void) {

  noise_shaper_.setAccelNoiseParams(noise_params_.accel_bias, noise_params_.accel_stddev);
  noise_shaper_.setGyroNoiseParams(noise_params_.gyro_bias, noise_params_.gyro_stddev, noise_params_.gyro_separate_z_bias, noise_params_.gyro_z_bias);
}

//}

/* setShapingFilter() //{ */

// * expects mutex_noise_ to be locked by the caller (or not needed, as in the constructor)
bool ImuSensor::setShapingFilter(void) {

  if (shaping_filter_set_) {
    return true;
  }

  // white noise, the default pass-through filters of the noise shaper
  if (noise_params_.shaping_filter_name == "none") {
    shaping_filter_set_ = true;
    return true;
  }

  const auto filter = shaping_filters_->getFilter(noise_params_.shaping_filter_name);

  if (!filter) {
    return false;
  }

  noise_shaper_.setGyroShapingFilter(filter->gyro);
  noise_shaper_.setAccelShapingFilter(filter->accel);

  shaping_filter_set_ = true;

  return true;
}

//}

// | -------------------- service callbacks ------------------- |

/* callbackSetNoise() //{ */

bool ImuSensor::callbackSetNoise(const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
                                 const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  {
    std::scoped_lock lock(mutex_noise_);

    if (request->data && !setShapingFilter()) {

      RCLCPP_ERROR(node_->get_logger(), "[%s] could not enable the noise, failed to set the shaping filter '%s'", _params_.frame_id.c_str(),
                   noise_params_.shaping_filter_name.c_str());

      response->success = false;
      response->message = "could not set the noise shaping filter '" + noise_params_.shaping_filter_name + "'";

      return true;
    }

    noise_params_.enabled = request->data;
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] noise %s", _params_.frame_id.c_str(), request->data ? "enabled" : "disabled");

  response->success = true;
  response->message = request->data ? "noise enabled" : "noise disabled";

  return true;
}

//}

/* callbackSetGyroBias() //{ */

bool ImuSensor::callbackSetGyroBias(const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request>  request,
                                    const std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  if (!std::isfinite(request->value)) {
    response->success = false;
    response->message = "the value is not finite";
    return true;
  }

  {
    std::scoped_lock lock(mutex_noise_);

    noise_params_.gyro_bias = request->value;

    updateNoiseShaperParams();
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] gyro bias set to %.6f", _params_.frame_id.c_str(), request->value);

  response->success = true;
  response->message = "gyro bias set";

  return true;
}

//}

/* callbackSetGyroStddev() //{ */

bool ImuSensor::callbackSetGyroStddev(const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request>  request,
                                      const std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  if (!std::isfinite(request->value) || request->value < 0.0) {
    response->success = false;
    response->message = "the stddev has to be finite and >= 0";
    return true;
  }

  {
    std::scoped_lock lock(mutex_noise_);

    noise_params_.gyro_stddev = request->value;

    updateNoiseShaperParams();
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] gyro stddev set to %.6f", _params_.frame_id.c_str(), request->value);

  response->success = true;
  response->message = "gyro stddev set";

  return true;
}

//}

/* callbackSetAccelBias() //{ */

bool ImuSensor::callbackSetAccelBias(const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request>  request,
                                     const std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  if (!std::isfinite(request->value)) {
    response->success = false;
    response->message = "the value is not finite";
    return true;
  }

  {
    std::scoped_lock lock(mutex_noise_);

    noise_params_.accel_bias = request->value;

    updateNoiseShaperParams();
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] accel bias set to %.6f", _params_.frame_id.c_str(), request->value);

  response->success = true;
  response->message = "accel bias set";

  return true;
}

//}

/* callbackSetAccelStddev() //{ */

bool ImuSensor::callbackSetAccelStddev(const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request>  request,
                                       const std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  if (!std::isfinite(request->value) || request->value < 0.0) {
    response->success = false;
    response->message = "the stddev has to be finite and >= 0";
    return true;
  }

  {
    std::scoped_lock lock(mutex_noise_);

    noise_params_.accel_stddev = request->value;

    updateNoiseShaperParams();
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] accel stddev set to %.6f", _params_.frame_id.c_str(), request->value);

  response->success = true;
  response->message = "accel stddev set";

  return true;
}

//}

} // namespace mrs_multirotor_simulator
