/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_hw_api/api.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>

#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>

#include <mrs_lib/gps_conversions.h>

//}

namespace mrs_uav_simulator_hw_api_plugin
{

/* class Api //{ */

class Api : public mrs_uav_hw_api::MrsUavHwApi {

public:
  ~Api(){};

  void initialize(const ros::NodeHandle& parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers, const std::string& topic_prefix,
                  const std::string& uav_name);

  // | ------------------------- params ------------------------- |

  mrs_msgs::HwApiMode _mode_;

  double _utm_x_;
  double _utm_y_;
  double _amsl_;

  // | --------------------- status methods --------------------- |

  mrs_msgs::HwApiDiagnostics getDiagnostics();
  mrs_msgs::HwApiMode        getMode();

  // | --------------------- topic callbacks -------------------- |

  bool callbackActuatorCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiActuatorCmd>& wrp);
  bool callbackControlGroupCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd>& wrp);
  bool callbackAttitudeRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp);
  bool callbackAttitudeCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>& wrp);
  bool callbackAccelerationHdgRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgRateCmd>& wrp);
  bool callbackAccelerationHdgCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgCmd>& wrp);
  bool callbackVelocityHdgRateCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgRateCmd>& wrp);
  bool callbackVelocityHdgCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgCmd>& wrp);
  bool callbackPositionCmd(mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd>& wrp);

  // | -------------------- service callbacks ------------------- |

  std::tuple<bool, std::string> callbackArming(const bool& request);
  std::tuple<bool, std::string> callbackOffboard(void);

private:
  bool is_initialized_ = false;

  std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

  // | ----------------------- parameters ----------------------- |

  std::string _simulator_prefix_;

  std::string _topic_simulator_odom_;
  std::string _topic_simulator_imu_;
  std::string _topic_simulator_rangefinder_;

  std::string _topic_simulator_actuators_cmd_;
  std::string _topic_simulator_control_group_cmd_;
  std::string _topic_simulator_attitude_rate_cmd_;
  std::string _topic_simulator_attitude_cmd_;
  std::string _topic_simulator_acceleration_hdg_rate_cmd_;
  std::string _topic_simulator_acceleration_hdg_cmd_;
  std::string _topic_simulator_velocity_hdg_rate_cmd_;
  std::string _topic_simulator_velocity_hdg_cmd_;
  std::string _topic_simulator_position_cmd_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_odom_;
  mrs_lib::SubscribeHandler<sensor_msgs::Imu>   sh_imu_;
  mrs_lib::SubscribeHandler<sensor_msgs::Range> sh_range_;

  void callbackOdom(mrs_lib::SubscribeHandler<nav_msgs::Odometry>& wrp);
  void callbackImu(mrs_lib::SubscribeHandler<sensor_msgs::Imu>& wrp);
  void callbackRangefinder(mrs_lib::SubscribeHandler<sensor_msgs::Range>& wrp);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mrs_msgs::HwApiActuatorCmd>            ph_actuators_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiControlGroupCmd>        ph_control_group_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiAttitudeRateCmd>        ph_attitude_rate_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiAttitudeCmd>            ph_attitude_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiAccelerationHdgRateCmd> ph_acceleration_hdg_rate_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiAccelerationHdgCmd>     ph_acceleration_hdg_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgRateCmd>     ph_velocity_hdg_rate_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgCmd>         ph_velocity_hdg_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiPositionCmd>            ph_position_cmd_;

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_main_;

  void timerMain(const ros::TimerEvent& event);

  // | ------------------------ variables ----------------------- |

  std::atomic<bool> offboard_ = false;
  std::string       mode_;
  std::atomic<bool> armed_     = true;
  std::atomic<bool> connected_ = false;
  std::mutex        mutex_diagnostics_;

  // | ------------------------- methods ------------------------ |

  void publishBatteryState(void);

  void publishRC(void);

  void timeoutInputs(void);
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void Api::initialize(const ros::NodeHandle& parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers,
                     [[maybe_unused]] const std::string& topic_prefix, [[maybe_unused]] const std::string& uav_name) {

  ros::NodeHandle nh_(parent_nh);

  common_handlers_ = common_handlers;

  _mode_.api_name = "MrsSimulator";

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "MrsUavHwApi");

  param_loader.loadParam("gnss/utm_x", _utm_x_);
  param_loader.loadParam("gnss/utm_y", _utm_y_);
  param_loader.loadParam("gnss/amsl", _amsl_);

  param_loader.loadParam("input_mode/actuators", (bool&)_mode_.accepts_actuator_cmd);
  param_loader.loadParam("input_mode/control_group", (bool&)_mode_.accepts_control_group_cmd);
  param_loader.loadParam("input_mode/attitude_rate", (bool&)_mode_.accepts_attitude_rate_cmd);
  param_loader.loadParam("input_mode/attitude", (bool&)_mode_.accepts_attitude_cmd);
  param_loader.loadParam("input_mode/acceleration_hdg_rate", (bool&)_mode_.accepts_acceleration_hdg_rate_cmd);
  param_loader.loadParam("input_mode/acceleration_hdg", (bool&)_mode_.accepts_acceleration_hdg_cmd);
  param_loader.loadParam("input_mode/velocity_hdg_rate", (bool&)_mode_.accepts_velocity_hdg_rate_cmd);
  param_loader.loadParam("input_mode/velocity_hdg", (bool&)_mode_.accepts_velocity_hdg_cmd);
  param_loader.loadParam("input_mode/position", (bool&)_mode_.accepts_position_cmd);

  param_loader.loadParam("outputs/distance_sensor", (bool&)_mode_.produces_distance_sensor);
  param_loader.loadParam("outputs/gnss", (bool&)_mode_.produces_gnss);
  param_loader.loadParam("outputs/imu", (bool&)_mode_.produces_imu);
  param_loader.loadParam("outputs/altitude", (bool&)_mode_.produces_altitude);
  param_loader.loadParam("outputs/magnetometer_heading", (bool&)_mode_.produces_magnetometer_heading);
  param_loader.loadParam("outputs/odometry_local", (bool&)_mode_.produces_odometry_local);
  param_loader.loadParam("outputs/rc_channels", (bool&)_mode_.produces_rc_channels);
  param_loader.loadParam("outputs/orientation", (bool&)_mode_.produces_orientation);
  param_loader.loadParam("outputs/battery_state", (bool&)_mode_.produces_battery_state);

  param_loader.loadParam("topics/prefix", _simulator_prefix_);
  param_loader.loadParam("topics/simulator/odom", _topic_simulator_odom_);
  param_loader.loadParam("topics/simulator/imu", _topic_simulator_imu_);
  param_loader.loadParam("topics/simulator/rangefinder", _topic_simulator_rangefinder_);
  param_loader.loadParam("topics/simulator/actuators_cmd", _topic_simulator_actuators_cmd_);
  param_loader.loadParam("topics/simulator/control_group_cmd", _topic_simulator_control_group_cmd_);
  param_loader.loadParam("topics/simulator/attitude_rate_cmd", _topic_simulator_attitude_rate_cmd_);
  param_loader.loadParam("topics/simulator/attitude_cmd", _topic_simulator_attitude_cmd_);
  param_loader.loadParam("topics/simulator/acceleration_hdg_rate_cmd", _topic_simulator_acceleration_hdg_rate_cmd_);
  param_loader.loadParam("topics/simulator/acceleration_hdg_cmd", _topic_simulator_acceleration_hdg_cmd_);
  param_loader.loadParam("topics/simulator/velocity_hdg_rate_cmd", _topic_simulator_velocity_hdg_rate_cmd_);
  param_loader.loadParam("topics/simulator/velocity_hdg_cmd", _topic_simulator_velocity_hdg_cmd_);
  param_loader.loadParam("topics/simulator/position_cmd", _topic_simulator_position_cmd_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MrsUavHwDummyApi]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "MrsSimulatorHwApi";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_odom_ =
      mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "/" + _simulator_prefix_ + "/" + uav_name + "/" + _topic_simulator_odom_, &Api::callbackOdom, this);

  sh_imu_ =
      mrs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts, "/" + _simulator_prefix_ + "/" + uav_name + "/" + _topic_simulator_imu_, &Api::callbackImu, this);

  sh_range_ = mrs_lib::SubscribeHandler<sensor_msgs::Range>(shopts, "/" + _simulator_prefix_ + "/" + uav_name + "/" + _topic_simulator_rangefinder_,
                                                            &Api::callbackRangefinder, this);

  // | ----------------------- publishers ----------------------- |

  if (_mode_.accepts_actuator_cmd) {
    ph_actuators_cmd_ =
        mrs_lib::PublisherHandler<mrs_msgs::HwApiActuatorCmd>(nh_, "/" + _simulator_prefix_ + "/" + uav_name + "/" + _topic_simulator_actuators_cmd_, 1);
  }

  if (_mode_.accepts_control_group_cmd) {
    ph_control_group_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiControlGroupCmd>(
        nh_, "/" + _simulator_prefix_ + "/" + uav_name + "/" + _topic_simulator_control_group_cmd_, 1);
  }

  if (_mode_.accepts_attitude_rate_cmd) {
    ph_attitude_rate_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiAttitudeRateCmd>(
        nh_, "/" + _simulator_prefix_ + "/" + uav_name + "/" + _topic_simulator_attitude_rate_cmd_, 1);
  }

  if (_mode_.accepts_attitude_cmd) {
    ph_attitude_cmd_ =
        mrs_lib::PublisherHandler<mrs_msgs::HwApiAttitudeCmd>(nh_, "/" + _simulator_prefix_ + "/" + uav_name + "/" + _topic_simulator_attitude_cmd_, 1);
  }

  if (_mode_.accepts_acceleration_hdg_rate_cmd) {
    ph_acceleration_hdg_rate_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiAccelerationHdgRateCmd>(
        nh_, "/" + _simulator_prefix_ + "/" + uav_name + "/" + _topic_simulator_acceleration_hdg_rate_cmd_, 1);
  }

  if (_mode_.accepts_acceleration_hdg_cmd) {
    ph_acceleration_hdg_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiAccelerationHdgCmd>(
        nh_, "/" + _simulator_prefix_ + "/" + uav_name + "/" + _topic_simulator_acceleration_hdg_cmd_, 1);
  }

  if (_mode_.accepts_velocity_hdg_rate_cmd) {
    ph_velocity_hdg_rate_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgRateCmd>(
        nh_, "/" + _simulator_prefix_ + "/" + uav_name + "/" + _topic_simulator_velocity_hdg_rate_cmd_, 1);
  }

  if (_mode_.accepts_velocity_hdg_cmd) {
    ph_velocity_hdg_cmd_ =
        mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgCmd>(nh_, "/" + _simulator_prefix_ + "/" + uav_name + "/" + _topic_simulator_velocity_hdg_cmd_, 1);
  }

  if (_mode_.accepts_position_cmd) {
    ph_position_cmd_ =
        mrs_lib::PublisherHandler<mrs_msgs::HwApiPositionCmd>(nh_, "/" + _simulator_prefix_ + "/" + uav_name + "/" + _topic_simulator_position_cmd_, 1);
  }

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(10.0), &Api::timerMain, this);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[MrsUavHwDummyApi]: initialized");

  is_initialized_ = true;
}

//}

/* getDiagnostics() //{ */

mrs_msgs::HwApiDiagnostics Api::getDiagnostics() {

  mrs_msgs::HwApiDiagnostics diag;

  diag.stamp = ros::Time::now();

  {
    std::scoped_lock lock(mutex_diagnostics_);

    diag.armed     = armed_;
    diag.offboard  = offboard_;
    diag.connected = connected_;
    diag.mode      = mode_;
  }

  return diag;
}

//}

/* getMode() //{ */

mrs_msgs::HwApiMode Api::getMode() {

  _mode_.stamp = ros::Time::now();

  return _mode_;
}

//}

/* callbackArming() //{ */

std::tuple<bool, std::string> Api::callbackArming([[maybe_unused]] const bool& request) {

  std::stringstream ss;

  if (!request && !offboard_) {
    ss << "can not disarm, not in OFFBOARD mode";
    ROS_WARN_STREAM_THROTTLE(1.0, "[MrsSimulatorHwApi]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  if (request) {

    armed_ = true;

    ss << "armed";
    ROS_INFO_STREAM_THROTTLE(1.0, "[MrsSimulatorHwApi]: " << ss.str());
    return std::tuple(true, ss.str());

  } else {

    armed_ = false;

    ss << "disarmed";
    ROS_INFO_STREAM_THROTTLE(1.0, "[MrsSimulatorHwApi]: " << ss.str());
    return std::tuple(true, ss.str());
  }
}

//}

/* callbackOffboard() //{ */

std::tuple<bool, std::string> Api::callbackOffboard(void) {

  std::stringstream ss;

  if (!armed_) {
    ss << "Cannot switch to offboard, not armed.";
    ROS_INFO_THROTTLE(1.0, "[MrsSimulatorHwApi]: %s", ss.str().c_str());
    return {false, ss.str()};
  }

  offboard_ = true;

  ss << "Offboard set";
  ROS_INFO_THROTTLE(1.0, "[MrsSimulatorHwApi]: %s", ss.str().c_str());
  return {true, ss.str()};
}

//}

// | --------------------- input callbacks -------------------- |

/* callbackActuatorCmd() //{ */

bool Api::callbackActuatorCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiActuatorCmd>& wrp) {

  if (!_mode_.accepts_actuator_cmd) {
    return false;
  }

  ROS_INFO_ONCE("[Api]: getting actuators cmd");

  ph_actuators_cmd_.publish(wrp.getMsg());

  return true;
}

//}

/* callbackControlGroupCmd() //{ */

bool Api::callbackControlGroupCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiControlGroupCmd>& wrp) {

  if (!_mode_.accepts_control_group_cmd) {
    return false;
  }

  ROS_INFO_ONCE("[Api]: getting control group cmd");

  ph_control_group_cmd_.publish(wrp.getMsg());

  return true;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool Api::callbackAttitudeRateCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeRateCmd>& wrp) {

  if (!_mode_.accepts_attitude_rate_cmd) {
    return false;
  }

  ROS_INFO_ONCE("[Api]: getting attitude rate cmd");

  ph_attitude_rate_cmd_.publish(wrp.getMsg());

  return true;
}

//}

/* callbackAttitudeCmd() //{ */

bool Api::callbackAttitudeCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAttitudeCmd>& wrp) {

  if (!_mode_.accepts_attitude_cmd) {
    return false;
  }

  ROS_INFO_ONCE("[Api]: getting attitude cmd");

  ph_attitude_cmd_.publish(wrp.getMsg());

  return true;
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

bool Api::callbackAccelerationHdgRateCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgRateCmd>& wrp) {

  if (!_mode_.accepts_acceleration_hdg_rate_cmd) {
    return false;
  }

  ROS_INFO_ONCE("[Api]: getting acceleration+hdg rate cmd");

  ph_acceleration_hdg_rate_cmd_.publish(wrp.getMsg());

  return true;
}

//}

/* callbackAccelerationHdgCmd() //{ */

bool Api::callbackAccelerationHdgCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiAccelerationHdgCmd>& wrp) {

  if (!_mode_.accepts_acceleration_hdg_cmd) {

    return false;
  }

  ROS_INFO_ONCE("[Api]: getting acceleration+hdg cmd");

  ph_acceleration_hdg_cmd_.publish(wrp.getMsg());

  return true;
}

//}

/* callbackVelocityHdgRateCmd() //{ */

bool Api::callbackVelocityHdgRateCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgRateCmd>& wrp) {

  if (!_mode_.accepts_velocity_hdg_rate_cmd) {
    return false;
  }

  ROS_INFO_ONCE("[Api]: getting velocity+hdg rate cmd");

  ph_velocity_hdg_rate_cmd_.publish(wrp.getMsg());

  return true;
}

//}

/* callbackVelocityHdgCmd() //{ */

bool Api::callbackVelocityHdgCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiVelocityHdgCmd>& wrp) {

  if (!_mode_.accepts_velocity_hdg_cmd) {
    return false;
  }

  ROS_INFO_ONCE("[Api]: getting velocity+hdg cmd");

  ph_velocity_hdg_cmd_.publish(wrp.getMsg());

  return true;
}

//}

/* callbackPositionCmd() //{ */

bool Api::callbackPositionCmd([[maybe_unused]] mrs_lib::SubscribeHandler<mrs_msgs::HwApiPositionCmd>& wrp) {

  if (!_mode_.accepts_position_cmd) {
    return false;
  }

  ROS_INFO_ONCE("[Api]: getting position cmd");

  ph_position_cmd_.publish(wrp.getMsg());

  return true;
}

//}

// | ------------------------ callbacks ----------------------- |

/* //{ callbackOdom() */

void Api::callbackOdom(mrs_lib::SubscribeHandler<nav_msgs::Odometry>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Api]: getting simulater odom");

  auto odom = wrp.getMsg();

  {
    std::scoped_lock lock(mutex_diagnostics_);

    connected_ = true;
  }

  // | ----------------- publish the diagnostics ---------------- |

  mrs_msgs::HwApiDiagnostics diag;

  {
    std::scoped_lock lock(mutex_diagnostics_);

    diag.stamp     = ros::Time::now();
    diag.armed     = armed_;
    diag.offboard  = offboard_;
    diag.connected = connected_;
    diag.mode      = mode_;
  }

  common_handlers_->publishers.publishDiagnostics(diag);

  // | ----------------- publish local odometry ----------------- |

  if (_mode_.produces_odometry_local) {
    common_handlers_->publishers.publishOdometryLocal(*odom);
  }

  // | ---------------------- publish gnss ---------------------- |

  if (_mode_.produces_gnss) {

    double lat;
    double lon;

    mrs_lib::UTMtoLL(odom->pose.pose.position.y + _utm_y_, odom->pose.pose.position.x + _utm_x_, "32T", lat, lon);

    sensor_msgs::NavSatFix gnss;

    gnss.header.stamp = odom->header.stamp;

    gnss.latitude  = lat;
    gnss.longitude = lon;
    gnss.altitude  = odom->pose.pose.position.z + _amsl_;

    common_handlers_->publishers.publishGNSS(gnss);
  }

  if (_mode_.produces_altitude) {

    mrs_msgs::HwApiAltitude altitude;

    altitude.stamp = odom->header.stamp;

    altitude.amsl = odom->pose.pose.position.z + _amsl_;

    common_handlers_->publishers.publishAltitude(altitude);
  }

  // | --------------------- publish heading -------------------- |

  if (_mode_.produces_magnetometer_heading) {
    double heading = mrs_lib::AttitudeConverter(odom->pose.pose.orientation).getHeading();

    mrs_msgs::Float64Stamped hdg;

    hdg.header.stamp = ros::Time::now();
    hdg.value        = heading;

    common_handlers_->publishers.publishMagnetometerHeading(hdg);
  }
}

//}

/* callbackImu() //{ */

void Api::callbackImu(mrs_lib::SubscribeHandler<sensor_msgs::Imu>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Api]: getting IMU");

  if (_mode_.produces_imu) {
    sensor_msgs::ImuConstPtr imu = wrp.getMsg();

    common_handlers_->publishers.publishIMU(*imu);
  }
}

//}

/* callbackRangefinder() //{ */

void Api::callbackRangefinder(mrs_lib::SubscribeHandler<sensor_msgs::Range>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Api]: getting rangefinder");

  if (_mode_.produces_distance_sensor) {
    sensor_msgs::RangeConstPtr range = wrp.getMsg();

    common_handlers_->publishers.publishDistanceSensor(*range);
  }
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void Api::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Api]: main timer spinning");

  publishBatteryState();

  publishRC();
}

//}

// | ------------------------- methods ------------------------ |

/* publishBatteryState() //{ */

void Api::publishBatteryState(void) {

  if (_mode_.produces_battery_state) {

    sensor_msgs::BatteryState msg;

    msg.capacity = 100;
    msg.current  = 10.0;
    msg.voltage  = 15.8;
    msg.charge   = 0.8;

    common_handlers_->publishers.publishBatteryState(msg);
  }
}

//}

/* publishRC() //{ */

void Api::publishRC(void) {

  if (_mode_.produces_rc_channels) {

    mrs_msgs::HwApiRcChannels rc;

    rc.stamp = ros::Time::now();

    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);
    rc.channels.push_back(0);

    common_handlers_->publishers.publishRcChannels(rc);
  }
}

//}

/* MrsUavHwApi() //{ */

void Api::timeoutInputs(void) {
}

//}

}  // namespace mrs_uav_simulator_hw_api_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_simulator_hw_api_plugin::Api, mrs_uav_hw_api::MrsUavHwApi)
