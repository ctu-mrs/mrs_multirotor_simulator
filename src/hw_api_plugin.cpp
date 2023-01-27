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

#include <mrs_multirotor_simulator/Diagnostics.h>

#include <mrs_lib/gps_conversions.h>

//}

/* defines //{ */

#define PWM_MIDDLE 1500
#define PWM_MIN 1000
#define PWM_MAX 2000
#define PWM_DEADBAND 200
#define PWM_RANGE PWM_MAX - PWM_MIN

//}

namespace mrs_uav_simulator_hw_api_plugin
{

/* class Api //{ */

class Api : public mrs_uav_hw_api::MrsUavHwApi {

public:
  ~Api(){};

  void initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers, const std::string &topic_prefix,
                  const std::string &uav_name);

  // | --------------------- status methods --------------------- |

  mrs_msgs::HwApiDiagnostics getDiagnostics();
  mrs_msgs::HwApiMode        getMode();

  // | --------------------- topic callbacks -------------------- |

  bool callbackControlGroupCmd(const mrs_msgs::HwApiControlGroupCmd &msg);
  bool callbackAttitudeRateCmd(const mrs_msgs::HwApiAttitudeRateCmd &msg);
  bool callbackAttitudeCmd(const mrs_msgs::HwApiAttitudeCmd &msg);
  bool callbackAccelerationCmd(const mrs_msgs::HwApiAccelerationCmd &msg);
  bool callbackVelocityCmd(const mrs_msgs::HwApiVelocityCmd &msg);
  bool callbackPositionCmd(const mrs_msgs::HwApiPositionCmd &msg);

  // | -------------------- service callbacks ------------------- |

  std::tuple<bool, std::string> callbackArming(const bool &request);
  std::tuple<bool, std::string> callbackOffboard(void);

private:
  bool is_initialized_ = false;

  std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

  // | ----------------------- parameters ----------------------- |

  std::string _topic_simulator_odom_;
  std::string _topic_simulator_imu_;
  std::string _topic_simulator_diag_;

  std::string _topic_simulator_attitude_rate_cmd_;
  std::string _topic_simulator_attitude_cmd_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                    sh_odom_;
  mrs_lib::SubscribeHandler<sensor_msgs::Imu>                      sh_imu_;
  mrs_lib::SubscribeHandler<mrs_multirotor_simulator::Diagnostics> sh_simulator_diag_;

  void callbackOdom(mrs_lib::SubscribeHandler<nav_msgs::Odometry> &wrp);
  void callbackImu(mrs_lib::SubscribeHandler<sensor_msgs::Imu> &wrp);
  void callbackSimulatorDiag(mrs_lib::SubscribeHandler<mrs_multirotor_simulator::Diagnostics> &wrp);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mrs_msgs::HwApiAttitudeRateCmd> ph_attitude_rate_cmd_;
  mrs_lib::PublisherHandler<mrs_msgs::HwApiAttitudeCmd>     ph_attitude_cmd_;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<std_srvs::SetBool> sch_arm_;

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_main_;

  void timerMain(const ros::TimerEvent &event);

  // | ------------------------ variables ----------------------- |

  std::atomic<bool> offboard_ = true;
  std::string       mode_;
  std::atomic<bool> armed_     = true;
  std::atomic<bool> connected_ = false;
  std::mutex        mutex_diagnostics_;

  // | ------------------------- methods ------------------------ |

  void publishBatteryState(void);

  void publishRC(void);
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void Api::initialize(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers,
                     [[maybe_unused]] const std::string &topic_prefix, [[maybe_unused]] const std::string &uav_name) {

  ros::NodeHandle nh_(parent_nh);

  common_handlers_ = common_handlers;

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "MrsUavHwApi");

  param_loader.loadParam("topics/simulator/odom", _topic_simulator_odom_);
  param_loader.loadParam("topics/simulator/imu", _topic_simulator_imu_);
  param_loader.loadParam("topics/simulator/diagnostics", _topic_simulator_diag_);
  param_loader.loadParam("topics/simulator/attitude_rate_cmd", _topic_simulator_attitude_rate_cmd_);
  param_loader.loadParam("topics/simulator/attitude_cmd", _topic_simulator_attitude_cmd_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MrsUavHwDummyApi]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "MrsHwPixhawkApi";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_odom_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, topic_prefix + "/" + _topic_simulator_odom_, &Api::callbackOdom, this);

  sh_imu_ = mrs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts, topic_prefix + "/" + _topic_simulator_imu_, &Api::callbackImu, this);

  sh_simulator_diag_ =
      mrs_lib::SubscribeHandler<mrs_multirotor_simulator::Diagnostics>(shopts, topic_prefix + "/" + _topic_simulator_diag_, &Api::callbackSimulatorDiag, this);

  // | --------------------- service clients -------------------- |

  sch_arm_ = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "arm_out");

  // | ----------------------- publishers ----------------------- |

  ph_attitude_rate_cmd_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiAttitudeRateCmd>(nh_, topic_prefix + "/" + _topic_simulator_attitude_rate_cmd_, 1);
  ph_attitude_cmd_      = mrs_lib::PublisherHandler<mrs_msgs::HwApiAttitudeCmd>(nh_, topic_prefix + "/" + _topic_simulator_attitude_cmd_, 1);

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

  mrs_msgs::HwApiMode mode;

  mode.api_name = "PixhawkApi";
  mode.stamp    = ros::Time::now();

  mode.accepts_control_group_cmd = false;
  mode.accepts_actuator_cmd      = false;
  mode.accepts_attitude_rate_cmd = true;
  mode.accepts_attitude_cmd      = false;
  mode.accepts_acceleration_cmd  = false;
  mode.accepts_velocity_cmd      = false;
  mode.accepts_position_cmd      = false;

  mode.produces_distance_sensor      = true;
  mode.produces_gnss                 = true;
  mode.produces_imu                  = true;
  mode.produces_altitude             = false;
  mode.produces_magnetometer_heading = true;
  mode.produces_odometry_local       = true;
  mode.produces_rc_channels          = true;

  return mode;
}

//}

/* callbackControlGroupCmd() //{ */

bool Api::callbackControlGroupCmd([[maybe_unused]] const mrs_msgs::HwApiControlGroupCmd &msg) {

  ROS_INFO_ONCE("[Api]: getting control group cmd");

  return false;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool Api::callbackAttitudeRateCmd([[maybe_unused]] const mrs_msgs::HwApiAttitudeRateCmd &msg) {

  ROS_INFO_ONCE("[Api]: getting attitude rate cmd");

  ph_attitude_rate_cmd_.publish(msg);

  return true;
}

//}

/* callbackAttitudeCmd() //{ */

bool Api::callbackAttitudeCmd([[maybe_unused]] const mrs_msgs::HwApiAttitudeCmd &msg) {

  ROS_INFO_ONCE("[Api]: getting attitude cmd");

  ph_attitude_cmd_.publish(msg);

  return true;
}

//}

/* callbackAccelerationCmd() //{ */

bool Api::callbackAccelerationCmd([[maybe_unused]] const mrs_msgs::HwApiAccelerationCmd &msg) {

  ROS_INFO_ONCE("[Api]: getting acceleration cmd");

  return false;
}

//}

/* callbackVelocityCmd() //{ */

bool Api::callbackVelocityCmd([[maybe_unused]] const mrs_msgs::HwApiVelocityCmd &msg) {

  ROS_INFO_ONCE("[Api]: getting velocity cmd");

  return false;
}

//}

/* callbackPositionCmd() //{ */

bool Api::callbackPositionCmd([[maybe_unused]] const mrs_msgs::HwApiPositionCmd &msg) {

  ROS_INFO_ONCE("[Api]: getting position cmd");

  return false;
}

//}

/* callbackArming() //{ */

std::tuple<bool, std::string> Api::callbackArming([[maybe_unused]] const bool &request) {

  std::stringstream ss;

  if (request) {

    ss << "Arming is not allowed using the companion computer.";
    ROS_WARN_STREAM_THROTTLE(1.0, "[PixhawkApi]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  if (!offboard_) {

    ss << "can not disarm, not in OFFBOARD mode";
    ROS_WARN_STREAM_THROTTLE(1.0, "[PixhawkApi]: " << ss.str());
    return std::tuple(false, ss.str());
  }

  offboard_ = false;

  ss << "disarmed";
  ROS_INFO_STREAM_THROTTLE(1.0, "[PixhawkApi]: " << ss.str());
  return std::tuple(false, ss.str());
}

//}

/* callbackOffboard() //{ */

std::tuple<bool, std::string> Api::callbackOffboard(void) {

  offboard_ = true;

  std::stringstream ss;
  ss << "Offboard set";
  ROS_INFO_THROTTLE(1.0, "[PixhawkApi]: %s", ss.str().c_str());
  return {true, ss.str()};
}

//}

// | ------------------------ callbacks ----------------------- |

/* //{ callbackOdom() */

void Api::callbackOdom(mrs_lib::SubscribeHandler<nav_msgs::Odometry> &wrp) {

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

  common_handlers_->publishers.publishOdometryLocal(*odom);

  // | ---------------------- publish gnss ---------------------- |

  double lat;
  double lon;

  mrs_lib::UTMtoLL(odom->pose.pose.position.y + 5249465.43086, odom->pose.pose.position.x + 465710.758973, "32T", lat, lon);

  sensor_msgs::NavSatFix gnss;

  gnss.altitude  = odom->pose.pose.position.z;
  gnss.latitude  = lat;
  gnss.longitude = lon;

  common_handlers_->publishers.publishGNSS(gnss);

  // | --------------------- publish heading -------------------- |

  double heading = mrs_lib::AttitudeConverter(odom->pose.pose.orientation).getHeading();

  mrs_msgs::Float64Stamped hdg;

  hdg.header.stamp = ros::Time::now();
  hdg.value        = heading;

  common_handlers_->publishers.publishMagnetometerHeading(hdg);

  // | ---------------------- publish range --------------------- |

  sensor_msgs::Range rng;

  rng.header.stamp = ros::Time::now();
  rng.max_range    = 40;
  rng.min_range    = 0;
  rng.range        = odom->pose.pose.position.z;

  common_handlers_->publishers.publishDistanceSensor(rng);
}

//}

/* callbackImu() //{ */

void Api::callbackImu(mrs_lib::SubscribeHandler<sensor_msgs::Imu> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Api]: getting IMU");

  sensor_msgs::ImuConstPtr imu = wrp.getMsg();

  common_handlers_->publishers.publishIMU(*imu);
}

//}

/* callbackSimulatorDiag() //{ */

void Api::callbackSimulatorDiag(mrs_lib::SubscribeHandler<mrs_multirotor_simulator::Diagnostics> &wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Api]: getting simulator diagnostics");
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void Api::timerMain([[maybe_unused]] const ros::TimerEvent &event) {

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

  sensor_msgs::BatteryState msg;

  msg.capacity = 100;
  msg.current  = 10.0;
  msg.voltage  = 15.8;
  msg.charge   = 0.8;

  common_handlers_->publishers.publishBatteryState(msg);
}

//}

/* publishRC() //{ */

void Api::publishRC(void) {

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

//}

}  // namespace mrs_uav_simulator_hw_api_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_simulator_hw_api_plugin::Api, mrs_uav_hw_api::MrsUavHwApi)
