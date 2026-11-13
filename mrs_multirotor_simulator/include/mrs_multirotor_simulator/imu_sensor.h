#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_server_handler.h>

#include <mrs_multirotor_simulator/uav_system/multirotor_model.hpp>
#include <mrs_multirotor_simulator/imu_noise_shaper.h>
#include <mrs_multirotor_simulator/imu_shaping_filters.h>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mrs_msgs/srv/float64_srv.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <atomic>
#include <mutex>

namespace mrs_multirotor_simulator
{

struct ImuNoiseParams_t
{
  bool enabled = false;

  // name of the filter in 'imu_noise_filters', "none" = white noise, bias and stddev in physical units
  std::string shaping_filter_name = "none";

  double accel_bias   = 0.0;
  double accel_stddev = 0.0;

  double gyro_bias            = 0.0;
  double gyro_stddev          = 0.0;
  bool   gyro_separate_z_bias = false;
  double gyro_z_bias          = 0.0;
};

struct ImuSensorParams_t
{
  std::string topic;                // e.g., "~/uav1/imu_lidar"
  std::string noiseless_topic;      // e.g., "~/uav1/imu_lidar_default", "" = do not publish
  std::string service_prefix;       // e.g., "~/uav1/imu_lidar/"
  std::string noise_toggle_service; // e.g., "set_noise", relative to service_prefix
  std::string frame_id;

  // pose of the IMU in the FCU frame
  Eigen::Vector3d position    = Eigen::Vector3d::Zero();
  Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity(); // columns = IMU axes expressed in the FCU frame

  ImuNoiseParams_t noise;
};

/**
 * @brief A simulated IMU rigidly attached to the UAV body at an arbitrary pose.
 */
class ImuSensor {

public:
  /**
   * @param shaping_filters the noise-shaping filters, loaded lazily only when the noise gets enabled
   */
  ImuSensor(const rclcpp::Node::SharedPtr &node, const rclcpp::CallbackGroup::SharedPtr &cbgrp_ss, const ImuSensorParams_t &params,
            const std::shared_ptr<ImuShapingFilters> &shaping_filters);

  /**
   * @param state          the current UAV state
   * @param imu_acc        the specific force in the origin of the FCU frame, expressed in the FCU frame
   * @param omega_dot      the angular acceleration of the UAV body, expressed in the FCU frame
   */
  void publish(const rclcpp::Time &stamp, const MultirotorModel::State &state, const Eigen::Vector3d &imu_acc, const Eigen::Vector3d &omega_dot);

  geometry_msgs::msg::TransformStamped getStaticTF(const std::string &parent_frame, const rclcpp::Time &stamp) const;

  std::string getFrameId(void) const;

private:
  rclcpp::Node::SharedPtr node_;

  std::atomic<bool> is_initialized_ = false;

  ImuSensorParams_t _params_;

  // | -------------------------- noise ------------------------- |

  ImuNoiseParams_t noise_params_;
  ImuNoiseShaper   noise_shaper_;
  std::mutex       mutex_noise_;

  std::shared_ptr<ImuShapingFilters> shaping_filters_;
  bool                               shaping_filter_set_ = false;

  void updateNoiseShaperParams(void);

  bool setShapingFilter(void);

  // | ----------------------- publishers ----------------------- |

  std::shared_ptr<mrs_lib::PublisherHandler<sensor_msgs::msg::Imu>> ph_imu_;
  std::shared_ptr<mrs_lib::PublisherHandler<sensor_msgs::msg::Imu>> ph_imu_noiseless_;

  // | --------------------- service servers -------------------- |

  mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>    ss_set_noise_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::Float64Srv> ss_set_gyro_bias_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::Float64Srv> ss_set_gyro_stddev_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::Float64Srv> ss_set_accel_bias_;
  mrs_lib::ServiceServerHandler<mrs_msgs::srv::Float64Srv> ss_set_accel_stddev_;

  bool callbackSetNoise(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  bool callbackSetGyroBias(const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request>  request,
                           const std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> response);

  bool callbackSetGyroStddev(const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request>  request,
                             const std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> response);

  bool callbackSetAccelBias(const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request>  request,
                            const std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> response);

  bool callbackSetAccelStddev(const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request>  request,
                              const std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> response);
};

} // namespace mrs_multirotor_simulator

#endif // IMU_SENSOR_H
