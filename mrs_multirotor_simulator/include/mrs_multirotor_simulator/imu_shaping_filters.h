#ifndef IMU_SHAPING_FILTERS_H
#define IMU_SHAPING_FILTERS_H

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/param_loader.h>

#include <mrs_multirotor_simulator/imu_noise_shaper.h>

#include <map>
#include <memory>
#include <mutex>

namespace mrs_multirotor_simulator
{

struct ImuShapingFilter_t
{
  ImuNoiseShaper::AxisFilters_t gyro;
  ImuNoiseShaper::AxisFilters_t accel;
};

/**
 * @brief Thread-safe, lazily-loaded storage of the IMU noise-shaping filters, shared by all the UAVs.
 *
 * The filters are loaded from 'imu_noise_filters/<name>/{gyro,accel}/{x,y,z}' on the first request and cached.
 */
class ImuShapingFilters {

public:
  /**
   * @param parent_param_loader the YAML files are copied from this param loader
   */
  ImuShapingFilters(const rclcpp::Node::SharedPtr &node, const mrs_lib::ParamLoader &parent_param_loader);

  /**
   * @brief returns the filter, loads it on the first request
   *
   * @return nullptr when the filter could not be loaded
   */
  std::shared_ptr<const ImuShapingFilter_t> getFilter(const std::string &name);

private:
  rclcpp::Node::SharedPtr node_;

  // * separate param loader with printing disabled, the filters have hundreds of coefficients
  std::unique_ptr<mrs_lib::ParamLoader> param_loader_;

  std::map<std::string, std::shared_ptr<const ImuShapingFilter_t>> filters_;
  std::mutex                                                       mutex_filters_;
};

} // namespace mrs_multirotor_simulator

#endif // IMU_SHAPING_FILTERS_H
