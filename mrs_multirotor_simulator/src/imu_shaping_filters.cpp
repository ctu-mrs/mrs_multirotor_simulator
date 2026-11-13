#include <mrs_multirotor_simulator/imu_shaping_filters.h>

namespace mrs_multirotor_simulator
{

/* ImuShapingFilters() //{ */

ImuShapingFilters::ImuShapingFilters(const rclcpp::Node::SharedPtr &node, const mrs_lib::ParamLoader &parent_param_loader) {

  node_ = node;

  param_loader_ = std::make_unique<mrs_lib::ParamLoader>(node_, false, node_->get_name() + std::string("_imu_shaping_filters"));

  param_loader_->copyYamls(parent_param_loader);
}

//}

/* getFilter() //{ */

std::shared_ptr<const ImuShapingFilter_t> ImuShapingFilters::getFilter(const std::string &name) {

  std::scoped_lock lock(mutex_filters_);

  if (filters_.contains(name)) {
    return filters_.at(name);
  }

  const std::string yaml_prefix = "imu_noise_filters/" + name + "/";

  auto filter = std::make_shared<ImuShapingFilter_t>();

  param_loader_->resetLoadedSuccessfully();

  // reusable, a failed attempt might be repeated
  param_loader_->loadParamReusable(yaml_prefix + "gyro/x", filter->gyro[0]);
  param_loader_->loadParamReusable(yaml_prefix + "gyro/y", filter->gyro[1]);
  param_loader_->loadParamReusable(yaml_prefix + "gyro/z", filter->gyro[2]);
  param_loader_->loadParamReusable(yaml_prefix + "accel/x", filter->accel[0]);
  param_loader_->loadParamReusable(yaml_prefix + "accel/y", filter->accel[1]);
  param_loader_->loadParamReusable(yaml_prefix + "accel/z", filter->accel[2]);

  bool coeffs_empty = false;

  for (size_t i = 0; i < 3; i++) {
    coeffs_empty |= filter->gyro[i].empty() || filter->accel[i].empty();
  }

  if (!param_loader_->loadedSuccessfully() || coeffs_empty) {
    RCLCPP_ERROR(node_->get_logger(), "could not load the IMU noise shaping filter '%s'", name.c_str());
    return nullptr;
  }

  RCLCPP_INFO(node_->get_logger(), "loaded the IMU noise shaping filter '%s' (%d coefficients)", name.c_str(), int(filter->gyro[0].size()));

  filters_[name] = filter;

  return filter;
}

//}

} // namespace mrs_multirotor_simulator
