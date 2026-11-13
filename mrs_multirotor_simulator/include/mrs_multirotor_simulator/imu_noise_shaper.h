#ifndef IMU_NOISE_SHAPER_H
#define IMU_NOISE_SHAPER_H

#include <Eigen/Dense>

#include <mrs_lib/iir_filter.h>

#include <array>
#include <random>
#include <vector>

namespace mrs_multirotor_simulator
{

/* class ImuNoiseShaper //{ */

/**
 * @brief Generates (optionally spectrally-shaped) noise for an IMU.
 *
 * For each axis, a gaussian noise sample N(bias, stddev^2) is passed through a FIR filter (mrs_lib::IirFilter with only
 * the feed-forward coefficients) and added to the measurement.
 * Without a shaping filter (the default), the noise is white and the bias and stddev are in physical units.
 */
class ImuNoiseShaper {

public:
  using AxisFilters_t = std::array<std::vector<double>, 3>;

  ImuNoiseShaper();

  void setAccelNoiseParams(const double bias, const double stddev);

  /**
   * @param separate_z_bias when true, the z axis uses z_bias instead of bias
   */
  void setGyroNoiseParams(const double bias, const double stddev, const bool separate_z_bias = false, const double z_bias = 0.0);

  void setAccelShapingFilter(const AxisFilters_t &b_coeffs);
  void setGyroShapingFilter(const AxisFilters_t &b_coeffs);

  void addNoise(Eigen::Vector3d &gyro, Eigen::Vector3d &accel);

private:
  std::mt19937                     gen_;
  std::normal_distribution<double> normal_{0.0, 1.0};

  double accel_bias_   = 0.0;
  double accel_stddev_ = 0.0;

  double gyro_bias_            = 0.0;
  double gyro_stddev_          = 0.0;
  bool   gyro_separate_z_bias_ = false;
  double gyro_z_bias_          = 0.0;

  // one filter per axis, default-constructed = pass-through
  std::array<mrs_lib::IirFilter, 3> accel_filters_;
  std::array<mrs_lib::IirFilter, 3> gyro_filters_;

  double sample(const double bias, const double stddev);
};

//}

} // namespace mrs_multirotor_simulator

#endif // IMU_NOISE_SHAPER_H
