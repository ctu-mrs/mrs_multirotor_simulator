#include <mrs_multirotor_simulator/imu_noise_shaper.h>

#include <stdexcept>

namespace mrs_multirotor_simulator
{

// | --------------------- ImuNoiseShaper --------------------- |

/* ImuNoiseShaper() //{ */

ImuNoiseShaper::ImuNoiseShaper() : gen_(std::random_device{}()) {
}

//}

/* setAccelNoiseParams() //{ */

void ImuNoiseShaper::setAccelNoiseParams(const double bias, const double stddev) {

  accel_bias_   = bias;
  accel_stddev_ = stddev;
}

//}

/* setGyroNoiseParams() //{ */

void ImuNoiseShaper::setGyroNoiseParams(const double bias, const double stddev, const bool separate_z_bias, const double z_bias) {

  gyro_bias_            = bias;
  gyro_stddev_          = stddev;
  gyro_separate_z_bias_ = separate_z_bias;
  gyro_z_bias_          = z_bias;
}

//}

/* setAccelShapingFilter() //{ */

void ImuNoiseShaper::setAccelShapingFilter(const AxisFilters_t &b_coeffs) {

  for (size_t i = 0; i < 3; i++) {

    if (b_coeffs.at(i).empty()) {
      throw std::invalid_argument("ImuNoiseShaper: the shaping filter coefficients must not be empty");
    }

    // FIR filter, the a[0] coefficient is not used
    accel_filters_.at(i) = mrs_lib::IirFilter(std::vector<double>{1.0}, b_coeffs.at(i));
  }
}

//}

/* setGyroShapingFilter() //{ */

void ImuNoiseShaper::setGyroShapingFilter(const AxisFilters_t &b_coeffs) {

  for (size_t i = 0; i < 3; i++) {

    if (b_coeffs.at(i).empty()) {
      throw std::invalid_argument("ImuNoiseShaper: the shaping filter coefficients must not be empty");
    }

    // FIR filter, the a[0] coefficient is not used
    gyro_filters_.at(i) = mrs_lib::IirFilter(std::vector<double>{1.0}, b_coeffs.at(i));
  }
}

//}

/* addNoise() //{ */

void ImuNoiseShaper::addNoise(Eigen::Vector3d &gyro, Eigen::Vector3d &accel) {

  const double gyro_z_bias = gyro_separate_z_bias_ ? gyro_z_bias_ : gyro_bias_;

  gyro(0) += gyro_filters_[0].iterate(sample(gyro_bias_, gyro_stddev_));
  gyro(1) += gyro_filters_[1].iterate(sample(gyro_bias_, gyro_stddev_));
  gyro(2) += gyro_filters_[2].iterate(sample(gyro_z_bias, gyro_stddev_));

  for (int i = 0; i < 3; i++) {
    accel(i) += accel_filters_[i].iterate(sample(accel_bias_, accel_stddev_));
  }
}

//}

/* sample() //{ */

double ImuNoiseShaper::sample(const double bias, const double stddev) {

  return bias + stddev * normal_(gen_);
}

//}

} // namespace mrs_multirotor_simulator
