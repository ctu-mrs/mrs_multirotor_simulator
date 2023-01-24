#include <controllers/mixer.h>
#include <math.h>
#include <iostream>

namespace mrs_multirotor_simulator
{

// constructor
Mixer::Mixer() {
}

void Mixer::setParams(const Params& params) {

  params_ = params;

  Eigen::MatrixXd allocation_tmp = params.allocation_matrix;

  for (int i = 0; i < allocation_tmp.rows(); i++) {

    double sum_abs = 0;

    for (int j = 0; j < allocation_tmp.cols(); j++) {
      sum_abs += abs(params.allocation_matrix(i, j));
    }

    allocation_tmp.row(i) = allocation_tmp.row(i) / sum_abs;
  }

  allocation_matrix_inv_ = allocation_tmp.inverse();

  std::cout << "Mixer's allocation = " << std::endl << allocation_matrix_inv_ << std::endl;
}

reference::Motors Mixer::getControlSignal(const reference::ControlGroup& reference) {

  Eigen::Vector4d ctrl_group(reference.roll, reference.pitch, reference.yaw, reference.throttle);

  reference::Motors motors;
  motors.motors = Eigen::VectorXd::Zero(params_.n_motors);

  motors.motors = allocation_matrix_inv_ * ctrl_group;

  // TODO: implement desaturation
  for (int i = 0; i < params_.n_motors; i++) {
    if (motors.motors[i] > 1.0) {
      motors.motors[i] = 1.0;
    } else if (motors.motors[i] < 0.0) {
      motors.motors[i] = 0.0;
    }
  }

  return motors;
}

}  // namespace mrs_multirotor_simulator
