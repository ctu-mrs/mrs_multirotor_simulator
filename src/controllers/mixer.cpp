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

  allocation_matrix_inv_ = allocation_tmp.transpose() * (allocation_tmp * allocation_tmp.transpose()).inverse();
}

reference::Actuators Mixer::getControlSignal(const reference::ControlGroup& reference) {

  Eigen::Vector4d ctrl_group(reference.roll, reference.pitch, reference.yaw, reference.throttle);

  reference::Actuators actuators;
  actuators.motors = Eigen::VectorXd::Zero(params_.n_motors);

  actuators.motors = allocation_matrix_inv_ * ctrl_group;

  // TODO: implement desaturation
  for (int i = 0; i < params_.n_motors; i++) {
    if (actuators.motors[i] > 1.0) {
      actuators.motors[i] = 1.0;
    } else if (actuators.motors[i] < 0.0) {
      actuators.motors[i] = 0.0;
    }
  }

  return actuators;
}

}  // namespace mrs_multirotor_simulator
