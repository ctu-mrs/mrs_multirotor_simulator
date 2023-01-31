#include <controllers/mixer.h>

namespace mrs_multirotor_simulator
{

/* Mixer() //{ */

Mixer::Mixer() {
}

//}

/* setParams() //{ */

void Mixer::setParams(const Params& params) {

  params_ = params;

  Eigen::MatrixXd allocation_tmp = params.allocation_matrix;

  // | ------------ normalize the third row - yaw mix ----------- |

  {
    double sum_abs = 0;
    for (int j = 0; j < allocation_tmp.cols(); j++) {
      sum_abs += abs(params.allocation_matrix(2, j));
    }

    allocation_tmp.row(2) = allocation_tmp.row(2) / (0.5 * sum_abs * allocation_tmp.cols());
  }

  // | --------- normalize the forth row - throttle mix --------- |

  {
    double sum_abs = 0;
    for (int j = 0; j < allocation_tmp.cols(); j++) {
      sum_abs += abs(params.allocation_matrix(3, j));
    }

    allocation_tmp.row(3) = allocation_tmp.row(3) / (sum_abs);
  }

  allocation_matrix_inv_ = allocation_tmp.transpose() * (allocation_tmp * allocation_tmp.transpose()).inverse();
}

//}

/* getControlSignal() //{ */

reference::Actuators Mixer::getControlSignal(const reference::ControlGroup& reference) {

  Eigen::Vector4d ctrl_group(reference.roll, reference.pitch, reference.yaw, reference.throttle);

  reference::Actuators actuators;
  actuators.motors = Eigen::VectorXd::Zero(params_.n_motors);

  actuators.motors = allocation_matrix_inv_ * ctrl_group;

  // desaturation
  {

    double min = actuators.motors.minCoeff();

    if (min < 0.0) {
      actuators.motors.array() += abs(min);
    }

    double max = actuators.motors.maxCoeff();

    if (max > 1.0) {

      if (reference.throttle > 1e-2) {

        // scale down roll/pitch/yaw actions to maintain desired throttle
        for (int i = 0; i < 3; i++) {
          ctrl_group(i) = ctrl_group(i) / (actuators.motors.mean() / reference.throttle);
        }

        actuators.motors = allocation_matrix_inv_ * ctrl_group;

      } else {
        actuators.motors /= max;
      }
    }
  }

  return actuators;
}

//}

}  // namespace mrs_multirotor_simulator
