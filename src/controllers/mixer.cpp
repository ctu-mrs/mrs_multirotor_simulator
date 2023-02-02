#include <mrs_multirotor_simulator/controllers/mixer.h>

namespace mrs_multirotor_simulator
{

/* Mixer() //{ */

Mixer::Mixer() {
}

Mixer::Mixer(const ModelParams& model_params) {

  model_params_ = model_params;

  Eigen::MatrixXd allocation_tmp = model_params_.allocation_matrix;

  allocation_matrix_inv_ = allocation_tmp.transpose() * (allocation_tmp * allocation_tmp.transpose()).inverse();

  // | ------------- normalize the allocation matrix ------------ |
  // this will make it match the PX4 control group mixing

  // the first two columns (roll, pitch)
  for (int i = 0; i < model_params_.n_motors; i++) {
    allocation_matrix_inv_.block(i, 0, 1, 2).normalize();
  }

  // the 3rd column (yaw)
  for (int i = 0; i < model_params_.n_motors; i++) {
    if (allocation_matrix_inv_(i, 2) > 1e-2) {
      allocation_matrix_inv_(i, 2) = 1.0;
    } else if (allocation_matrix_inv_(i, 2) < -1e-2) {
      allocation_matrix_inv_(i, 2) = -1.0;
    } else {
      allocation_matrix_inv_(i, 2) = 0.0;
    }
  }

  // the 4th column (throttle)
  for (int i = 0; i < model_params_.n_motors; i++) {
    allocation_matrix_inv_(i, 3) = 1.0;
  }
}

//}

/* setParams() //{ */

void Mixer::setParams(const Params& params) {

  params_ = params;
}

//}

/* getControlSignal() //{ */

reference::Actuators Mixer::getControlSignal(const reference::ControlGroup& reference) {

  Eigen::Vector4d ctrl_group(reference.roll, reference.pitch, reference.yaw, reference.throttle);

  reference::Actuators actuators;
  actuators.motors = Eigen::VectorXd::Zero(model_params_.n_motors);

  actuators.motors = allocation_matrix_inv_ * ctrl_group;

  if (params_.desaturation) {

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
