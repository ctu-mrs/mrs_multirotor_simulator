#ifndef MIXER_H
#define MIXER_H

#include "references.h"

namespace mrs_multirotor_simulator
{

class Mixer {

public:
  class Params {
  public:
    bool desaturation = true;
  };

  Mixer();
  Mixer(const MultirotorModel::ModelParams& model_params);

  void setParams(const Params& params);

  reference::Actuators getControlSignal(const reference::ControlGroup& reference);

  /**
   * @brief get the normalized allocation matrix used in the mixer
   *
   * @return
   */
  Eigen::MatrixXd getAllocationMatrix(void);

private:
  MultirotorModel::ModelParams model_params_;
  Params                       params_;

  void calculateAllocation(void);

  Eigen::MatrixXd allocation_matrix_inv_;
};

}  // namespace mrs_multirotor_simulator

#include "impl/mixer_impl.hpp"

#endif  // MIXER_H
