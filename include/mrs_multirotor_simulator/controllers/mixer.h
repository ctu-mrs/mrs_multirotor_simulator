#ifndef MIXER_H
#define MIXER_H

#include <mrs_multirotor_simulator/multirotor_model.h>
#include <mrs_multirotor_simulator/controllers/references.h>

namespace mrs_multirotor_simulator
{

class Mixer {

public:
  class Params {
  public:
    bool desaturation = true;
  };

  Mixer();
  Mixer(const ModelParams& model_params);

  void setParams(const Params& params);

  reference::Actuators getControlSignal(const reference::ControlGroup& reference);

private:
  ModelParams model_params_;
  Params      params_;

  Eigen::MatrixXd allocation_matrix_inv_;
};

}  // namespace mrs_multirotor_simulator

#endif  // MIXER_H
