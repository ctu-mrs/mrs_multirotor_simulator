#ifndef MIXER_H
#define MIXER_H

#include <controllers/references.h>

namespace mrs_multirotor_simulator
{

class Mixer {

public:
  struct Params
  {
    int             n_motors;
    bool            desaturation;
    Eigen::MatrixXd allocation_matrix;
  };

  Mixer();

  void setParams(const Params& params);

  reference::Actuators getControlSignal(const reference::ControlGroup& reference);

private:
  Params params_;

  Eigen::MatrixXd allocation_matrix_inv_;
};

}  // namespace mrs_multirotor_simulator

#endif  // MIXER_H
