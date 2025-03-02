#pragma once

#include "Controller.h"
#include "ControlInput.h"
#include "State.h"

#include <Eigen/Dense>

namespace itl
{

class MpcController : public Controller
{
public:
  ~MpcController();
  MpcController();

  State update(const State & current_state,
               ControlInput const & input,
               double dt) override;

  Eigen::VectorXd update(const Eigen::VectorXd & current_state,
                         const Eigen::VectorXd & control_input,
                         double dt);

  ControlInput optimize_control_inputs(const State & current_state,
                                       const State & reference_state,
                                       double dt);

  double calculate_cost_matrix(Eigen::VectorXd state_error,
                               Eigen::VectorXd control_input,
                               Eigen::MatrixXd Q,
                               Eigen::MatrixXd R);

private:
  double calculate_cost(const State & current_state,
                        const State & reference_state,
                        const ControlInput & input);

  double calculate_cost(const Eigen::VectorXd & current_state,
                        const Eigen::VectorXd & reference_state,
                        const Eigen::VectorXd & control_input);

  bool within_constraints(const ControlInput & input);
};

}  // namespace itl
