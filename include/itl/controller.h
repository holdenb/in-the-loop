#pragma once

#include <Eigen/Dense>

namespace itl
{

struct State_t
{
  double x;
  double y;
  double theta;
  double v;
};

struct ControlInput_t
{
  double accel;
  double steering_angle;
};

class Controller_t
{
public:
  ~Controller_t();
  Controller_t();

  ControlInput_t optimize_control_inputs(const State_t & current_state,
                                         const State_t & reference_state,
                                         double dt);

  State_t update(const State_t & current_state,
                 ControlInput_t const & input,
                 double dt);

  Eigen::VectorXd update(const Eigen::VectorXd & current_state,
                         const Eigen::VectorXd & control_input,
                         double dt);

  double calculate_cost_matrix(Eigen::VectorXd state_error,
                               Eigen::VectorXd control_input,
                               Eigen::MatrixXd Q,
                               Eigen::MatrixXd R);

private:
  double calculate_cost(const State_t & current_state,
                        const State_t & reference_state,
                        const ControlInput_t & input);

  double calculate_cost(const Eigen::VectorXd & current_state,
                        const Eigen::VectorXd & reference_state,
                        const Eigen::VectorXd & control_input);

  bool within_constraints(const ControlInput_t & input);
};

}  // namespace itl
