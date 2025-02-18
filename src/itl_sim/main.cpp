#include <iostream>

#include "itl/controller.h"

//---------------------------------------------------------------------//
void logCurrentState(itl::State const & curr_state)
{
  std::cout << "Current state: x=" << curr_state.x << ", y=" << curr_state.y
            << ", theta=" << curr_state.theta << ", v=" << curr_state.v
            << std::endl;
}

//---------------------------------------------------------------------//
int main(int argc, const char ** argv)
{
  // Run MPC simulation
  // x_k+1 (state vector at time step k i.e. pos/velocity/etc.)
  itl::State curr_state{};
  const std::vector<itl::State> ref_traj = {
      {1, 1, 0, 1}, {2, 2, 0, 1}, {3, 3, 0, 1}  // Some predefined trajectory
  };

  auto controller = itl::Controller();

  double dt = 0.1;
  for (const auto & ref_state : ref_traj)
  {
    // Controller f(x_k, u_k) where f = update(optimize(x_k, u_k))
    // where u_k represents the control input vector at time step k
    // i.e. accel/heading angle/etc.
    // f also represents the "system dynamics" (can either be linear or a
    // non-linear function)
    itl::ControlInput optimal_control =
        controller.optimize_control_inputs(curr_state, ref_state, dt);
    curr_state = controller.update(curr_state, optimal_control, dt);

    logCurrentState(curr_state);
  }

  // Example of calculating cost with Q & R (quadratic) weight matrices
  Eigen::MatrixXd Q(4, 4);
  Q << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
      0.0, 0.1;  // Weights for state error

  Eigen::MatrixXd R(2, 2);
  R << 0.1, 0.0, 0.0, 0.1;  // Weights for control effort

  const Eigen::VectorXd state_error(4);
  const Eigen::VectorXd control_input(2);

  const double cost =
      controller.calculate_cost_matrix(state_error, control_input, Q, R);
  std::cout << "Cost with matrices: " << cost << std::endl;

  return EXIT_SUCCESS;
}
