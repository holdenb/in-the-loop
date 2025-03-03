#include "itl/MpcController.h"
#include "itl/State.h"

#include "util/Logger.h"

#include "argparse.hpp"

#include <Eigen/Dense>

#include <getopt.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <vector>
#include <string>

using namespace itl;
using namespace util;

#define SIM_VERSION "1.0.0"

//====================================================================//
namespace
{

std::string fmtCurrentState(State const & curr_state)
{
  std::ostringstream log_stream;
  log_stream << "Current state: x=" << curr_state.x << ", y=" << curr_state.y
             << ", theta=" << curr_state.theta << ", v=" << curr_state.v;

  return log_stream.str();
}

std::string fmtCost(double cost)
{
  std::ostringstream log_stream;
  log_stream << "Cost with matrices: " << cost;

  return log_stream.str();
}

}  // namespace

//====================================================================//
int main(int argc, const char ** argv)
{
  argparse::ArgumentParser program("ITL Simulator", SIM_VERSION);

  program.add_argument("--verbose")
      .help("enable verbose mode")
      .default_value(false)
      .implicit_value(true);

  try
  {
    program.parse_args(argc, argv);
    if (program.get<bool>("--help"))
    {
      std::cout << program;
      return EXIT_SUCCESS;
    }
    if (program.get<bool>("--version"))
    {
      std::cout << "ITL Simulator Version: " << SIM_VERSION << std::endl;
      return EXIT_SUCCESS;
    }

    bool verbose = program.get<bool>("--verbose");
    std::cout << "Verbose mode: " << (verbose ? "ON" : "OFF") << std::endl;
  }
  catch (const std::exception & e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
    std::cerr << "Use --help for more information.\n";
    return EXIT_FAILURE;
  }

  // TODO: Set verbose flag on logger
  auto log = std::make_unique<Logger>();

  // Run MPC simulation
  // x_k+1 (state vector at time step k i.e. pos/velocity/etc.)
  State curr_state{};
  const std::vector<State> ref_traj = {
      {1, 1, 0, 1}, {2, 2, 0, 1}, {3, 3, 0, 1}  // Some predefined trajectory
  };

  auto controller = MpcController();

  double dt = 0.1;
  for (const auto & ref_state : ref_traj)
  {
    // Controller f(x_k, u_k) where f = update(optimize(x_k, u_k))
    // where u_k represents the control input vector at time step k
    // i.e. accel/heading angle/etc.
    // f also represents the "system dynamics" (can either be linear or a
    // non-linear function)
    ControlInput optimal_control =
        controller.optimize_control_inputs(curr_state, ref_state, dt);

    curr_state = controller.update(curr_state, optimal_control, dt);

    log->debug(fmtCurrentState(curr_state));
  }

  // Example of calculating cost with Q & R (quadratic) weight matrices
  Eigen::MatrixXd Q(4, 4);
  Q << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
      0.0,
      0.1;  // Weights for state error

  Eigen::MatrixXd R(2, 2);
  R << 0.1, 0.0, 0.0, 0.1;  // Weights for control effort

  const Eigen::VectorXd state_error(4);
  const Eigen::VectorXd control_input(2);

  const double cost =
      controller.calculate_cost_matrix(state_error, control_input, Q, R);

  log->debug(fmtCost(cost));

  return EXIT_SUCCESS;
}
