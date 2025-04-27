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

using namespace std::string_literals;

#define SIM_VERSION "1.0.0"s

//====================================================================================//
namespace
{

using namespace itl;
using namespace util;

struct Args
{
  bool early_exit_succ;
  bool early_exit_fail;
  bool verbose;
};

Args parse_args(int argc, const char ** argv)
{
  argparse::ArgumentParser program("ITL Simulator"s, SIM_VERSION);
  program.add_argument("--verbose"s)
      .help("enable verbose mode"s)
      .default_value(false)
      .implicit_value(true);

  try
  {
    program.parse_args(argc, argv);
    if (program.get<bool>("--help"s))
    {
      std::cout << program;
      return Args{.early_exit_succ = true};
    }
    if (program.get<bool>("--version"s))
    {
      std::cout << "ITL Simulator Version: "s << SIM_VERSION << std::endl;
      return Args{.early_exit_succ = true};
    }

    const bool verbose = program.get<bool>("--verbose"s);
    std::cout << "Verbose mode: "s << (verbose ? "ON"s : "OFF"s) << std::endl;

    return Args{.verbose = verbose};
  }
  catch (const std::exception & e)
  {
    std::cerr << "Error: "s << e.what() << std::endl;
    std::cerr << "Use --help for more information.\n"s;
    return Args{.early_exit_fail = true};
  }
}

std::string fmt_current_state(const State & curr_state)
{
  std::ostringstream log_stream;
  log_stream << "Current state: x="s << curr_state.x << ", y="s << curr_state.y
             << ", theta="s << curr_state.theta << ", v="s << curr_state.v;

  return log_stream.str();
}

std::string fmt_cost(double cost)
{
  std::ostringstream log_stream;
  log_stream << "Cost with matrices: "s << cost;

  return log_stream.str();
}

}  // namespace

//====================================================================================//
int main(int argc, const char ** argv)
{
  const Args args = parse_args(argc, argv);
  if (args.early_exit_succ) return EXIT_SUCCESS;
  if (args.early_exit_fail) return EXIT_FAILURE;

  auto log = std::make_shared<Logger>(args.verbose);
  log->info("Simulation Starting."s);

  // Run MPC simulation
  // x_k+1 (state vector at time step k i.e. pos/velocity/etc.)
  State curr_state{};
  const std::vector<State> ref_traj = {
      {1, 1, 0, 1},
      {2, 2, 0, 1},  // Some predefined trajectory
      {3, 3, 0, 1},
  };

  const auto controller = MpcController().attach_logger(log);

  const double dt = 0.1;
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

    log->debug(fmt_current_state(curr_state));
  }

  // Example of calculating cost with Q & R (quadratic) weight matrices
  Eigen::MatrixXd Q(4, 4);
  Q << 1.0, 0.0, 0.0, 0.0,  // r1
      0.0, 1.0, 0.0, 0.0,   // r2
      0.0, 0.0, 0.5, 0.0,   // r3
      0.0, 0.0, 0.0, 0.1;   // r4 - weights for state error

  Eigen::MatrixXd R(2, 2);
  R << 0.1, 0.0,  // r1
      0.0, 0.1;   // r2 - weights for control effort

  const Eigen::VectorXd state_error(4);
  const Eigen::VectorXd control_input(2);

  const double cost =
      controller.calculate_cost_matrix(state_error, control_input, Q, R);

  log->debug(fmt_cost(cost));
  log->info("Simulation Finished."s);

  return EXIT_SUCCESS;
}
