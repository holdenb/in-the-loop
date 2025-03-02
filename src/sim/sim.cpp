#include "itl/MpcController.h"
#include "itl/State.h"

#include <Eigen/Dense>

#include <iostream>
#include <vector>

using namespace itl;

//====================================================================//
namespace
{

void logCurrentState(State const & curr_state)
{
  std::cout << "Current state: x=" << curr_state.x << ", y=" << curr_state.y
            << ", theta=" << curr_state.theta << ", v=" << curr_state.v
            << std::endl;
}

}  // namespace

//====================================================================//
int main(int argc, const char ** argv)
{
  // std::optional<int> seed = std::nullopt;
  // int num_players = DEFAULT_NUM_PLAYERS;
  // int num_rounds = DEFAULT_NUM_ROUNDS;
  //
  // struct option long_options[] = {{"players", required_argument, 0, 'p'},
  //                                 {"rounds", required_argument, 0, 'r'},
  //                                 {"seed", required_argument, 0, 's'},
  //                                 {"help", no_argument, 0, 'h'},
  //                                 {0, 0, 0, 0}};
  //
  // int opt;
  // int option_index = 0;
  // while ((opt = getopt_long(
  //             _argc, _argv, "p:r:s:h", long_options, &option_index)) != -1)
  // {
  //   switch (opt)
  //   {
  //     case 'p':
  //       num_players = std::stoi(optarg);
  //       if (num_players <= 0 || num_players > MAX_NUM_PLAYERS)
  //       {
  //         std::cerr << "Number of players must be a positive integer "
  //                      "and no more than "
  //                   << MAX_NUM_PLAYERS << ".\n";
  //         return EXIT_FAILURE;
  //       }
  //       break;
  //     case 'r':
  //       num_rounds = std::stoi(optarg);
  //       if (num_rounds <= 0 || num_rounds > MAX_NUM_ROUNDS)
  //       {
  //         std::cerr << "Number of rounds must be a positive integer "
  //                      "and no more than "
  //                   << MAX_NUM_ROUNDS << ".\n";
  //         return EXIT_FAILURE;
  //       }
  //       break;
  //     case 's':
  //       seed = std::stoi(optarg);
  //       if (seed <= 0 || seed > MAX_SEED)
  //       {
  //         std::cerr << "Seed value must be a positive integer and no "
  //                      "more than "
  //                   << MAX_SEED << ".\n";
  //         return EXIT_FAILURE;
  //       }
  //       break;
  //     case 'h':
  //       std::cout << "Usage: " << _argv[0] << " [options]\n"
  //                 << "Options:\n"
  //                 << "  -p, --players [num]  Set the number of players (1-"
  //                 << MAX_NUM_PLAYERS << ", default: " << DEFAULT_NUM_PLAYERS
  //                 << ")\n"
  //                 << "  -r, --rounds [num]   Set the number of rounds (1-"
  //                 << MAX_NUM_ROUNDS << ", default: " << DEFAULT_NUM_ROUNDS
  //                 << ")\n"
  //                 << "  -s, --seed [num]     Set the seed value for "
  //                    "randomization\n"
  //                 << "  -h, --help           Show this help message\n";
  //       return EXIT_SUCCESS;
  //     default:
  //       std::cerr << "Use --help to see available options.\n";
  //       return EXIT_FAILURE;
  //   }
  // }

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
