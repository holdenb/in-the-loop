#include "MpcController.h"

#include "Units.h"

#include <math.h>

//====================================================================================//
namespace
{

using namespace util;

using namespace std::string_literals;

// Constraints
constexpr double WHEEL_BASE = 1.0;
constexpr double MAX_STEERING_ANGLE = 45_deg_as_rad;
constexpr double MAX_ACCEL = 2.0_mps2;

// Weights for different components of the cost function
constexpr double WEIGHT_POS_ERROR = 1.0_w;
constexpr double WEIGHT_HEADING_ERROR = 0.5_w;
constexpr double WEIGHT_CONTROL_EFFORT = 0.1_w;

}  // namespace

//====================================================================================//
namespace itl
{

MpcController::MpcController() : m_log(std::make_shared<util::NoOpLogger>())
{
  assert(m_log);
}

State MpcController::update(const State & current_state,
                            const ControlInput & input,
                            double dt) const
{
  State new_state;

  new_state.x =
      current_state.x + current_state.v * std::cos(current_state.theta) * dt;
  new_state.y =
      current_state.y + current_state.v * std::cos(current_state.theta) * dt;
  new_state.theta = current_state.theta + (current_state.v / WHEEL_BASE) *
                                              std::tan(input.steering_angle);
  new_state.v = current_state.v + input.accel * dt;

  return new_state;
}

Eigen::VectorXd MpcController::update(const Eigen::VectorXd & current_state,
                                      const Eigen::VectorXd & control_input,
                                      double dt) const
{
  // Assuming:
  // -    state is a 4x1 vector [x, y, theta, v]
  // -    control_input is a 2x1 vector [acceleration, steering_angle]

  // Really should be using a transform in SE(3)...but this is quick & easy
  // for now.
  Eigen::VectorXd new_state(4);

  // x position
  new_state(0) =
      current_state(0) + current_state(3) * std::cos(current_state(2)) * dt;

  // y position
  new_state(1) =
      current_state(1) + current_state(3) * std::sin(current_state(2)) * dt;

  // theta (orientation)
  new_state(2) = current_state(2) + (current_state(3) / WHEEL_BASE) *
                                        std::tan(control_input(1)) * dt;
  // velocity
  new_state(3) = current_state(3) + control_input(0) * dt;

  return new_state;
}

ControlInput MpcController::optimize_control_inputs(
    const State & current_state, const State & reference_state, double dt) const
{
  // Basic brute force search optimization over a grid of control inputs.
  // A more complex approach would be to use gradient descent, etc.
  double best_cost = std::numeric_limits<double>::max();
  ControlInput best_input;

  // Grid search for control inputs
  for (double acc = -2.0; acc <= 2.0; acc += 0.1)
  {
    for (double steering = -45_deg_as_rad; steering <= 45_deg_as_rad;
         steering += 0.1)
    {
      const ControlInput candidate_input{acc, steering};
      if (!within_constraints(candidate_input)) continue;

      // Simulate system forward in time
      const State predicted_state = update(current_state, candidate_input, dt);
      const double cost =
          calculate_cost(predicted_state, reference_state, candidate_input);

      if (!(cost < best_cost)) continue;

      best_cost = cost;
      best_input = candidate_input;
    }
  }

  return best_input;
}

double MpcController::calculate_cost(const State & current_state,
                                     const State & reference_state,
                                     const ControlInput & input) const
{
  const double position_error = std::hypot(
      reference_state.x - current_state.x, reference_state.y, current_state.y);
  const double heading_error =
      std::fabs(reference_state.theta - current_state.theta);
  const double control_effort =
      std::fabs(input.accel) + std::fabs(input.steering_angle);

  // Penalize deviation from the reference trajectory and alo penalize large
  // control inputs (smoother control)
  return WEIGHT_POS_ERROR * position_error +
         WEIGHT_HEADING_ERROR * heading_error +
         WEIGHT_CONTROL_EFFORT * control_effort;
}

double MpcController::calculate_cost(
    const Eigen::VectorXd & current_state,
    const Eigen::VectorXd & reference_state,
    const Eigen::VectorXd & control_input) const
{
  Eigen::VectorXd state_error = reference_state - current_state;

  // Error in x, y
  const double position_error = state_error.head(2).norm();
  // Error in orientation
  const double heading_error = std::fabs(state_error(2));
  // Control effort for both acceleration and steering
  const double control_effort = control_input.norm();

  return WEIGHT_POS_ERROR * position_error +
         WEIGHT_HEADING_ERROR * heading_error +
         WEIGHT_CONTROL_EFFORT * control_effort;
}

double MpcController::calculate_cost_matrix(
    const Eigen::VectorXd & state_error,
    const Eigen::VectorXd & control_input,
    const Eigen::MatrixXd & Q,
    const Eigen::MatrixXd & R) const
{
  m_log->debug("Calculating cost matrix."s);

  // Quadratic cost: J = state_error^T Q state_error + control_input^T R
  // control_input Q is a positive semi-definite matrix that penalizes
  // deviations of the predicted state from the reference state Measures how
  // far the predicted state is from the desired state (trajectory tracking)
  const double state_error_penalty = state_error.transpose() * Q * state_error;

  // R is a positive definite matrix that penalizes the control effort
  // (i.e., the magnitude of control inputs)
  // Penalizes large control actions to encourage smoother control
  const double control_effort_penalty =
      control_input.transpose() * R * control_input;

  return state_error_penalty + control_effort_penalty;
}

MpcController & MpcController::attach_logger(std::shared_ptr<util::ILogger> log)
{
  assert(log);
  m_log = log;

  return *this;
}

bool MpcController::within_constraints(const ControlInput & input) const
{
  if (std::fabs(input.steering_angle) > MAX_STEERING_ANGLE) return false;
  if (std::fabs(input.accel) > MAX_ACCEL) return false;

  return true;
}

}  // namespace itl
