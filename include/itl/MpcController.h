#pragma once

#include "Controller.h"
#include "ControlInput.h"
#include "Logger.h"
#include "State.h"

#include <Eigen/Dense>

#include <memory>

namespace itl
{

/**
 * @class MpcController
 * @brief
 *
 */
class MpcController : public Controller
{
public:
  ~MpcController() = default;
  MpcController();

  /**
   * @brief
   *
   * @param current_state
   * @param input
   * @param dt
   * @return
   */
  State update(const State & current_state,
               const ControlInput & input,
               double dt) const override;

  /**
   * @brief
   *
   * @param current_state
   * @param control_input
   * @param dt
   */
  Eigen::VectorXd update(const Eigen::VectorXd & current_state,
                         const Eigen::VectorXd & control_input,
                         double dt) const;

  /**
   * @brief
   *
   * @param current_state
   * @param reference_state
   * @param dt
   * @return
   */
  ControlInput optimize_control_inputs(const State & current_state,
                                       const State & reference_state,
                                       double dt) const;

  /**
   * @brief
   *
   * @param state_error
   * @param control_input
   * @param Q
   * @param R
   * @return
   */
  double calculate_cost_matrix(const Eigen::VectorXd & state_error,
                               const Eigen::VectorXd & control_input,
                               const Eigen::MatrixXd & Q,
                               const Eigen::MatrixXd & R) const;

  /**
   * @brief
   *
   * @param log
   * @return
   */
  MpcController & attach_logger(std::shared_ptr<util::ILogger> log);

private:
  /**
   * @brief
   *
   * @param current_state
   * @param reference_state
   * @param input
   * @return
   */
  double calculate_cost(const State & current_state,
                        const State & reference_state,
                        const ControlInput & input) const;

  /**
   * @brief
   *
   * @param current_state
   * @param reference_state
   * @param control_input
   * @return
   */
  double calculate_cost(const Eigen::VectorXd & current_state,
                        const Eigen::VectorXd & reference_state,
                        const Eigen::VectorXd & control_input) const;

  /**
   * @brief
   *
   * @param input
   * @return
   */
  bool within_constraints(const ControlInput & input) const;

  /**
   * @brief
   */
  std::shared_ptr<util::ILogger> m_log;
};

}  // namespace itl
