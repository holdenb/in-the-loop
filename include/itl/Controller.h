#pragma once

#include "State.h"
#include "ControlInput.h"

namespace itl
{

/**
 * @class Controller
 * @brief
 *
 */
class Controller
{
public:
  virtual ~Controller() = default;

  /**
   * @brief
   *
   * @param current_state
   * @param input
   * @param dt
   * @return
   */
  virtual State update(const State & current_state,
                       ControlInput const & input,
                       double dt) const = 0;

protected:
  Controller() = default;
};

}  // namespace itl
