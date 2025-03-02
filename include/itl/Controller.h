#pragma once

#include "State.h"
#include "ControlInput.h"

namespace itl
{

class Controller
{
public:
  virtual ~Controller() = default;

  virtual State update(const State & current_state,
                       ControlInput const & input,
                       double dt) = 0;

protected:
  Controller() = default;
};

}  // namespace itl
