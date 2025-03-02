#pragma once

#include "Simulation.h"

namespace sim
{

class MpcSimulation : public Simulation
{
public:
  ~MpcSimulation();
  MpcSimulation();

  virtual void update(double dt) = 0;
};

}  // namespace sim
