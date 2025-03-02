#pragma once

namespace sim
{

class Simulation
{
public:
  virtual ~Simulation() = default;

  virtual void update(double dt) = 0;

protected:
  Simulation() = default;
};

}  // namespace sim
