#pragma once

#include "Simulation.h"

namespace sim
{

/**
 * @class MpcSimulation
 * @brief
 *
 */
class MpcSimulation : public Simulation
{
public:
  ~MpcSimulation() = default;
  MpcSimulation() = default;

  /**
   * @brief
   *
   * @param dt
   */
  void update(double dt) override;
};

}  // namespace sim
