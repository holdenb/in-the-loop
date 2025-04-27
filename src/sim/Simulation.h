#pragma once

namespace sim
{

/**
 * @class Simulation
 * @brief
 *
 */
class Simulation
{
public:
  virtual ~Simulation() = default;

  /**
   * @brief
   *
   * @param dt
   */
  virtual void update(double dt) = 0;

protected:
  Simulation() = default;
};

}  // namespace sim
