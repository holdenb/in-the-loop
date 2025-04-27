#pragma once

#include "Sensor.h"

#include <memory>

namespace itl
{

/**
 * @class SensorDecorator
 * @brief
 *
 */
class SensorDecorator
{
public:
  ~SensorDecorator() = default;

  /**
   * @brief
   *
   * @param sensor
   */
  SensorDecorator(std::unique_ptr<Sensor> && sensor);

private:
  /**
   * @brief
   */
  std::unique_ptr<Sensor> m_sensor;
};

}  // namespace itl
