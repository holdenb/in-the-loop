#pragma once

#include "Sensor.h"

#include <memory>

namespace itl
{

class SensorDecorator
{
public:
  ~SensorDecorator() = default;
  SensorDecorator(std::unique_ptr<Sensor> && sensor);

private:
  std::unique_ptr<Sensor> m_sensor;
};

}  // namespace itl
