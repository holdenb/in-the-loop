#pragma once

#include "SensorUpdate.h"

namespace itl
{

class Sensor
{
public:
  virtual ~Sensor() = default;

  void update(double dt);
  void listen(SensorUpdate sensor_update);

protected:
  Sensor();

  virtual SensorResult _update(double dt) = 0;

  SensorUpdate m_update;
};

}  // namespace itl
