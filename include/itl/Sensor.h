#pragma once

#include "Logger.h"
#include "SensorUpdate.h"

#include <memory>

using namespace util;

namespace itl
{

class Sensor
{
public:
  virtual ~Sensor() = default;

  void update(double dt);
  void listen(SensorUpdate sensor_update);
  void setLogger(std::unique_ptr<ILogger> && log);

protected:
  Sensor();

  virtual SensorResult _update(double dt) = 0;

  SensorUpdate m_update;
  std::unique_ptr<ILogger> m_log;
};

}  // namespace itl
