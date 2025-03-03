#include "Sensor.h"

using namespace std::string_literals;

namespace itl
{

Sensor::Sensor()
    : m_update([](SensorResult) { return; }),
      m_log(std::make_unique<Logger>()){};

void Sensor::update(double dt)
{
  m_log->debug("Sensor Update"s);
  m_update(_update(dt));
}

void Sensor::listen(SensorUpdate sensor_update) { m_update = sensor_update; }

void Sensor::setLogger(std::unique_ptr<ILogger> && log)
{
  m_log = std::move(log);
}

}  // namespace itl
