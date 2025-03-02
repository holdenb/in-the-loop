#include "Sensor.h"

namespace itl
{

Sensor::Sensor() : m_update([](SensorResult) { return; }){};

void Sensor::update(double dt) { m_update(_update(dt)); }

void Sensor::listen(SensorUpdate sensor_update) { m_update = sensor_update; }

}  // namespace itl
