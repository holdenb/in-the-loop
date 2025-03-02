#include "SensorDecorator.h"

namespace itl
{

SensorDecorator::SensorDecorator(std::unique_ptr<Sensor> && sensor)
    : m_sensor(std::move(sensor)){};

}  // namespace itl
