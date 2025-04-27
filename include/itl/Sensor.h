#pragma once

#include "Logger.h"
#include "SensorUpdate.h"

#include <memory>

using namespace util;

namespace itl
{

/**
 * @class Sensor
 * @brief
 *
 */
class Sensor
{
public:
  virtual ~Sensor() = default;

  /**
   * @brief
   *
   * @param dt
   */
  void update(double dt);

  /**
   * @brief
   *
   * @param sensor_update
   */
  void listen(SensorUpdate sensor_update);

  /**
   * @brief
   *
   * @param log
   */
  void setLogger(std::unique_ptr<ILogger> && log);

protected:
  Sensor();

  /**
   * @brief
   *
   * @param dt
   * @return
   */
  virtual SensorResult _update(double dt) = 0;

  /**
   * @brief
   */
  SensorUpdate m_update;

  /**
   * @brief
   */
  std::unique_ptr<ILogger> m_log;
};

}  // namespace itl
